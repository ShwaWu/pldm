#include "event_signal.hpp"
#include "common/types.hpp"
#include "common/utils.hpp"

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>

#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <libpldm/pldm.h>
#include <algorithm>
#include <map>
#include <string_view>
#include <vector>

static constexpr uint16_t MCStateSensorID = 180;

namespace pldm
{

void PldmDbusEventSignal::PldmMessagePollEventSignal()
{
    pldmMessagePollEventSignal = std::make_unique<sdbusplus::bus::match_t>(
         pldm::utils::DBusHandler::getBus(),
         sdbusplus::bus::match::rules::type::signal() +
             sdbusplus::bus::match::rules::member("PldmMessagePollEvent") +
             sdbusplus::bus::match::rules::path("/xyz/openbmc_project/pldm") +
             sdbusplus::bus::match::rules::interface(
                 "xyz.openbmc_project.PLDM.Event"),
         [&](sdbusplus::message::message& msg) {
             try
             {
                 uint8_t msgTID{};
                 uint8_t msgEventClass{};
                 uint8_t msgFormatVersion{};
                 uint16_t msgEventID{};
                 uint32_t msgEventDataTransferHandle{};

                 // Read the msg and populate each variable
                 msg.read(msgTID, msgEventClass, msgFormatVersion, msgEventID,
                          msgEventDataTransferHandle);
 #ifdef DEBUG
                 std::cout << "\n->Coming DBUS Event Signal\n"
                           << "TID: " << std::hex << (unsigned)msgTID << "\n"
                           << "msgEventClass: " << std::hex
                           << (unsigned)msgEventClass << "\n"
                           << "msgFormatVersion: " << std::hex
                           << (unsigned)msgFormatVersion << "\n"
                           << "msgEventID: " << std::hex << msgEventID << "\n"
                           << "msgEventDataTransferHandle: " << std::hex
                           << msgEventDataTransferHandle << "\n";
 #endif
                 // add the priority
                 if (devManager)
                     devManager->addEventMsg(msgTID, msgEventID, PLDM_MESSAGE_POLL_EVENT,
                                      0);
             }
             catch (const std::exception& e)
             {
                 std::cerr << "subscribePldmDbusEventSignal failed\n"
                           << e.what() << std::endl;
             }
         });
}

void PldmDbusEventSignal::handleMCStateSensorEvent(uint8_t tid,
                [[maybe_unused]]uint16_t sensorId, uint32_t presentReading, [[maybe_unused]]uint8_t eventState)
{
    /*
     * PresentReading value format
     * FIELD       |                   COMMENT
     * Bit 31:16   |   Indicate status of the last FW update operation (16 bits):
     *             |   0 = Operation successful
     *             |   1 = Operation failed due to BMC acknowledgement failure
     *             |   2 = Operation failed due to internal hardware error condition
     *             |   3 = Operation failed due to firmware error condition                   
     * Bit 15:3    |   Reserved (13 bits)
     * Bit 2       |   Indicates FW update complete (1 bit)
     * Bit 1       |   Reserved (1 bit)
     * Bit 0       |   Indicates FW update initiated (1 bit)
     */

    std::cerr << "DEBUG: HandleMCStateSensorEvent presentReading is " << unsigned(presentReading) << "\n";

    uint8_t fwUpdateInitiated = (presentReading & 0x00000001);
    uint8_t fwUpdateComplete = (presentReading & 0x00000004) >> 2;
    uint16_t lastFWUpdateStatus = (presentReading & 0xFFFF0000) >> 16;

    std::stringstream strStream;
    std::string description = "";
    description += "IMPACTLESS UPDATE: ";

    if (0x01 == fwUpdateInitiated)
    {
        strStream << "TID " << unsigned(tid) << " - Firmware Update Initiated";
        description += strStream.str();
        if (!description.empty())
        {
            std::string REDFISH_MESSAGE_ID = "OpenBMC.0.1.AmpereEvent";

            sd_journal_send("MESSAGE=%s", description.c_str(),
                            "REDFISH_MESSAGE_ID=%s",
                            REDFISH_MESSAGE_ID.c_str(),
                            "REDFISH_MESSAGE_ARGS=%s",
                            description.c_str(), NULL);
        }
        devManager->startQuiesceMode(tid);
    }
    else if (0x01 == fwUpdateComplete)
    {
        if (0x00 == lastFWUpdateStatus)
        {
            strStream << "TID " << unsigned(tid) << " - Firmware Update SUCCEEDED";
            description += strStream.str();
            if (!description.empty())
            {
                std::string REDFISH_MESSAGE_ID = "OpenBMC.0.1.AmpereEvent";

                sd_journal_send("MESSAGE=%s", description.c_str(),
                                "REDFISH_MESSAGE_ID=%s",
                                REDFISH_MESSAGE_ID.c_str(),
                                "REDFISH_MESSAGE_ARGS=%s",
                                description.c_str(), NULL);
            }
        }
        else
        {
            strStream << "TID " << unsigned(tid) << " - Firmware Update FAILED";

            switch (lastFWUpdateStatus)
            {
                case 0x01:
                strStream << " - BMC Acknowledgement failure";
                break;
                case 0x02:
                strStream << " - Internal hardware error";
                break;
                case 0x03:
                strStream << " - Firmware error";
                break;
                default:
                strStream << " - Unknown error";
            }

            description += strStream.str();
            if (!description.empty())
            {
                std::string REDFISH_MESSAGE_ID = "OpenBMC.0.1.AmpereCritical";

                sd_journal_send("MESSAGE=%s", description.c_str(),
                                "REDFISH_MESSAGE_ID=%s",
                                REDFISH_MESSAGE_ID.c_str(),
                                "REDFISH_MESSAGE_ARGS=%s",
                                description.c_str(), NULL);
            }
            devManager->notifyFWUpdateFailure(tid);
        }
    }
}

void PldmDbusEventSignal::PldmNumericSensorEventSignal()
{
    pldmNumericSensorEventSignal = std::make_unique<sdbusplus::bus::match_t>(
        pldm::utils::DBusHandler::getBus(),
        sdbusplus::bus::match::rules::type::signal() +
            sdbusplus::bus::match::rules::member("NumericSensorEvent") +
            sdbusplus::bus::match::rules::path("/xyz/openbmc_project/pldm") +
            sdbusplus::bus::match::rules::interface(
                "xyz.openbmc_project.PLDM.Event"),
        [&](sdbusplus::message::message& msg) {
            try
            {
                uint8_t tid{};
                uint16_t sensorId{};
                uint8_t eventState{};
                uint8_t preEventState{};
                uint8_t sensorDataSize{};
                uint32_t presentReading{};

                /*
                 * Read the information of event
                 */
                msg.read(tid, sensorId, eventState, preEventState,
                         sensorDataSize, presentReading);
                // RAS sensors
                if ((sensorId >= 191) && (sensorId <= 198))
                {
                    if (devManager)
                        devManager->addEventMsg(tid, sensorId, PLDM_SENSOR_EVENT,
                                         PLDM_NUMERIC_SENSOR_STATE);
                }
                // MC State sensor
                else if (sensorId == MCStateSensorID)
                {
                    handleMCStateSensorEvent(tid, sensorId, presentReading, eventState);
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "handleDbusEventSignalMatch failed\n"
                          << e.what() << std::endl;
            }
        });
}

PldmDbusEventSignal::PldmDbusEventSignal(
        terminus::Manager *dev): devManager(dev)
{
    PldmMessagePollEventSignal();
    PldmNumericSensorEventSignal();
}

}
