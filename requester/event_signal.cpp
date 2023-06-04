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

                if ((sensorId >= 191) && (sensorId <= 198))
                {
                    if (devManager)
                        devManager->addEventMsg(tid, sensorId, PLDM_SENSOR_EVENT,
                                         PLDM_NUMERIC_SENSOR_STATE);
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
