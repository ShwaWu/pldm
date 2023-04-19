#include "event_signal.hpp"
#include "common/types.hpp"
#include "common/utils.hpp"
#include "bert.hpp"

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

typedef struct
{
    std::string firstStt;
    std::string completedStt;
    std::string failStt;
} sensorDescription;

std::unordered_map<uint8_t, sensorDescription> numericNormalSensorDesTbl = {
    {0x90, {"SECpro booting", "SECpro completed", "SECpro boot failed"}},
    {0x91, {"Mpro booting", "Mpro completed", "Mpro boot failed"}},
    {0x92, {"ATF BL1 booting", "ATF BL1 completed", "ATF BL1 boot failed"}},
    {0x93, {"ATF BL2 booting", "ATF BL2 completed", "ATF BL2 boot failed"}},
    {0x94,
     {"DDR initialization started", "DDR initialization completed",
      "DDR initialization failed"}},
    {0x97, {"ATF BL31 booting", "ATF BL31 completed", "ATF BL31 boot failed"}},
    {0x98, {"ATF BL32 booting", "ATF BL32 completed", "ATF BL32 boot failed"}}};


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

void PldmDbusEventSignal::handleBootOverallEvent([[maybe_unused]]uint8_t tid,
                [[maybe_unused]]uint16_t sensorId, uint32_t presentReading)
{
    bool failFlg = false;
    std::stringstream strStream;
    std::string description;
    uint8_t byte3 = (presentReading & 0x000000ff);
    uint8_t byte2 = (presentReading & 0x0000ff00) >> 8;
    uint8_t byte1 = (presentReading & 0x00ff0000) >> 16;
    uint8_t byte0 = (presentReading & 0xff000000) >> 24;

    /*
     * Handle SECpro, Mpro, ATF BL1, ATF BL2, ATF BL31,
     * ATF BL32 and DDR initialization
     */
    if (numericNormalSensorDesTbl.find(byte3) !=
        numericNormalSensorDesTbl.end())
    {
        // Sensor report action is fail
        if (0x81 == byte2)
        {
            description +=
                numericNormalSensorDesTbl[byte3].failStt;
            failFlg = true;
        }
        else
        {
            // Sensor report action is complete
            if (0x01 == byte0)
            {
                description += numericNormalSensorDesTbl[byte3]
                                    .completedStt;
            }
            else // Action is running
            {
                description +=
                    numericNormalSensorDesTbl[byte3].firstStt;
            }
        }
    }

    // DDR trainning progress
    if (0x95 == byte3)
    {
        // Report Started DDR training
        if (0x00 == byte0)
        {
            description += "DDR training progress started";
        }
        // Report Completed DDR training
        else if (0x02 == byte0)
        {
            description += "DDR training progress completed";
        }
        // Report DDR training is progress
        else if (0x01 == byte0)
        {
            description += "DDR training in-progress " +
                            std::to_string(byte1) + "%";
        }
    }

    // Handle DDR training fail
    if ((0x96 == byte3) || (0x99 == byte3))
    {
        // The index of failed DIMMs are 0xbyte2.byte1.byte0
        uint32_t failDimmIdx = (uint32_t)byte0 |
                                ((uint32_t)byte1 << 8) |
                                ((uint32_t)byte2 << 16);
        failFlg = true;

        description +=
            (0x96 == byte3) ? "Socket 0:" : "Socket 1:";

        description += " Training progress failed at DIMMs:";

        for (uint32_t idx = 0; idx < 24; idx++)
        {
            if (failDimmIdx & ((uint32_t)1 << idx))
            {
                description += " #" + std::to_string(idx);
            }
        }
    }

    // Handle UEFI state reports
    if (byte3 <= 0x7f)
    {
        description = "ATF BL33 (UEFI) booting status = 0x";

        strStream
            << std::setfill('0') << std::hex
            << std::setw(sizeof(uint32_t) * 2) << presentReading
            << ", Status Class (0x"
            << std::setw(sizeof(uint8_t) * 2) << (uint32_t)byte3
            << "), Status SubClass (0x" << (uint32_t)byte2
            << "), Operation Code (0x"
            << std::setw(sizeof(uint16_t) * 2)
            << (uint32_t)((presentReading & 0xffff0000) >> 16)
            << ")" << std::dec;

        description += strStream.str();
    }
    // Log to Redfish event
    if (!description.empty())
    {
        std::string REDFISH_MESSAGE_ID =
            (true == failFlg)
                ? "OpenBMC.0.1.BIOSFirmwarePanicReason.Warning"
                : "OpenBMC.0.1.AmpereEvent.OK";

        sd_journal_send("MESSAGE=%s", description.c_str(),
                        "REDFISH_MESSAGE_ID=%s",
                        REDFISH_MESSAGE_ID.c_str(),
                        "REDFISH_MESSAGE_ARGS=%s",
                        description.c_str(), NULL);
    }
    /* Handler BERT flow in case host on. BMC should handshake
     * with Host to accessing SPI-NOR when UEFI boot complete.
     */
    if (failFlg)
        setHostStatus(HOST_FAILURE);
    else
        setHostStatus(HOST_BOOTING);
    if ((byte3 == 0x03) && (byte2 == 0x10) && isBertCheck())
    {
        std::cerr << "Host is on, UEFI boot complete."
                        "Read SPI to check valid BERT\n";
        checkValidBertRecord(HOST_ON);
        setBertCheck(false);
    }
}

void PldmDbusEventSignal::handlePCIeHotPlugEvent(uint8_t tid,
                [[maybe_unused]]uint16_t sensorId, uint32_t presentReading)
{
    /*
     * PresentReading value format
     * FIELD       |                   COMMENT
     * Bit 31      |   Reserved
     * Bit 30:24   |   Media slot number (0 - 63) This field can be used by UEFI
     *             |   to indicate the media slot number (such as NVMe/SSD slot)
     *             |   (7 bits)
     * Bit 23      |   Operation status: 1 = operation failed
     *             |   0 = operation successful
     * Bit 22      |   Action: 0 - Insertion 1 - Removal
     * Bit 21:18   |   Function (4 bits)
     * Bit 17:13   |   Device (5 bits)
     * Bit 12:5    |   Bus (8 bits)
     * Bit 4:0     |   Segment (5 bits)
     */
    uint8_t segment = (presentReading & 0x0000001f);
    uint8_t bus = (presentReading & 0x00001fe0) >> 5;
    uint8_t device = (presentReading & 0x0003e000) >> 13;
    uint8_t function = (presentReading & 0x003C0000) >> 18;
    uint8_t action = (presentReading & 0x00400000) >> 22;
    uint8_t operationStatus = (presentReading & 0x00800000) >> 23;
    uint8_t slot = (presentReading & 0x7f000000) >> 24;
    std::string sAction = (action == 0) ? "insertion" : "removal";
    std::string sOpStatus = (operationStatus == 0) ? "successful" : "failed";
    std::stringstream strStream;
    std::string description = "";

    description += (tid == 1) ? "SOCKET0 " : "SOCKET1 ";
    description += "PCIe Hot Plug ";
    description += "SENSOR: ";

    strStream
        << " Segment (0x" << std::setfill('0') << std::hex
        << std::setw(2) << segment
        << "), Bus (0x" << std::setfill('0') << std::hex
        << std::setw(2) << bus
        << "), Device (0x" << std::setfill('0') << std::hex
        << std::setw(2) << device
        << "), Function (0x" << std::setfill('0') << std::hex
        << std::setw(2) << function
        << "), Action (" << sAction
        << "), Operation status (" << sAction
        << "), Media slot number (" << std::dec << slot
        << ")";

    description += strStream.str();
    if (!description.empty())
    {
        std::string REDFISH_MESSAGE_ID =
            (0 == operationStatus)
                ? "OpenBMC.0.1.AmpereEvent"
                : "OpenBMC.0.1.AmpereWarning";

        sd_journal_send("MESSAGE=%s", description.c_str(),
                        "REDFISH_MESSAGE_ID=%s",
                        REDFISH_MESSAGE_ID.c_str(),
                        "REDFISH_MESSAGE_ARGS=%s",
                        description.c_str(), NULL);
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

                if ((sensorId >= 191) && (sensorId <= 198))
                {
                    if (devManager)
                        devManager->addEventMsg(tid, sensorId, PLDM_SENSOR_EVENT,
                                         PLDM_NUMERIC_SENSOR_STATE);
                }

                /*
                 * Handle Overall sensor
                 */
                if (sensorId == 175)
                {
                    handleBootOverallEvent(tid, sensorId, presentReading);
                }
                else if (sensorId == 169)
                {
                    handlePCIeHotPlugEvent(tid, sensorId, presentReading);
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
