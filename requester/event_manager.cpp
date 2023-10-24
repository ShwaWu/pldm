#include "common/types.hpp"
#include "common/utils.hpp"

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>
#include <phosphor-logging/lg2.hpp>

#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <libpldm/pldm.h>
#include <algorithm>
#include <map>
#include <string_view>
#include <vector>
#include "event_manager.hpp"

static constexpr uint16_t MCStateSensorID = 180;

namespace pldm
{

int EventManager::handleMessagePollEvent(const pldm_msg* request,
                                          size_t payloadLength,
                                          uint8_t /* formatVersion */,
                                          uint8_t tid,
                                          size_t eventDataOffset)
{
    lg2::info("received poll event tid={TID}", "TID", tid);

    uint8_t evtFormatVersion = 0;
    uint16_t evtId = 0;
    uint32_t evtDataTransferHandle = 0;
    auto eventData = reinterpret_cast<const uint8_t*>(request->payload) +
                     eventDataOffset;
    auto eventDataSize = payloadLength - eventDataOffset;

    auto rc = decode_pldm_message_poll_event_data(eventData, eventDataSize,
                                                  &evtFormatVersion, &evtId,
                                                  &evtDataTransferHandle);
    if (rc)
    {
        lg2::error("Failed to decode message poll event data, rc={RC}.", "RC",
                   rc);
        return rc;
    }

    if (devManager)
    {
        devManager->addEventMsg(tid, evtId, PLDM_MESSAGE_POLL_EVENT, 0);
    }
    return PLDM_SUCCESS;
}

int EventManager::handleSensorEvent(const pldm_msg* request,
                                          size_t payloadLength,
                                          uint8_t /* formatVersion */,
                                          uint8_t tid,
                                          size_t eventDataOffset)
{
    uint16_t sensorId = 0;
    uint8_t sensorEventClassType = 0;
    size_t eventClassDataOffset = 0;
    auto eventData = reinterpret_cast<const uint8_t*>(request->payload) +
                     eventDataOffset;
    auto eventDataSize = payloadLength - eventDataOffset;

    auto rc = decode_sensor_event_data(eventData, eventDataSize, &sensorId,
                                       &sensorEventClassType,
                                       &eventClassDataOffset);
    if (rc)
    {
        lg2::error("Failed to decode sensor event data, rc={RC}.", "RC",
                   rc);
        return rc;
    }
    switch (sensorEventClassType)
    {
        case PLDM_NUMERIC_SENSOR_STATE:
        {
            const uint8_t* sensorData = eventData + eventClassDataOffset;
            size_t sensorDataLength = eventDataSize - eventClassDataOffset;

            return processNumericSensorEvent(tid, sensorId, sensorData,
                                             sensorDataLength);
        }
        case PLDM_STATE_SENSOR_STATE:
        case PLDM_SENSOR_OP_STATE:
        default:
            lg2::info("unhandled sensor event, class type={CLASSTYPE}",
                      "CLASSTYPE", sensorEventClassType);
            return PLDM_ERROR;
    }
    return PLDM_SUCCESS;
}

int EventManager::processNumericSensorEvent(uint8_t tid, uint16_t sensorId,
                                            const uint8_t* sensorData,
                                            size_t sensorDataLength)
{
    uint8_t eventState = 0;
    uint8_t previousEventState = 0;
    uint8_t sensorDataSize = 0;
    uint32_t presentReading;
    auto rc = decode_numeric_sensor_data(sensorData, sensorDataLength,
                                         &eventState, &previousEventState,
                                         &sensorDataSize, &presentReading);
    if (rc)
    {
        return rc;
    }

    // RAS sensors
    if ((sensorId >= 191) && (sensorId <= 198))
    {
        if (devManager)
        {
            devManager->addEventMsg(tid, sensorId, PLDM_SENSOR_EVENT,
                             PLDM_NUMERIC_SENSOR_STATE);
        }
    }
    // MC State sensor
    else if (sensorId == MCStateSensorID)
    {
        handleMCStateSensorEvent(tid, sensorId, presentReading, eventState);
    }

    return PLDM_SUCCESS;
}

void EventManager::handleMCStateSensorEvent(uint8_t tid,
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

}
