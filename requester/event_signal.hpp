#pragma once

#include "common/types.hpp"
#include "common/utils.hpp"
#include "event_hander_interface.hpp"

#include <systemd/sd-journal.h>

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>

namespace pldm
{
namespace events
{
namespace sensors
{

class NumericSensorHanler
{
  public:
    NumericSensorHanler();
    virtual ~NumericSensorHanler() = default;
    NumericSensorHanler(const NumericSensorHanler&) = default;
    NumericSensorHanler& operator=(const NumericSensorHanler&) = default;
    NumericSensorHanler(NumericSensorHanler&&) = default;
    NumericSensorHanler& operator=(NumericSensorHanler&&) = default;

  private:
    void handleDbusEventSignalMatch();

    void handleBootOverallEvent([[maybe_unused]]uint8_t tid,
                [[maybe_unused]]uint16_t sensorId, uint32_t presentReading);

    void handlePCIeHotPlugEvent(uint8_t tid, [[maybe_unused]]uint16_t sensorId,
                uint32_t presentReading);

    void addJournalRecord(const std::string& message, std::uint8_t tid,
                          std::uint16_t sensorId, std::uint8_t eventState,
                          std::uint8_t preEventState, uint8_t sensorDataSize,
                          uint32_t presentReading)
    {
        try
        {
            sd_journal_send("MESSAGE=%s", message.c_str(), "TID=%d", tid,
                            "SENSOR_ID=%d", sensorId, "EVENT_STATE=%d",
                            eventState, "PRE_EVENT_STATE=%d", preEventState,
                            "SENSOR_DATA_SIZE=%d", sensorDataSize,
                            "PRESENT_READING=%d", presentReading);
        }
        catch (const std::exception& e)
        {
            std::cerr << "ERROR:\n" << e.what() << std::endl;
        }
    }
}; // class NumericSensorHanler
} // namespace sensors
} // namespace events
} // namespace pldm
