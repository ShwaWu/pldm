#pragma once

#include "requester/terminus_manager.hpp"

#include <sdbusplus/bus/match.hpp>

namespace pldm
{

class PldmDbusEventSignal
{
  public:
    PldmDbusEventSignal() = delete;
    PldmDbusEventSignal(const PldmDbusEventSignal&) = delete;
    PldmDbusEventSignal(PldmDbusEventSignal&&) = delete;
    PldmDbusEventSignal& operator=(const PldmDbusEventSignal&) = delete;
    PldmDbusEventSignal& operator=(PldmDbusEventSignal&&) = delete;
    ~PldmDbusEventSignal() = default;

    explicit PldmDbusEventSignal(terminus::Manager *dev);

    void PldmMessagePollEventSignal();
    void PldmNumericSensorEventSignal();
    void handleMCStateSensorEvent(uint8_t tid, [[maybe_unused]]uint16_t sensorId,
                uint32_t presentReading, [[maybe_unused]]uint8_t eventState);

  private:

    std::unique_ptr<sdbusplus::bus::match_t> pldmMessagePollEventSignal;
    std::unique_ptr<sdbusplus::bus::match_t> pldmNumericSensorEventSignal;
    terminus::Manager *devManager;
};


}
