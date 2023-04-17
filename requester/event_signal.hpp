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
  private:
    std::unique_ptr<sdbusplus::bus::match_t> pldmEventSignal;
    std::unique_ptr<sdbusplus::bus::match_t> pldmNumericSensorEventSignal;
};


}
