#pragma once

#include "libpldm/base.h"
#include "libpldm/fru.h"
#include "libpldm/platform.h"

#include "common/instance_id.hpp"
#include "common/types.hpp"
#include "common/utils.hpp"
#include "requester/handler.hpp"

#include <sdbusplus/timer.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>
#include <sdeventplus/utility/timer.hpp>

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <queue>
#include <vector>
#include <config.h>

namespace pldm
{

using HandlerFunc =
    std::function<int(uint8_t, uint8_t, uint16_t, const std::vector<uint8_t>)>;

class EventHandlerInterface
{
  public:
    explicit EventHandlerInterface(
        uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
        InstanceIdDb& instanceIdDb,
        pldm::requester::Handler<pldm::requester::Request>* handler);
    virtual ~EventHandlerInterface() = default;

    virtual void normalEventCb();
    virtual void criticalEventCb();
    void registerEventHandler(uint8_t event_class, HandlerFunc func);
    int enqueueCriticalEvent(uint16_t item);
    int enqueueOverflowEvent(uint16_t item);
    void addEventMsg(uint8_t eventId, uint8_t eventType, uint8_t eventClass);
    void startEventSignalPolling();
    void stopEventSignalPolling();
    bool areBMCRASQueuesEmpty()
    {
      if (critEventQueue.empty() && overflowEventQueue.empty())
      {
          return true;
      }
      return false;
    }

    bool areMProRASQueuesEmpty()
    {
      return mProRASQueuesAreEmpty;
    }

  private:
    std::size_t MAX_QUEUE_SIZE = 256;
    bool isProcessPolling = false;
    bool isPolling = false;
    bool isCritical = false;
    bool isInQuiesceMode = false;
    bool mProRASQueuesAreEmpty = false;
    uint8_t eid;
    sdbusplus::bus::bus& bus;
    sdeventplus::Event& event;
    /** @brief Instance ID database for managing instance ID*/
    InstanceIdDb& instanceIdDb;
    pldm::requester::Handler<pldm::requester::Request>* handler;
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> normEventTimer;
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> critEventTimer;
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> pollEventReqTimer;

#ifdef AMPERE
    bool isBertPolling = false;
#endif
    void processResponseMsg(mctp_eid_t eid, const pldm_msg* response,
                            size_t respMsgLen);
    void resetCacheAndFlags();
    void pollReqTimeoutHdl();
    void pollEventReqCb();
    void clearOverflow();
    void startCallback();
    void stopCallback();

    struct ReqPollInfo
    {
        uint8_t operationFlag;
        uint32_t dataTransferHandle;
        uint16_t eventIdToAck;
    };
    struct RecvPollInfo
    {
        uint8_t eventClass;
        uint32_t totalSize;
        std::vector<uint8_t> data;
    };

  protected:

    uint8_t instanceId;
    bool responseReceived = false;
    std::unique_ptr<sdbusplus::Timer> pollReqTimeoutTimer;
    std::map<uint8_t, HandlerFunc> eventHndls;
    std::deque<uint16_t> critEventQueue;
    std::deque<uint16_t> overflowEventQueue;
    ReqPollInfo reqData;
    RecvPollInfo recvData;
};

} // namespace pldm
