#pragma once

#include "libpldm/base.h"
#include "libpldm/fru.h"
#include "libpldm/platform.h"

#include "common/types.hpp"
#include "common/utils.hpp"
#include "requester/handler.hpp"

#include <function2/function2.hpp>
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

namespace pldm
{

using HandlerFunc =
    std::function<int(uint8_t, uint8_t, uint16_t, const std::vector<uint8_t>)>;

class EventHandlerInterface
{
  public:
    explicit EventHandlerInterface(
        uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
        pldm::dbus_api::Requester& requester,
        pldm::requester::Handler<pldm::requester::Request>* handler);
    virtual ~EventHandlerInterface() = default;
    virtual void exec();
    virtual void callback();

    void registerEventHandler(uint8_t eventClass, HandlerFunc func);
    int enqueue(uint16_t item);
    void startCallback();
    void stopCallback();

  private:
    std::size_t MAX_QUEUE_SIZE = 256;
    uint64_t interval = 250000;
    uint64_t timeout = 1000000;
    bool isProcessPolling = false;
    uint8_t eid;
    sdbusplus::bus::bus& bus;
    sdeventplus::Event& event;
    pldm::dbus_api::Requester& requester;
    pldm::requester::Handler<pldm::requester::Request>* handler;
    std::chrono::microseconds usec{timeout};
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> eventTimer;

    int requestPollData();
    void processResponseMsg(mctp_eid_t eid, const pldm_msg* response,
                            size_t respMsgLen);

    void reset();
    void timeoutHandler();

    struct PollingInfo
    {
        uint8_t TID;
        uint8_t operationFlag;
        uint32_t dataTransferHandle;
        uint16_t eventIdToAck;
        uint8_t transferFlag;
        uint32_t eventDataCRC;
        uint8_t eventClass;
        uint32_t totalSize;
        uint8_t priority;
        std::vector<uint8_t> data;
    };

  protected:
    uint8_t instanceId;
    bool responseReceived = false;
    std::unique_ptr<phosphor::Timer> _timer;
    std::map<uint8_t, HandlerFunc> _eventHndls;
    std::deque<uint16_t> _eventQueue;
    std::unordered_map<uint16_t, std::unique_ptr<PollingInfo>> _data;
    std::unordered_map<uint16_t, int> retries;
};

} // namespace pldm
