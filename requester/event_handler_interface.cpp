#include "config.h"

#include "libpldm/requester/pldm.h"

#include "event_hander_interface.hpp"

#include <assert.h>
#include <systemd/sd-journal.h>

#include <nlohmann/json.hpp>
#include <sdbusplus/timer.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/exception.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/time.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>

using namespace pldm::utils;

#undef DEBUG
#define MAX_ATTEMPT 3

namespace pldm
{
EventHandlerInterface::EventHandlerInterface(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    pldm::dbus_api::Requester& requester,
    pldm::requester::Handler<pldm::requester::Request>* handler) :
    eid(eid),
    bus(bus), event(event), requester(requester), handler(handler),
    eventTimer(event, std::bind(&EventHandlerInterface::callback, this))
{
    _timer = std::make_unique<phosphor::Timer>([&](void) { timeoutHandler(); });
    startCallback();
}

void EventHandlerInterface::callback()
{
    exec();
}

void EventHandlerInterface::exec()
{
    if (!isProcessPolling)
    {
        requestPollData();
    }
}
/**
 *  outside uses to insert event only
 * */
int EventHandlerInterface::enqueue(uint16_t item)
{
    if (_eventQueue.size() < MAX_QUEUE_SIZE)
    {
        for (auto& i : _eventQueue)
        {
            if (i == item)
            {
                return -2;
            }
        }

        // prepare cache data
        auto element = _data.find(item);
        auto _tmp = std::make_unique<PollingInfo>();

        if (element == _data.end())
        {
            _tmp->operationFlag = PLDM_GET_FIRSTPART;
            _tmp->dataTransferHandle = item;
            _tmp->eventIdToAck = item;
            _tmp->eventClass = PLDM_MESSAGE_POLL_EVENT;
            _tmp->transferFlag = 0x0;
            _tmp->totalSize = 0;
            _tmp->eventDataCRC = 0;

            // save info into the map
            _data[item] = std::move(_tmp);
#ifdef DEBUG
            std::cout << "\nQUEUING NEXT EVENT_ID " << std::hex << item << "\n";
#endif
            // insert to event queue
            _eventQueue.push_back(item);
        }

        return 0;
    }

    return -1;
}

void EventHandlerInterface::timeoutHandler()
{
    if (!responseReceived)
    {
#ifdef DEBUG
        std::cerr << "TIMEOUT ...\n";
#endif
        uint16_t eventIdToAck = _eventQueue.front();
        if (retries[eventIdToAck] > MAX_ATTEMPT)
        {
#ifdef DEBUG
            std::cout << "HIT MAX RETRIES, DROP EVENT_ID " << std::hex
                      << eventIdToAck << "\n";
#endif
            // clear cached data
            retries[eventIdToAck] = 0;
            _data.extract(eventIdToAck);

            if (_eventQueue.size() > 0)
            {
                // pop the queue
                _eventQueue.pop_front();
                _eventQueue.shrink_to_fit();
            }
        }
        else
        {
            // increase retry counter
            retries[eventIdToAck] += 1;
        }

        // resend the package
        reset();
    }
}

void EventHandlerInterface::registerEventHandler(uint8_t eventClass,
                                                 HandlerFunc function)
{
    _eventHndls.emplace(eventClass, function);
}

void EventHandlerInterface::reset()
{
    isProcessPolling = false;
    requester.markFree(eid, instanceId);
    startCallback();
}

void EventHandlerInterface::processResponseMsg(mctp_eid_t /*eid*/,
                                               const pldm_msg* response,
                                               size_t respMsgLen)
{
    uint8_t retCompletionCode;
    uint8_t retTid{};
    uint16_t retEventId;
    uint32_t retNextDataTransferHandle{};
    uint8_t retTransferFlag{};
    uint8_t retEventClass{};
    uint32_t retEventDataSize{};
    uint32_t retEventDataIntegrityChecksum{};

    auto _tmp = std::make_unique<PollingInfo>();
    std::vector<uint8_t> tmp(respMsgLen, 0);

    auto rc = decode_poll_for_platform_event_message_resp(
        response, respMsgLen, &retCompletionCode, &retTid, &retEventId,
        &retNextDataTransferHandle, &retTransferFlag, &retEventClass,
        &retEventDataSize, tmp.data(), &retEventDataIntegrityChecksum);
    if (rc != PLDM_SUCCESS)
    {
#ifdef DEBUG
        std::cerr
            << "ERROR: Failed to decode_poll_for_platform_event_message_resp, rc = "
            << rc << std::endl;
#endif
        return;
    }

    retEventId &= 0xffff;
    retNextDataTransferHandle &= 0xffffffff;

#ifdef DEBUG
    std::cout << "\nRESPONSE: \n"
              << "retTid: " << std::hex << (unsigned)retTid << "\n"
              << "retEventId: " << std::hex << retEventId << "\n"
              << "retNextDataTransferHandle: " << std::hex
              << retNextDataTransferHandle << "\n"
              << "retTransferFlag: " << std::hex << (unsigned)retTransferFlag
              << "\n"
              << "retEventClass: " << std::hex << (unsigned)retEventClass
              << "\n"
              << "retEventDataSize: " << retEventDataSize << "\n"
              << "retEventDataIntegrityChecksum: " << std::hex
              << retEventDataIntegrityChecksum << "\n";
#endif

    if (retEventId == 0x0 || retEventId == 0xffff)
    {
        return;
    }

    auto element = _data.find(retEventId);
    if (element == _data.end())
    {
        // not found event id in store
        _tmp->TID = retTid;
        _tmp->operationFlag = PLDM_GET_NEXTPART;
        _tmp->dataTransferHandle = retNextDataTransferHandle;
        _tmp->eventIdToAck = retEventId;
        _tmp->eventClass = retEventClass;
        _tmp->transferFlag = retTransferFlag;
        _tmp->totalSize = retEventDataSize;
        _tmp->eventDataCRC = 0;
        _tmp->data.insert(_tmp->data.begin(), tmp.begin(),
                          tmp.begin() + retEventDataSize);

        // save info into the map
        _data[retEventId] = std::move(_tmp);

        if (_eventQueue.size() < MAX_QUEUE_SIZE)
        {
            for (auto& i : _eventQueue)
            {
                if (i == retEventId)
                {
                    return;
                }
            }
#ifdef DEBUG
            // insert to event queue
            std::cout << "QUEUING EVENT_ID=" << retEventId << "\n";
#endif
            _eventQueue.push_back(retEventId);
        }
        reset();
        return;
    }

    // drop if response eventId doesn't match with request eventId
    if (_eventQueue.front() > 0 && retEventId != _eventQueue.front())
    {
#ifdef DEBUG
        std::cerr << "WARNING: RESPONSED EVENT_ID DOESN'T MATCH WITH QUEUING\n"
                  << "retEventId=" << std::hex << retEventId
                  << " queue=" << _eventQueue.front()
                  << " size=" << _eventQueue.size() << "\n";
#endif
        return;
    }

    // announce that correct data is received
    responseReceived = true;
    _timer->stop();

    // reset the retry
    retries[retEventId] = 0;

    // found
    auto& entry = *element->second;
    int flag = static_cast<int>(retTransferFlag);
    if (flag == 0x1)
    {
        entry.dataTransferHandle = 0;
    }
    entry.operationFlag = PLDM_GET_NEXTPART;
    entry.transferFlag = retTransferFlag;

    try
    {
        if (entry.totalSize < entry.dataTransferHandle)
        {
            entry.data.resize(entry.dataTransferHandle, 0);
        }

        entry.totalSize += retEventDataSize;
        entry.data.insert(entry.data.begin() + entry.dataTransferHandle,
                          tmp.begin(), tmp.begin() + retEventDataSize);

        entry.dataTransferHandle = retNextDataTransferHandle;

#ifdef DEBUG
        std::cout << "\nEVENT_ID:" << entry.eventIdToAck
                  << " DATA LENGTH:" << entry.totalSize << "\n ";
        for (auto it = entry.data.begin(); it != entry.data.end(); it++)
        {
            std::cout << std::setfill('0') << std::setw(2) << std::hex
                      << (unsigned)*it << " ";
        }
        std::cout << "\n";
#endif
    }
    catch (const std::exception& e)
    {
        std::cerr << "WARNING: NO FUNCTION TO PROCESS DATA!!!\n" << std::endl;
    }

    if (flag == 0x4 || flag == 0x5)
    {
        entry.eventDataCRC = retEventDataIntegrityChecksum;
        uint32_t checksum = crc32(entry.data.data(), entry.data.size());

        if (checksum == entry.eventDataCRC)
        {
            try
            {
                // invoke class handler
                _eventHndls.at(retEventClass)(entry.TID, entry.eventClass,
                                              entry.eventIdToAck, entry.data);

                // clear cache data, avoid problem.
                retries[entry.eventIdToAck] = 0;
                entry.operationFlag = 0x0;
                entry.totalSize = 0x0;
                entry.dataTransferHandle = 0x0;
                entry.transferFlag = 0x0;
                entry.data.clear();
            }
            catch (const std::exception& e)
            {
                std::cerr << "ERROR:\n" << e.what() << std::endl;
            }
        }
        else
        {
            std::cerr << "\nchecksum isn't correct chks=" << std::hex
                      << checksum << " eventDataCRC=" << std::hex
                      << retEventDataIntegrityChecksum << "\n ";
        }

        entry.operationFlag = PLDM_ACKNOWLEDGEMENT_ONLY;
    }

    reset();
}

int EventHandlerInterface::requestPollData()
{
    std::vector<uint8_t> requestMsg(
        sizeof(pldm_msg_hdr) + PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE_REQ_BYTES);

    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());

    if (isProcessPolling)
    {
        return PLDM_ERROR;
    }

    uint16_t eventIdToAck = 0x0;
    uint8_t operationFlag = PLDM_GET_FIRSTPART;
    uint32_t dataTransferHandle = 0x0;

    // get the event id from queue
    if (_eventQueue.size() > 0)
    {
        eventIdToAck = _eventQueue.front();

#ifdef DEBUG
        std::cout << "\nGET EVENT_ID=" << std::hex << eventIdToAck
                  << " FROM QUEUE\n";
#endif
    }

    // find the data information
    auto element = _data.find(eventIdToAck);
    if (element != _data.end())
    {
        auto& entry = *element->second;
        if (entry.eventIdToAck != 0xffff)
        {
            operationFlag = entry.operationFlag;
            dataTransferHandle = entry.dataTransferHandle;
            eventIdToAck = entry.eventIdToAck;
        }
    }

    // if found event in queue
    if (eventIdToAck == 0)
    {
        operationFlag = PLDM_GET_FIRSTPART;
        dataTransferHandle = 0x0;
    }

#ifdef DEBUG
    std::cout << "\nREQUEST \n"
              << "operationFlag: " << std::hex << (unsigned)operationFlag
              << "\n"
              << "eventIdToAck: " << std::hex << eventIdToAck << "\n"
              << "dataTransferHandle: " << std::hex << dataTransferHandle
              << "\n";
#endif

    instanceId = requester.getInstanceId(eid);
    auto rc = encode_poll_for_platform_event_message_req(
        instanceId, 1, operationFlag, dataTransferHandle, eventIdToAck, request,
        PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE_REQ_BYTES);

    if (rc != PLDM_SUCCESS)
    {
        std::cerr
            << "ERROR: Failed to encode_poll_for_platform_event_message_req(1), rc = "
            << rc << std::endl;
        return PLDM_ERROR;
    }

    rc = handler->registerRequest(
        eid, instanceId, PLDM_PLATFORM, PLDM_POLL_FOR_EVENT_MESSAGE,
        std::move(requestMsg),
        std::move(
            std::bind_front(&EventHandlerInterface::processResponseMsg, this)));
    if (rc)
    {
        std::cerr << "ERROR: failed to send the poll request\n";
        requester.markFree(eid, instanceId);
        return PLDM_ERROR;
    }

    // flags settings
    isProcessPolling = true;
    responseReceived = false;
    _timer->start(usec);

    return PLDM_SUCCESS;
}

void EventHandlerInterface::startCallback()
{
    std::function<void()> callback(
        std::bind(&EventHandlerInterface::callback, this));
    try
    {
        eventTimer.restart(std::chrono::microseconds(interval));
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: cannot start callback" << std::endl;
        throw;
    }
}

void EventHandlerInterface::stopCallback()
{
    try
    {
        eventTimer.setEnabled(false);
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: cannot stop callback" << std::endl;
        throw;
    }
}

} // namespace pldm
