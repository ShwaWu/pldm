#include "config.h"

#include "pldm_message_poll_event.hpp"

#include "libpldm/requester/pldm.h"

#include <assert.h>
#include <systemd/sd-journal.h>

#include <nlohmann/json.hpp>
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
#include "cper.hpp"


#undef DEBUG
#define OEM_EVENT               0xaa
#define SENSOR_TYPE_OEM         0xF0

#define CPER_FORMAT_TYPE        0

namespace pldm
{

static std::unique_ptr<sdbusplus::bus::match_t> pldmEventSignal;

PldmMessagePollEvent::PldmMessagePollEvent(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    pldm::dbus_api::Requester& requester,
    pldm::requester::Handler<pldm::requester::Request>* handler) :
    EventHandlerInterface(eid, event, bus, requester, handler)
{
    // register event class handler
    registerEventHandler(PLDM_MESSAGE_POLL_EVENT,
                         [&](uint8_t TID, uint8_t eventClass, uint16_t eventID,
                             const std::vector<uint8_t> data) {
                             return pldmPollForEventMessage(TID, eventClass,
                                                            eventID, data);
                         });
    // TODO: update OEM class handler
    registerEventHandler(OEM_EVENT, [&](uint8_t TID, uint8_t eventClass,
                                        uint16_t eventID,
                                        const std::vector<uint8_t> data) {
        return pldmPollForEventMessage(TID, eventClass, eventID, data);
    });

    handlePldmDbusEventSignal();
}

static void addSELLog(uint8_t TID, uint16_t eventID, AmpereSpecData *p)
{
    std::vector<uint8_t> evtData;
    std::string message = "PLDM RAS SEL Event";
    uint8_t recordType;
    uint8_t evtData1, evtData2, evtData3, evtData4, evtData5, evtData6;
    uint8_t socket;

    /*
     * OEM IPMI SEL Recode Format for RAS event:
     * evtData1:
     *    Bit [7:4]: eventClass
     *        0xF: oemEvent for RAS
     *    Bit [3:1]: Reserved
     *    Bit 0: SocketID
     *        0x0: Socket 0 0x1: Socket 1
     * evtData2:
     *    Event ID, indicates RAS PLDM sensor ID.
     * evtData3:
     *     Bit [7:4]: Payload Type
     *     Bit [3:0]: Error Type ID - Bit [11:8]
     * evtData4:
     *     Error Type ID - Bit [7:0]
     * evtData5:
     *     Error Sub Type ID high byte
     * evtData6:
     *     Error Sub Type ID low byte
     */
    socket = (TID == 1) ? 0 : 1;
    recordType = 0xD0;
    evtData1 = SENSOR_TYPE_OEM | socket;
    evtData2 = eventID;
    evtData3 = (p->typeId >> 8) & 0xF;
    evtData4 = p->typeId & 0xFF;
    evtData5 = (p->subTypeId >> 8) & 0xFF;
    evtData6 = p->subTypeId & 0xFF;
    /*
     * OEM data bytes
     *    Ampere IANA: 3 bytes [0x3a 0xcd 0x00]
     *    event data: 9 bytes [evtData1 evtData2 evtData3
     *                         evtData4 evtData5 evtData6
     *                         0x00     0x00     0x00 ]
     *    sel type: 1 byte [0xC0]
     */
    evtData.push_back(0x3a);
    evtData.push_back(0xcd);
    evtData.push_back(0);
    evtData.push_back(evtData1);
    evtData.push_back(evtData2);
    evtData.push_back(evtData3);
    evtData.push_back(evtData4);
    evtData.push_back(evtData5);
    evtData.push_back(evtData6);
    evtData.push_back(0);
    evtData.push_back(0);
    evtData.push_back(0);

    auto& bus = pldm::utils::DBusHandler::getBus();
    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.Logging.IPMI", "/xyz/openbmc_project/Logging/IPMI",
            "xyz.openbmc_project.Logging.IPMI", "IpmiSelAddOem");
        method.append(message, evtData , recordType);

        auto selReply =bus.call(method);
        if (selReply.is_method_error())
        {
            std::cerr << "add SEL log error\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "call SEL log error: " << e.what() << "\n" ;
    }
}

int PldmMessagePollEvent::pldmPollForEventMessage(uint8_t TID,
                                                  uint8_t /*eventClass*/,
                                                  uint16_t eventID,
                                                  std::vector<uint8_t> data)
{
#ifdef DEBUG
    std::cout << "\nOUTPUT DATA\n";
    for (auto it = data.begin(); it != data.end(); it++)
    {
        std::cout << std::hex << (unsigned)*it << " ";
    }
    std::cout << "\n";
#endif

    //Open a stream to the CPER event data.
    FILE* stream = fmemopen((void*)data.data(), data.size(), "r");
    AmpereSpecData ampHdr;
    CommonEventData evtData;

    fseek(stream, 0, SEEK_SET);
    if (fread(&evtData, sizeof(CommonEventData), 1, stream) != 1) {
        std::cerr << "Invalid event data: Invalid length (log too short)." << "\n";
        return -1;
    }

    switch (evtData.formatType) {
    case CPER_FORMAT_TYPE:{
        /*TBD: Define CPER storage location */
        FILE* out = fopen("/tmp/cper.dump", "w");
        if (out == NULL)
        {
            std::cerr << "Can not open CPER storage\n";
            return -1;
        }
        decodeCperRecord(stream, &ampHdr, out);
        fclose(out);
    }break;
    //TBD: BERT, Diagnostic
    default:
        std::cerr << "Not support format type: " << evtData.formatType << "\n";
        break;
    }

    addSELLog(TID, eventID, &ampHdr);

    fclose(stream);
    return data.size();
}

void PldmMessagePollEvent::handlePldmDbusEventSignal()
{
    pldmEventSignal = std::make_unique<sdbusplus::bus::match_t>(
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
                try
                {
                    // add the priority
                    this->enqueueCriticalEvent(msgEventID);
                }
                catch (const std::exception& e)
                {
                    std::cerr << "subscribePldmDbusEventSignal failed\n"
                              << e.what() << std::endl;
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "subscribePldmDbusEventSignal failed\n"
                          << e.what() << std::endl;
            }
        });
}

} // namespace pldm
