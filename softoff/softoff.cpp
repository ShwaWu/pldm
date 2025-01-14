#include "softoff.hpp"

#include "common/instance_id.hpp"
#include "common/transport.hpp"
#include "common/utils.hpp"

#include <libpldm/entity.h>
#include <libpldm/platform.h>
#include <libpldm/state_set.h>

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/bus.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/exception.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/time.hpp>

#include <array>

PHOSPHOR_LOG2_USING;

namespace pldm
{
using namespace sdeventplus;
using namespace sdeventplus::source;
constexpr auto clockId = sdeventplus::ClockId::RealTime;
using Clock = Clock<clockId>;
using Timer = Time<clockId>;

constexpr pldm::pdr::TerminusID TID = 0; // TID will be implemented later.
namespace sdbusRule = sdbusplus::bus::match::rules;

SoftPowerOff::SoftPowerOff(sdbusplus::bus_t& bus, sd_event* event) :
    bus(bus), timer(event)
{
    getHostState();
    if (hasError || completed)
    {
        return;
    }

    auto rc = getEffecterID();
    if (completed)
    {
        error("pldm-softpoweroff: effecter to initiate softoff not found");
        return;
    }
    else if (rc != PLDM_SUCCESS)
    {
        hasError = true;
        return;
    }

    rc = getSensorInfo();
    if (rc != PLDM_SUCCESS)
    {
        error("Message get Sensor PDRs error. PLDM error code = {RC}", "RC",
              lg2::hex, static_cast<int>(rc));
        hasError = true;
        return;
    }

    // Matches on the pldm StateSensorEvent signal
    pldmEventSignal = std::make_unique<sdbusplus::bus::match_t>(
        bus,
        sdbusRule::type::signal() + sdbusRule::member("StateSensorEvent") +
            sdbusRule::path("/xyz/openbmc_project/pldm") +
            sdbusRule::interface("xyz.openbmc_project.PLDM.Event"),
        std::bind(std::mem_fn(&SoftPowerOff::hostSoftOffComplete), this,
                  std::placeholders::_1));
}

int SoftPowerOff::getHostState()
{
    try
    {
        pldm::utils::PropertyValue propertyValue =
            pldm::utils::DBusHandler().getDbusPropertyVariant(
                "/xyz/openbmc_project/state/host0", "CurrentHostState",
                "xyz.openbmc_project.State.Host");

        if ((std::get<std::string>(propertyValue) !=
             "xyz.openbmc_project.State.Host.HostState.Running") &&
            (std::get<std::string>(propertyValue) !=
             "xyz.openbmc_project.State.Host.HostState.TransitioningToOff"))
        {
            // Host state is not "Running", this app should return success
            completed = true;
            return PLDM_SUCCESS;
        }
    }
    catch (const std::exception& e)
    {
        error("PLDM host soft off: Can't get current host state: {ERROR}",
              "ERROR", e);
        hasError = true;
        return PLDM_ERROR;
    }

    return PLDM_SUCCESS;
}

void SoftPowerOff::hostSoftOffComplete(sdbusplus::message_t& msg)
{
    pldm::pdr::TerminusID msgTID;
    pldm::pdr::SensorID msgSensorID;
    pldm::pdr::SensorOffset msgSensorOffset;
    pldm::pdr::EventState msgEventState;
    pldm::pdr::EventState msgPreviousEventState;

    // Read the msg and populate each variable
    msg.read(msgTID, msgSensorID, msgSensorOffset, msgEventState,
             msgPreviousEventState);

    if (msgSensorID == sensorID && msgSensorOffset == sensorOffset &&
        msgEventState == PLDM_SW_TERM_GRACEFUL_SHUTDOWN)
    {
        // Receive Graceful shutdown completion event message. Disable the timer
        auto rc = timer.stop();
        if (rc < 0)
        {
            error("PLDM soft off: Failure to STOP the timer. ERRNO={RC}", "RC",
                  rc);
        }

        // This marks the completion of pldm soft power off.
        completed = true;
    }
}

int SoftPowerOff::getEffecterID()
{
    auto& bus = pldm::utils::DBusHandler::getBus();

    // VMM is a logical entity, so the bit 15 in entity type is set.
    pdr::EntityType entityType = PLDM_ENTITY_VIRTUAL_MACHINE_MANAGER | 0x8000;

    try
    {
        std::vector<std::vector<uint8_t>> VMMResponse{};
        auto VMMMethod = bus.new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.PDR", "FindStateEffecterPDR");
        VMMMethod.append(TID, entityType,
                         (uint16_t)PLDM_STATE_SET_SW_TERMINATION_STATUS);

        auto VMMResponseMsg = bus.call(VMMMethod, dbusTimeout);

        VMMResponseMsg.read(VMMResponse);
        if (VMMResponse.size() != 0)
        {
            for (auto& rep : VMMResponse)
            {
                auto VMMPdr =
                    reinterpret_cast<pldm_state_effecter_pdr*>(rep.data());
                effecterID = VMMPdr->effecter_id;
            }
        }
        else
        {
            VMMPdrExist = false;
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        error("PLDM soft off: Error get VMM PDR,ERROR={ERR_EXCEP}", "ERR_EXCEP",
              e.what());
        VMMPdrExist = false;
    }

    if (VMMPdrExist)
    {
        return PLDM_SUCCESS;
    }

    // If the Virtual Machine Manager PDRs doesn't exist, go find the System
    // Firmware PDRs.
    // System Firmware is a logical entity, so the bit 15 in entity type is set
    entityType = PLDM_ENTITY_SYS_FIRMWARE | 0x8000;
    try
    {
        std::vector<std::vector<uint8_t>> sysFwResponse{};
        auto sysFwMethod = bus.new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.PDR", "FindStateEffecterPDR");
        sysFwMethod.append(TID, entityType,
                           (uint16_t)PLDM_STATE_SET_SW_TERMINATION_STATUS);

        auto sysFwResponseMsg = bus.call(sysFwMethod, dbusTimeout);

        sysFwResponseMsg.read(sysFwResponse);

        if (sysFwResponse.size() == 0)
        {
            error("No effecter ID has been found that matches the criteria");
            return PLDM_ERROR;
        }

        for (auto& rep : sysFwResponse)
        {
            auto sysFwPdr =
                reinterpret_cast<pldm_state_effecter_pdr*>(rep.data());
            effecterID = sysFwPdr->effecter_id;
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        error("PLDM soft off: Error get system firmware PDR,ERROR={ERR_EXCEP}",
              "ERR_EXCEP", e.what());
        completed = true;
        return PLDM_ERROR;
    }

    return PLDM_SUCCESS;
}

int SoftPowerOff::getSensorInfo()
{
    pldm::pdr::EntityType entityType;

    entityType = VMMPdrExist ? PLDM_ENTITY_VIRTUAL_MACHINE_MANAGER
                             : PLDM_ENTITY_SYS_FIRMWARE;

    // The Virtual machine manager/System firmware is logical entity, so bit 15
    // need to be set.
    entityType = entityType | 0x8000;

    try
    {
        auto& bus = pldm::utils::DBusHandler::getBus();
        std::vector<std::vector<uint8_t>> Response{};
        auto method = bus.new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.PDR", "FindStateSensorPDR");
        method.append(TID, entityType,
                      (uint16_t)PLDM_STATE_SET_SW_TERMINATION_STATUS);

        auto ResponseMsg = bus.call(method, dbusTimeout);

        ResponseMsg.read(Response);

        if (Response.size() == 0)
        {
            error("No sensor PDR has been found that matches the criteria");
            return PLDM_ERROR;
        }

        pldm_state_sensor_pdr* pdr = nullptr;
        for (auto& rep : Response)
        {
            pdr = reinterpret_cast<pldm_state_sensor_pdr*>(rep.data());
            if (!pdr)
            {
                error("Failed to get state sensor PDR.");
                return PLDM_ERROR;
            }
        }

        sensorID = pdr->sensor_id;

        auto compositeSensorCount = pdr->composite_sensor_count;
        auto possibleStatesStart = pdr->possible_states;

        for (auto offset = 0; offset < compositeSensorCount; offset++)
        {
            auto possibleStates =
                reinterpret_cast<state_sensor_possible_states*>(
                    possibleStatesStart);
            auto setId = possibleStates->state_set_id;
            auto possibleStateSize = possibleStates->possible_states_size;

            if (setId == PLDM_STATE_SET_SW_TERMINATION_STATUS)
            {
                sensorOffset = offset;
                break;
            }
            possibleStatesStart += possibleStateSize + sizeof(setId) +
                                   sizeof(possibleStateSize);
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        error("PLDM soft off: Error get State Sensor PDR,ERROR={ERR_EXCEP}",
              "ERR_EXCEP", e.what());
        return PLDM_ERROR;
    }

    return PLDM_SUCCESS;
}

int SoftPowerOff::hostSoftOff(sdeventplus::Event& event)
{
    constexpr uint8_t effecterCount = 1;
    PldmTransport pldmTransport{};
    uint8_t instanceID;
    uint8_t mctpEID;

    mctpEID = pldm::utils::readHostEID();
    // TODO: fix mapping to work around OpenBMC ecosystem deficiencies
    pldm_tid_t pldmTID = static_cast<pldm_tid_t>(mctpEID);

    std::array<uint8_t, sizeof(pldm_msg_hdr) + sizeof(effecterID) +
                            sizeof(effecterCount) +
                            sizeof(set_effecter_state_field)>
        requestMsg{};
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    set_effecter_state_field stateField{
        PLDM_REQUEST_SET, PLDM_SW_TERM_GRACEFUL_SHUTDOWN_REQUESTED};
    pldm::InstanceIdDb instanceIdDb;
    instanceID = instanceIdDb.next(pldmTID);
    auto rc = encode_set_state_effecter_states_req(
        instanceID, effecterID, effecterCount, &stateField, request);
    if (rc != PLDM_SUCCESS)
    {
        instanceIdDb.free(pldmTID, instanceID);
        error("Message encode failure. PLDM error code = {RC}", "RC", lg2::hex,
              static_cast<int>(rc));
        return PLDM_ERROR;
    }

    // Add a timer to the event loop, default 30s.
    auto timerCallback = [=, this](Timer& /*source*/,
                                   Timer::TimePoint /*time*/) mutable {
        if (!responseReceived)
        {
            instanceIdDb.free(pldmTID, instanceID);
            error(
                "PLDM soft off: ERROR! Can't get the response for the PLDM request msg. Time out! Exit the pldm-softpoweroff");
            exit(-1);
        }
        return;
    };
    Timer time(event, (Clock(event).now() + std::chrono::seconds{30}),
               std::chrono::seconds{1}, std::move(timerCallback));

    // Add a callback to handle EPOLLIN on fd
    auto callback = [=, &pldmTransport, this](IO& io, int fd,
                                              uint32_t revents) mutable {
        if (fd != pldmTransport.getEventSource())
        {
            return;
        }

        if (!(revents & EPOLLIN))
        {
            return;
        }

        void* responseMsg = nullptr;
        size_t responseMsgSize{};
        pldm_tid_t srcTID = pldmTID;

        auto rc = pldmTransport.recvMsg(pldmTID, responseMsg, responseMsgSize);
        if (rc)
        {
            error("Soft off: failed to recv pldm data. PLDM RC = {RC}", "RC",
                  static_cast<int>(rc));
            return;
        }

        std::unique_ptr<void, decltype(std::free)*> responseMsgPtr{responseMsg,
                                                                   std::free};

        // We've got the response meant for the PLDM request msg that was
        // sent out
        io.set_enabled(Enabled::Off);
        auto response = reinterpret_cast<pldm_msg*>(responseMsgPtr.get());

        if (srcTID != pldmTID ||
            !pldm_msg_hdr_correlate_response(&request->hdr, &response->hdr))
        {
            /* This isn't the response we were looking for */
            return;
        }

        /* We have the right response, release the instance ID and process */
        io.set_enabled(Enabled::Off);
        instanceIdDb.free(pldmTID, instanceID);

        if (response->payload[0] != PLDM_SUCCESS)
        {
            error("Getting the wrong response. PLDM RC = {RC}", "RC",
                  (unsigned)response->payload[0]);
            exit(-1);
        }

        responseReceived = true;

        // Start Timer
        using namespace std::chrono;
        auto timeMicroseconds =
            duration_cast<microseconds>(seconds(SOFTOFF_TIMEOUT_SECONDS));

        auto ret = startTimer(timeMicroseconds);
        if (ret < 0)
        {
            error(
                "Failure to start Host soft off wait timer, ERRNO = {RET}. Exit the pldm-softpoweroff",
                "RET", ret);
            exit(-1);
        }
        else
        {
            error(
                "Timer started waiting for host soft off, TIMEOUT_IN_SEC = {TIMEOUT_SEC}",
                "TIMEOUT_SEC", SOFTOFF_TIMEOUT_SECONDS);
        }
        return;
    };
    IO io(event, pldmTransport.getEventSource(), EPOLLIN, std::move(callback));

    // Asynchronously send the PLDM request
    rc = pldmTransport.sendMsg(pldmTID, requestMsg.data(), requestMsg.size());
    if (0 > rc)
    {
        instanceIdDb.free(pldmTID, instanceID);
        error(
            "Failed to send message/receive response. RC = {RC}, errno = {ERR}",
            "RC", static_cast<int>(rc), "ERR", errno);
        return PLDM_ERROR;
    }

    // Time out or soft off complete
    while (!isCompleted() && !isTimerExpired())
    {
        try
        {
            event.run(std::nullopt);
        }
        catch (const sdeventplus::SdEventError& e)
        {
            instanceIdDb.free(pldmTID, instanceID);
            error(
                "PLDM host soft off: Failure in processing request.ERROR= {ERR_EXCEP}",
                "ERR_EXCEP", e.what());
            return PLDM_ERROR;
        }
    }

    return PLDM_SUCCESS;
}

int SoftPowerOff::startTimer(const std::chrono::microseconds& usec)
{
    return timer.start(usec);
}
} // namespace pldm
