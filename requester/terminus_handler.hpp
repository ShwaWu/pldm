#pragma once

#include "libpldm/fru.h"

#include "common/types.hpp"
#include "pldmd/dbus_impl_fru.hpp"
#include "pldmd/instance_id.hpp"
#include "requester/handler.hpp"
#include "requester/pldm_message_poll_event.hpp"
#include "sensors/pldm_sensor.hpp"

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>
#include <sdeventplus/utility/timer.hpp>

#include <unistd.h>
#include <map>

namespace pldm
{

namespace terminus
{

using namespace pldm::dbus_api;
using namespace pldm::sensor;

using BitField8 = bitfield8_t;
using EntityType = uint16_t;
using Length8bs = uint8_t;
using BaseUnit = uint8_t;
using UnitModifier = int8_t;
using OccurrenceRate = uint8_t;
using PldmSensorValue = double;
using Name = std::string;

using PDRList = std::vector<std::vector<uint8_t>>;

/** @struct PLDMSupportedCommands
 *  @brief PLDM supported commands
 */
struct PLDMSupportedCommands
{
    BitField8 cmdTypes[32];
};

/** @struct PldmDeviceInfo
 *  @brief PLDM terminus info
 *  @details Include EID, TID, supported PLDM types, supported PLDM commands of
 *  each type
 */
struct PldmDeviceInfo
{
    uint8_t eid;
    uint8_t tid;
    BitField8 supportedTypes[8];
    PLDMSupportedCommands supportedCmds[PLDM_MAX_TYPES];
};

/** @struct PldmSensorInfo
 *
 *  Structure representing PLDM Sensor Info
 */
struct PldmSensorInfo
{
    EntityType entityType;
    pdr::EntityInstance entityInstance;
    pdr::ContainerID containerId;
    Length8bs sensorNameLength; // 0 indicate no name
    BaseUnit baseUnit;
    UnitModifier unitModifier;
    PldmSensorValue offset;
    PldmSensorValue resolution;
    OccurrenceRate occurrenceRate;
    BitField8 rangeFieldSupport;
    PldmSensorValue warningHigh;
    PldmSensorValue warningLow;
    PldmSensorValue criticalHigh;
    PldmSensorValue criticalLow;
    PldmSensorValue fatalHigh;
    PldmSensorValue fatalLow;
    PldmSensorValue maxSetTable;
    PldmSensorValue minSetTable;
    PldmSensorValue maxValue;
    PldmSensorValue minValue;
    Name sensorName;
};

/** @class TerminusHandler
 *  @brief This class can fetch and process PDRs from host firmware
 *  @details Provides an API to fetch PDRs from the host firmware. Upon
 *  receiving the PDRs, they are stored into the BMC's primary PDR repo.
 *  Adjustments are made to entity association PDRs received from the host,
 *  because they need to be assimilated into the BMC's entity association
 *  tree. A PLDM event containing the record handles of the updated entity
 *  association PDRs is sent to the host.
 */
class TerminusHandler
{
  public:
    using TerminusInfo =
        std::tuple<pdr::TerminusID, pdr::EID, pdr::TerminusValidity>;
    using TLPDRMap = std::map<pdr::TerminusHandle, TerminusInfo>;
    using PDRList = std::vector<std::vector<uint8_t>>;

    /** @brief Constructor
     *  @param[in] eid - MCTP EID of host firmware
     *  @param[in] event - reference of main event loop of pldmd
     *  @param[in] repo - pointer to BMC's primary PDR repo
     *  @param[in] eventsJsonDir - directory path which has the config JSONs
     *  @param[in] entityTree - Pointer to BMC and Host entity association tree
     *  @param[in] bmcEntityTree - pointer to BMC's entity association tree
     *  @param[in] requester - reference to Requester object
     *  @param[in] handler - PLDM request handler
     */
    explicit TerminusHandler(
        uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
        pldm_pdr* repo, pldm_entity_association_tree* entityTree,
        pldm_entity_association_tree* bmcEntityTree,
        InstanceIdDb& instanceIdDb,
        pldm::requester::Handler<pldm::requester::Request>* handler);

    TerminusHandler(const TerminusHandler&) = delete;
    TerminusHandler& operator=(const TerminusHandler&) = delete;
    TerminusHandler(TerminusHandler&&) = delete;
    TerminusHandler& operator=(TerminusHandler&&) = delete;

    ~TerminusHandler();

    /** @brief check whether terminus is running when pldmd starts
     */
    bool isTerminusOn()
    {
        return responseReceived;
    }

    /** @brief Update EID to Name string mapping for the terminus
     *
     *  @param[in] eidMap - the pair of mapping
     *
     *  @return - true
     *
     */
    bool udpateEidMapping(std::pair<bool, std::string> eidMap)
    {
        eidToName = eidMap;
        return true;
    }

    /** @brief Discovery new terminus
     *
     * @return - none
     */
    requester::Coroutine discoveryTerminus();

    /** @brief Start timer to get sensor info
     *
     *  @param - none
     *
     *  @return - none
     *
     */
    void startSensorsPolling();

    /** @brief Stop timer to get sensor info and discovery
     *
     *  @param - none
     *
     *  @return - none
     *
     */
    void stopSensorsPolling();

    /** @brief Set terminus handler flag to false to stop polling or discovery
     *
     *  @param[in] none
     *
     *  @return - none
     *
     */
    void stopTerminusHandler();

    /** @brief Add received event message to terminus handler
     *
     *  @param[in] tid - Terminus ID
     *  @param[in] eventID - Event ID
     *  @param[in] eventType - type of event
     *  @param[in] eventClass - Event Class of sensor event
     *
     *  @return - none
     *
     */
    void addEventMsg(uint8_t tid, uint8_t eventId, uint8_t eventType,
                     uint8_t eventClass);

    /** @brief Enter quiesce mode after polling all remaining RAS events
     *  @details Stop hang detection service, sensor and event polling
     *  after finishing polling the remaining RAS events. First, start
     *  a timer to wait for RAS polling completion.
     *
     *  @param - none
     *
     *  @return - none
     **/
    void startQuiesceMode();

    /** @brief Restart sensor and event polling
     *
     *  @param - none
     *
     *  @return - none
     *
     */
    void restartSensorAndEventPolling();

    /** @brief Get TID of this terminus handler
     *
     *  @param - none
     *
     *  @return - none
     *
     */
    uint8_t getTid()
    {
        return devInfo.tid;
    }

    void notifyFWUpdateFailure()
    {
        fwUpdateFailed = true;
    }

  private:
    /* DSP0248 Table1 PLDM monitoring and control data types */
    #define PLDM_STR_UTF_8_MAX_LEN 256
    #define PLDM_STR_UTF_16_MAX_LEN 256

    using mapped_type = std::tuple<uint16_t, ObjectInfo>;
    /* sensor_key tuple of eid, sensorId, pdr_type */
    using sensor_key = std::tuple<uint8_t, uint16_t, uint8_t>;
    using SensorState = std::map<sensor_key, mapped_type>;
    using NameLanguageTag = std::string;
    using SensorName = std::string;
    /* aux_name_key is pair of handler and sensorId */
    using auxNameKey = std::tuple<uint16_t, uint16_t>;
    /* names list of one state/effecter sensor */
    using auxNameList = std::vector<std::tuple<std::string, std::string>>;
    /* sensor name index map to names list*/
    using auxNameSensorMapping = std::vector<auxNameList>;
    /* auxNameKey to sensor auxNameList */
    using auxNameMapping = std::map<auxNameKey, auxNameSensorMapping>;

    /** @brief getPLDMTypes for every device in MCTP Control D-Bus interface
     */
    requester::Coroutine getPLDMTypes();

    /** @brief Get TID of remote MCTP Endpoint
     */
    requester::Coroutine getTID();

    /** @brief Get supported PLDM commands of the terminus for every supported
     *  PLDM type
     */
    requester::Coroutine getPLDMCommands();
    requester::Coroutine getPLDMCommand(const uint8_t& pldmTypeIdx);

    /** @brief whether terminus support PLDM command type
     */
    bool supportPLDMType(const uint8_t pldmType);

    /** @brief whether terminus support PLDM command of a PLDM type
     */
    bool supportPLDMCommand(const uint8_t type, const uint8_t command);

    /** @brief Get current system time in milliseconds
     */
    std::string getCurrentSystemTime();

    /** @brief SetDateTime if device support SetDateTime
     */
    requester::Coroutine setEventReceiver();

    /** @brief SetDateTime if device support SetDateTime
     */
    requester::Coroutine setDateTime();

    /** @brief Get FRU Record Table from remote MCTP Endpoint
     *  @param[in] total - Total number of record in table
     */
    requester::Coroutine getFRURecordTable(const uint16_t& total);

    /** @brief Get FRU Record Table Metadata from remote MCTP Endpoint
     */
    requester::Coroutine getFRURecordTableMetadata(uint16_t* total);

    /** @brief Parse record data from FRU table
     *  @param[in] fruData - pointer to FRU record table
     *  @param[in] fruLen - FRU table length
     */
    void parseFruRecordTable(const uint8_t* fruData, size_t& fruLen);

    /** @brief this function sends a GetPDR request to Host firmware.
     *  And processes the PDRs based on type
     *
     *  @param[in] - nextRecordHandle - the next record handle to ask for
     */
    requester::Coroutine getDevPDR(uint32_t nextRecordHandle = 0);

    /** @brief Merge host firmware's entity association PDRs into BMC's
     *  @details A merge operation involves adding a pldm_entity under the
     *  appropriate parent, and updating container ids.
     *  @param[in] pdr - entity association pdr
     */
    void mergeEntityAssociations(const std::vector<uint8_t>& pdr);

    /** @brief Find parent of input entity type, from the entity association
     *  tree
     *  @param[in] type - PLDM entity type
     *  @param[out] parent - PLDM entity information of parent
     *  @return bool - true if parent found, false otherwise
     */
    bool getParent(const EntityType& type, pldm_entity* parent);

    /** @brief process the Host's PDR and add to BMC's PDR repo
     *  @param[in] eid - MCTP id of Host
     *  @param[in] response - response from Host for GetPDR
     *  @param[in] respMsgLen - response message length
     */
    requester::Coroutine processDevPDRs(mctp_eid_t& eid,
                                        const pldm_msg* response,
                                        size_t& respMsgLen,
                                        uint32_t* nextRecordHandle);

    /** @brief Parse comback numeric sensor PDRs and create the sensor D-Bus
     *  objects
     *
     *  @param[in] effecterPDRs - device effecter PDRs
     *
     */
    void createCompactNummericSensorIntf(const PDRList& sensorPDRs);

    /** @brief Parse numeric effecter PDRs and create the effecter sensor D-Bus
     *  objects
     *
     *  @param[in] effecterPDRs - device effecter PDRs
     *
     */
    void createNummericEffecterDBusIntf(const PDRList& effecterPDRs);

    void parseSensorAuxiliaryNamesPDR(const std::vector<uint8_t>& pdrData);

    /** @brief Parse aux name PDRs and populate the aux name mapping
     *         lookup data structure
     *
     *  @param[in] auxNamePDRs - device effecter aux name PDRs
     *
     */
    void parseAuxNamePDRs(const PDRList& auxNamePDRs);

    /** @brief Start reading the sensors info process
     *
     *  @param - none
     *
     *  @return - none
     *
     */
    void pollSensors();

    /** @brief Start reading the sensors info process
     *
     *  @param - none
     *
     *  @return - none
     *
     */
    void readSensor();

    /** @brief Send the getSensorReading request to get sensor info
     *
     *  @param[in] sensor_id - Sensor ID
     *  @param[in] pdr_type - PDR type
     *
     *  @return - none
     *
     */
    void getSensorReading(uint16_t sensor_id, uint8_t pdr_type);

    /** @brief Process response data from the getSensorReading request
     *
     *  @param[in] response - response message
     *  @param[in] respMsgLen - response message length
     *
     *  @return - none
     *
     */
    void processSensorReading(mctp_eid_t, const pldm_msg* response,
                              size_t respMsgLen);

    /** @brief Remove the sensor which response OperationState as not enabled
     *  in GetSensorReading command
     *
     *  @param[in] vKeys - List of sensor keys
     *
     *  @return - none
     *
     */
    void removeUnavailableSensor(const std::vector<sensor_key>& vKeys);

    /** @brief Remove the effecter from polling list after first reading
     *
     *  @details Because the effecter is not changed after power on the
     *  terminus. It can only changed by user thru updating the value property
     *  of effecter D-Bus interface. This actions is covered in those PLDM code 
     *
     *  @param[in] vKeys - List of sensor keys
     *
     *  @return - none
     *
     */
    void removeEffecterFromPollingList(const std::vector<sensor_key>& vKeys);

    /** @brief Create/Update list of sensor keys which will be polling
     *
     *  @param[in] none
     *
     *  @return - none
     *
     */
    void updateSensorKeys();

    /** @brief Start waiting for MPro recovery from impactless update.
    *
    *   @details MPro's state is read by executing ampere_request_mpro_state script.
    *   If FW_BOOT_OK asserts within 60s, wait for MCTP interface. If MCTP interface
    *   from MPro is ready within 60s, restart sensor and event polling and hang
    *   detection service. Otherwise, do nothing.
    *
    *   @param[in] none
    *
    *   @return - none
    */
    void waitForMProRecovery();

    /** @brief Wait to retrieve normal operation after impactless update.
     *
     *  @details Acknowledge impactless firmware update to MPro by
     *  setting value to effecter MC Control to MPro. From now,
     *  start a timer to wait for MPro recovery by watching FW_BOOT_OK GPIO
     *  assertion and MCTP Interface from MPro.
     *
     *  @param[in] none
     *
     *  @return - none
     */
    requester::Coroutine waitForImpactlessUpdateRecovery();

    /** @brief Start waiting for RAS polling completion:
     *  1. Stop sensor polling
     *  2. Stop event polling
     *  3. Write to MC Control Effecter (effecterId = 254) to acknowledge host firmware update
     *
     *  @param[in] none
     *
     *  @return - none
     */
    void waitForRASPollingFinished();

    /** @brief Set Numeric Effecter Value
     *
     *  @param[in] effecterId - Effecter ID
     *  @param[in] effecterDataSize - Effecter data size
     *  @param[in] effecterValue - pointer of the value buffer
     *
     *  @return - none
     *
     */
    requester::Coroutine setNumericEffecterValue(uint16_t effecterId,
                    uint8_t effecterDataSize, const uint8_t* effecterValue);
    

    /** @brief map that captures various terminus information **/
    TLPDRMap tlPDRInfo;

    /** @brief MCTP EID of host firmware */
    uint8_t eid;
    /** @brief reference of main D-bus interface of pldmd terminus */
    sdbusplus::bus::bus& bus;

    /** @brief reference of main event loop of pldmd, primarily used to schedule
     *  work.
     */
    sdeventplus::Event& event;
    /** @brief pointer to BMC's primary PDR repo, host PDRs are added here */
    pldm_pdr* repo;

    /** @brief Pointer to BMC's and Host's entity association tree */
    pldm_entity_association_tree* entityTree;

    /** @brief Pointer to BMC's entity association tree */
    pldm_entity_association_tree* bmcEntityTree;

    /** @brief Instance ID database for managing instance ID*/
    InstanceIdDb& instanceIdDb;

    /** @brief PLDM request handler */
    pldm::requester::Handler<pldm::requester::Request>* handler;

    /** @brief whether response received from Host */
    bool responseReceived;

    /** @brief The basic info of terminus such as EID, TID, supported PLDM types
     *  supported PLDM commands for each types.
     */
    PldmDeviceInfo devInfo;

    /** @brief Mapping the terminus ID with the terminus name */
    std::pair<bool, std::string> eidToName;

    /** @brief Map of the object FRU */
    std::unordered_map<uint8_t, std::shared_ptr<pldm::dbus_api::FruReq>> frus;

    /** @brief Print in when GetPDR */
    bool debugGetPDR = true;

    /** @brief The start time of one measuring process */
    std::chrono::_V2::system_clock::time_point startTime =
        std::chrono::system_clock::now();
    /** @brief Measuring time */
    int readCount = 0;

    /** @brief maps an entity type to parent pldm_entity from the BMC's entity
     *  association tree
     */
    std::map<EntityType, pldm_entity> parents;

    /** @brief List of compack numeric sensor PDRs */
    PDRList compNumSensorPDRs{};
    /** @brief List of numeric effecter AUX Name PDRs */
    PDRList effecterAuxNamePDRs{};
    /** @brief List of numeric effecter PDRs */
    PDRList effecterPDRs{};

    /** @brief Terminus handle */
    uint16_t terminusHandle = 0;
    /** @brief List of mapping form effecter key to effecter name */
    auxNameMapping _auxNameMaps;
    /** @brief DBus object state. */
    SensorState _state;

    /** @brief Store the specifications of sensor objects */
    std::map<sensor_key, std::unique_ptr<PldmSensor>> _sensorObjects;
    /** @brief List of numeric effecter keys */
    std::vector<sensor_key> _effecterLists;
    /** @brief Identify the D-Bus interface for the sensors is created */
    bool createdDbusObject = false;
    /* The point to the reading sensors */
    std::vector<sensor_key>::iterator sensorKey;
    std::vector<sensor_key> sensorKeys;
    std::vector<sensor_key> unavailableSensorKeys;
    /** @brief Poll sensor timer. Reset after each poll-sensor-timer-interval
     *  milliseconds. poll-sensor-timer-interval is package configuration.
     */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer;

    /** @brief Sleep timer between PLDM GetSensorReading commands.
     *  @details Because the performance and the responding time for
     *  GetSensorReading of terminus can be different. The sleep interval
     *  between GetSensorReading commands will be added. This value can be
     *  configured thru sleep-between-get-sensor-reading option. The option is
     *  milliseconds. The default value is 10 milliseconds
     */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer2;

    /** @brief Timer to wait for all RAS polling completion in impactless
     *  update quiesce mode.
     */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer3;

    /** @brief Timer to wait for MPro recovery after impactless update.
     */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer4;
    /** @brief Polling sensor flag. True when pldmd is polling sensor values */
    bool pollingSensors = false;
    /** @brief Enable the measurement in polling sensors */
    bool debugPollSensor = true;
    /** @brief whether response received for getsensorreading pldm command  */
    bool sendingPldmCommand = false;
    bool continuePollSensor = false;
    std::shared_ptr<PldmMessagePollEvent> eventDataHndl;
    /** @brief the flag to stop polling or discoverying */
    bool stopTerminusPolling = false;
    /** @brief counter to wait for RAS polling completion */
    uint16_t countNum = 0;
    /** @brief Flag to indicate Impactless Update Failure */
    bool fwUpdateFailed = false;

    enum MProState
    {
        MProQuiesce = 0,
        MProDown,
        MProUp,
        MCTPReady,
        MProReady
    };

    /** @brief  MPro state during impactless update*/
    MProState mProState = MProState::MProDown;
};

} // namespace terminus

} // namespace pldm
