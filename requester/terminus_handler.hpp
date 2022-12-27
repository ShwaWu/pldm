#pragma once

#include "libpldm/fru.h"

#include "common/types.hpp"
#include "pldmd/dbus_impl_fru.hpp"
#include "pldmd/dbus_impl_requester.hpp"
#include "requester/handler.hpp"

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>

#include <map>

namespace pldm
{

namespace terminus
{

using namespace pldm::dbus_api;

using BitField8 = bitfield8_t;
using EntityType = uint16_t;

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
        pldm::dbus_api::Requester& requester,
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

  private:
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

    /** @brief reference to Requester object, primarily used to access API to
     *  obtain PLDM instance id.
     */
    pldm::dbus_api::Requester& requester;

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
};

} // namespace terminus

} // namespace pldm
