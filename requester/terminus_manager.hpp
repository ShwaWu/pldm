#pragma once

#include "pldmd/dbus_impl_requester.hpp"
#include "requester/handler.hpp"
#include "requester/request.hpp"
#include "requester/terminus_handler.hpp"

#include <sdeventplus/event.hpp>

#include <map>
#include <vector>

namespace pldm
{

namespace terminus
{

using namespace pldm::dbus_api;
using namespace pldm::terminus;

/** @class Manager
 *
 * This class manages the PLDM terminus
 */
class Manager
{

  public:
    Manager() = delete;
    Manager(const Manager&) = delete;
    Manager(Manager&&) = delete;
    Manager& operator=(const Manager&) = delete;
    Manager& operator=(Manager&&) = delete;
    ~Manager() = default;

    /** @brief Constructor
     *
     *  @param[in] handler - PLDM request handler
     */
    explicit Manager(
        sdbusplus::bus::bus& bus, sdeventplus::Event& event, pldm_pdr* repo,
        pldm_entity_association_tree* entityTree,
        pldm_entity_association_tree* bmcEntityTree,
        pldm::dbus_api::Requester& requester,
        pldm::requester::Handler<pldm::requester::Request>* handler) :
        bus(bus),
        event(event), repo(repo), entityTree(entityTree),
        bmcEntityTree(bmcEntityTree), requester(requester), handler(handler)
    {}

    /** @brief Add the discovered MCTP endpoints to the managed devices list
     *
     *  @param[in] eids - Array of MCTP endpoints
     *
     *  @return None
     */
    void addDevices(const std::vector<mctp_eid_t>& eids)
    {
        for (const auto& it : eids)
        {
            std::cerr << "Adding terminus EID : " << unsigned(it) << std::endl;

            auto dev = std::make_unique<TerminusHandler>(
                it, event, bus, repo, entityTree, bmcEntityTree, requester,
                handler);
            dev->discoveryTerminus();
            mDevices[it] = std::move(dev);
        }
        return;
    }

    /** @brief Remove the MCTP devices from the managed devices list
     *
     *  @param[in] eids - Array of MCTP endpoints
     *
     *  @return None
     */
    void removeDevices(const std::vector<mctp_eid_t>& eids)
    {
        for (const auto& it : eids)
        {
            std::cerr << "Removing Device EID : " << unsigned(it) << std::endl;
            if (!mDevices.count(it))
            {
                continue;
            }
            mDevices.erase(it);
        }
        return;
    }

  private:
    /** @brief reference of main D-bus interface of pldmd devices */
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

    std::map<mctp_eid_t, std::unique_ptr<TerminusHandler>> mDevices;
};

}; // namespace terminus

} // namespace pldm
