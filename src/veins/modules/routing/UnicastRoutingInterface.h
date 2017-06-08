//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef __UNICASTROUTINGINTERFACE_H__
#define __UNICASTROUTINGINTERFACE_H__

#include "veins/modules/messages/RoutingMessage_m.h"

/**
 * @brief Interface class for unicast based routing protocol design and any concrete subclass should private inherit it.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRoutingLayer
 * reference: Routing Protocol in Intervehicle Communication Systems: A Survey, IEEE Communications Magazine, December 2011
 */
class UnicastRoutingInterface
{
public:
    /** @name constructor, destructor. */
    ///@{
    UnicastRoutingInterface() {}
    virtual ~UnicastRoutingInterface() {}
    ///@}

protected:
    /** @brief A recommended data structure of routing table. */
    typedef std::map<LAddress::L3Type, std::list<LAddress::L3Type> > RoutingTable;

    /** @name Common interfaces. */
    ///@{
    /** @brief Select a next hop from one-hop neighbors and deliver routing request to it, return -1 if there is no neighbors. */
    virtual LAddress::L3Type selectNextHop(RoutingMessage* wsm) = 0;
    /** @brief Received a response message carried with routing path. */
    virtual void onRoutingSuccess(RoutingMessage* wsm) = 0;
    /** @brief Received a response message indicating cannot find a routing path. */
    virtual void onRoutingFail(RoutingMessage* wsm) = 0;
    /**
     * @brief Check if the route path to destination is already maintained in the routing table.
     *
     * Note that the data structure of routing table is determined by concrete routing protocol, though this interface
     * has provided a recommended one, a parameter of destination should be sufficient.
     */
    virtual bool checkRoutingTable(LAddress::L3Type destination) = 0;
    /** @brief List all the routing path maintained in the routing table. */
    virtual void displayRoutingTable() = 0;
    ///@}

    /** @name Proactive protocol interfaces. */
    ///@{
    /** @brief A proactive routing protocol periodically creates and updates the new routes of each pair of vehicles. */
    virtual void periodicallyUpdate(std::list<LAddress::L3Type>& routingPath, int destination) {};
    /**
     * @brief Determine a optimal periodical period for updating routing paths.
     *
     * Too short periods make the protocol suffer from high overhead. Conversely, too long periods make the protocol
     * suffer from frequent route failures. Note that the parameters for determining optimal period shall be the
     * data members of concrete routing protocol class, so this function has no input parameters.
     */
    virtual simtime_t determineOptimalPeriod() { return simtime_t::ZERO; };
    ///@}

    /** @name Reactive protocol interfaces. */
    ///@{
    /** @brief A reactive protocol creates a new route only when the existing one is broken. */
    virtual void onPathBroken(RoutingMessage* wsm) {};
    ///@}

    /** @name Prediction-based protocol interfaces. */
    ///@{
    /** @brief Predicts a probability of route breaking and return its breaking probability. */
    virtual double predictBreakProbability(std::list<LAddress::L3Type>& existingPath) { return 0.0; };
    /**
     * @brief Searches for alternative routes before the communication is disrupted.
     *
     * If searched an alternative path, then update old one in the routing table with the new one and return true,
     * else keep the old one unchanged and return false.
     */
    virtual bool seekAlternativePath(std::list<LAddress::L3Type>& existingPath, int destination) { return false; };
    ///@}

    /** @name Opportunistic protocol interfaces. */
    ///@{
    /** @brief Notified that cannot find a reachable route when the only way is to store messages until a destination is reachable. */
    virtual void onRouteUnreachable(RoutingMessage* wsm) {};
    /** @brief Come across a opportunity and try to seize it. */
    virtual void seizeOpportunity(RoutingMessage* wsm) {};
    ///@}
};

#endif /* __UNICASTROUTINGINTERFACE_H__ */
