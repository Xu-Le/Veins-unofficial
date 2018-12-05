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

#ifndef __GPCR_H__
#define __GPCR_H__

#include "veins/modules/wave/BaseWaveApplLayer.h"
#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/routing/UnicastRoutingInterface.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"

/**
 * @brief A concrete routing protocol.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseWaveApplLayer
 * @see UnicastRoutingInterface
 * reference: Geographic Routing in City Scenarios, Mobile Computing and Communications Review, Volume 9, Number 1
 */
class GPCR : public BaseWaveApplLayer, private UnicastRoutingInterface
{
public:
	/** @name constructors, destructor. */
	///@{
	GPCR() : BaseWaveApplLayer() {}
	GPCR(unsigned stacksize) : BaseWaveApplLayer(stacksize) {}
	~GPCR();
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from lower layer. */
	void handleLowerMsg(cMessage *msg) override;
	/** @brief handle control messages from lower layer. */
	void handleLowerControl(cMessage *msg) override;

	/** @brief routing message decorate method. */
	void decorateRouting(RoutingMessage *routingMsg);

	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg);
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg);
	/** @brief call-back method of receiving data control message. */
	void onDataLost(DataMessage *lostDataMsg);

	/** @brief call a routing request to a certain receiver determined by routingPlanList. */
	void callRouting(long receiver);
	/** @brief initialize routingPlanList through configured xmlfile. */
	void initializeRoutingPlanList(cXMLElement *xmlConfig);

	/** @name unicast routing interfaces implementation. */
	///@{
	/** @brief Select a next hop from one-hop neighbors and deliver routing request to it, return -1 if there is no neighbors. */
	LAddress::L3Type selectNextHop(RoutingMessage *routingMsg) override;
	/** @brief Received a response message carried with routing path. */
	void onRoutingSuccess(RoutingMessage *routingMsg) override;
	/** @brief Received a response message indicating cannot find a routing path. */
	void onRoutingFail(RoutingMessage *routingMsg) override;
	/** @brief Check if the route path to destination is already maintained in the routing table. */
	bool checkRoutingTable(LAddress::L3Type destination) override;
	/** @brief List all the routing path maintained in the routing table. */
	void displayRoutingTable() override;
	/** @brief A reactive protocol creates a new route only when the existing one is broken. */
	void onPathBroken(RoutingMessage *routingMsg) override;
	///@}

private:
	int routingLengthBits; ///< the length of routing message measured in bits.
	int routingPriority;   ///< the priority of routing message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.
	int maxHopConstraint;  ///< the maximum of routing message hop count constraint.

	cMessage *callRoutingEvt;  ///< self message event used to call routing request to certain destination determined by routingPlanList.

	std::list<std::pair<double /* simtime */, LAddress::L3Type /* destination */> > routingPlanList; ///< routing plans of all vehicles configured by a xmlfile.
	RoutingTable routingTable; ///< typedef std::map<LAddress::L3Type, std::list<LAddress::L3Type> > RoutingTable.
};

#endif /* __GPCR_H__ */
