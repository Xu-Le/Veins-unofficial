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
#include "veins/modules/routing/UnicastRoutingInterface.h"

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

	/** @brief wave short message decorate method. */
	void decorateWSM(WaveShortMessage *wsm) override;
	/** @brief call-back method of receiving beacon message. */
	void onBeacon(BeaconMessage *wsm) override;
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *wsm) override;
	/** @brief call-back method of receiving warning message. */
	void onWarning(WarningMessage *wsm) override;
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *wsm) override;
	/** @brief examine whether neighbors still in connected. */
	void examineNeighbors() override;

	/** @brief call a routing request to a certain receiver determined by routingPlanList. */
	void callRouting(long receiver) override;

	/** @name unicast routing interfaces implementation. */
	///@{
	/** @brief Select a next hop from one-hop neighbors and deliver routing request to it, return -1 if there is no neighbors. */
	LAddress::L3Type selectNextHop(RoutingMessage *wsm) override;
	/** @brief Received a response message carried with routing path. */
	void onRoutingSuccess(RoutingMessage *wsm) override;
	/** @brief Received a response message indicating cannot find a routing path. */
	void onRoutingFail(RoutingMessage *wsm) override;
	/** @brief Check if the route path to destination is already maintained in the routing table. */
	bool checkRoutingTable(LAddress::L3Type destination) override;
	/** @brief List all the routing path maintained in the routing table. */
	void displayRoutingTable() override;
	/** @brief A reactive protocol creates a new route only when the existing one is broken. */
	void onPathBroken(RoutingMessage *wsm) override;
	///@}

private:
	RoutingTable routingTable;
};

#endif /* __GPCR_H__ */
