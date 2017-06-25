//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>, 2016 Xu Le <xmutongxinXuLe@163.com>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef __BASEWAVEAPPLLAYER_H__
#define __BASEWAVEAPPLLAYER_H__

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/cellular/BaseStation.h"
#include "veins/modules/messages/BeaconMessage_m.h"
#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/WarningMessage_m.h"
#include "veins/modules/messages/ContentMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/messages/PacketExpiredMessage_m.h"
#include "veins/modules/messages/WaitTimeElapsedMessage_m.h"
#include "veins/modules/routing/RoutingUtils.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"
#include "veins/modules/routing/WarningStatisticCollector.h"
#include "veins/modules/routing/MobilityObserver.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"

/**
 * @brief WAVE application layer base class.
 *
 * @author David Eckhoff, Xu Le
 *
 * @defgroup routingLayer network layer for routing
 *
 * @see BaseApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 * @see TraCIMobility
 * @see TraCICommandInterface
 */
class BaseWaveApplLayer : public BaseApplLayer
{
public:
	/** @brief The message kinds this layer uses. */
	enum WaveApplMsgKinds {
		/** Stores the id on which classes extending WaveAppl should continue their own message kinds.*/
		FIRST_WAVE_APPL_MESSAGE_KIND = LAST_BASE_APPL_MESSAGE_KIND,
		SEND_BEACON_EVT,
		EXAMINE_NEIGHBORS_EVT,
		CALL_ROUTING_EVT,
		CALL_WARNING_EVT,
		CALL_CONTENT_EVT,
		FORGET_MEMORY_EVT,
		RECYCLE_GUID_EVT,
		PACKET_EXPIRES_EVT,
		WAIT_TIME_ELAPSED_EVT,
		LAST_WAVE_APPL_MESSAGE_KIND
	};
	/** @brief The control message kinds this layer uses. */
	enum WaveApplControlKinds {
		/** Stores the id on which classes extending WaveAppl should continue their own control kinds.*/
		LAST_WAVE_APPL_CONTROL_KIND = LAST_BASE_APPL_CONTROL_KIND,
	};

	/** @name constructors, destructor. */
	///@{
	BaseWaveApplLayer() : BaseApplLayer() {}
	BaseWaveApplLayer(unsigned stacksize) : BaseApplLayer(stacksize) {}
	virtual ~BaseWaveApplLayer();
	///@}

	virtual void initialize(int stage) override;
	virtual void finish() override;
	virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;

protected:
	/** @brief Called every time a message arrives(template method, subclass should not override it). */
	virtual void handleMessage(cMessage *msg) override;

	/** @brief handle self messages. */
	virtual void handleSelfMsg(cMessage* msg) override;
	/** @brief handle messages from below(template method, subclass should not override it). */
	virtual void handleLowerMsg(cMessage* msg) override;
	/** @brief Handle wireless cellular incoming messages. */
	virtual void handleCellularMsg(CellularMessage *cellularMsg) {}

	/** @brief wave short message factory method(template method, subclass should not override it). */
	virtual WaveShortMessage* prepareWSM(std::string name, int dataLength, t_channel channel, int priority, int serial);
	/** @brief wave short message decorate method. */
	virtual void decorateWSM(WaveShortMessage* wsm);
	/** @brief wave short message send method. */
	virtual void sendWSM(WaveShortMessage* wsm);
	/** @brief call-back method of receiving beacon message. */
	virtual void onBeacon(BeaconMessage* wsm);
	/** @brief call-back method of receiving routing message(aims to unicast protocols). */
	virtual void onRouting(RoutingMessage* wsm) = 0;
	/** @brief call-back method of receiving warning message(aims to broadcast and geocast protocols). */
	virtual void onWarning(WarningMessage* wsm) = 0;
	/** @brief call-back method of receiving content message(aims to download application). */
	virtual void onContent(ContentMessage* wsm) = 0;
	/** @brief call-back method of receiving data message. */
	virtual void onData(DataMessage* wsm) = 0;
	/** @brief examine whether neighbors still in connected. */
	virtual void examineNeighbors();
	/** @brief forget packets received long time ago. */
	virtual void forgetMemory();

	/** @name mobility relevant methods. */
	///@{
	/** @brief called by template method handleMobilityUpdate() when this vehicle has just cross a intersection without changing driving direction. */
	virtual void onStraight() {}
	/** @brief called by template method handleMobilityUpdate() when this vehicle has just turned left. */
	virtual void onTurnLeft() {}
	/** @brief called by template method handleMobilityUpdate() when this vehicle has just turned right. */
	virtual void onTurnRight() {}
	/** @brief called by template method handleMobilityUpdate() when this vehicle has just turned around. */
	virtual void onTurnAround() {}
	/** @brief update vehicle's mobility information that is read from SUMO(template method). */
	void handleMobilityUpdate(cObject* obj);
	/** @brief update(register or unregister) vehicle's NIC when it parks or resumes driving. */
	void handleParkingUpdate(cObject* obj);
	///@}

	/** @brief call a routing request to certain receiver determined by routingPlanList(aims to unicast protocols). */
	virtual void callRouting(LAddress::L3Type receiver);
	/** @brief call a warning notify to certain direction determined by warningPlanList(aims to broadcast and geocast protocols). */
	virtual void callWarning(double distance);
	/** @brief call a content request for certain size determined by contentPlanList(aims to download application). */
	virtual void callContent(int size);
	/** @brief initialize routingPlanList through configured xmlfile(aims to unicast protocols). */
	void initializeRoutingPlanList(cXMLElement* xmlConfig);
	/** @brief initialize warningPlanList through configured xmlfile(aims to broadcast and geocast protocols). */
	void initializeWarningPlanList(cXMLElement* xmlConfig);
	/** @brief initialize contentPlanList through configured xmlfile(aims to download application). */
	void initializeContentPlanList(cXMLElement* xmlConfig);

	/** @brief send beacon message(template method). */
	void sendBeacon();
	/** @brief return the number of target vehicles in ROI of the warning message. */
	long targetVehicles(bool plusX, Coord xMin, Coord xMax, LAddress::L3Type& farthestOne);

	/** @brief called by template method handleMobilityUpdate(). */
	virtual void calculateVehicleGap() {}

protected:
	/** @brief The class to store neighbor's information collected by beacon message. */
	class NeighborInfo
	{
	public:
		NeighborInfo(double a, Coord& p, Coord& s, Coord& f, Coord& t, simtime_t ra) : type(0), angle(a), pos(p), speed(s), from(f), to(t), receivedAt(ra) {}

		short type;   ///< reserved, defined by derived class since concrete protocol may define different types of neighbors, derived class should define an unknown type whose value equals 0.
		double angle; ///< current driving angle of the neighbor.
		Coord pos;    ///< current position of the neighbor.
		Coord speed;  ///< current speed of the neighbor.
		Coord from;   ///< the tail of road where the neighbor driving from.
		Coord to;     ///< the head of road where the neighbor driving to.
		simtime_t receivedAt; ///< the time received the most recently beacon message from the neighbor.
	};

	/** @name gate ID. */
	///@{
	int cellularIn; ///< receive packets from cellular base station.
	///@}

	/** @name POD types. */
	///@{
	bool isParking;        ///< whether vehicle is parked.
	bool sendWhileParking; ///< whether send messages when vehicle is parked.
	bool sendBeacons;      ///< whether send beacons periodically.
	bool callRoutings;     ///< whether send routing requests.
	bool callWarnings;     ///< whether send warning notifies when emergent incident happens.
	bool callContents;     ///< whether send data to other vehicles.
	bool dataOnSch;        ///< whether send data on service channel.
	int beaconLengthBits;  ///< the length of beacon message measured in bits.
	int beaconPriority;    ///< the priority of beacon message.
	int routingLengthBits; ///< the length of routing message measured in bits.
	int routingPriority;   ///< the priority of routing message.
	int warningLengthBits; ///< the length of warning message measured in bits.
	int warningPriority;   ///< the priority of warning message.
	int contentLengthBits; ///< the length of content message measured in bits.
	int contentPriority;   ///< the priority of content message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.
	int maxHopConstraint;  ///< the maximum of routing message hop count constraint.
	double transmissionRadius;   ///< the biggest transmission distance of transmitter.
	double beaconInterval; ///< the interval of sending beacon message.
	double examineNeighborsInterval; ///< the interval of examining the connectivity with neighbors.
	double forgetMemoryInterval; ///< the interval of forgetting message received too long time ago.
	double neighborElapsed; ///< the maximum time haven't receive message from neighbors leading to assume lose connectivity with it.
	double memoryElapsed;   ///< the maximum time can a message store in memory.
	///@}

	simtime_t maxStoreTime; ///< the maximum time a relay vehicle will store a routing message before discarding it.
	simtime_t guidUsedTime; ///< the maximum time from a GUID's allocated time to its recycled time.

	std::string laneId;     ///< the lane of this vehicle driving on.
	Coord fromRoadhead;     ///< the tail of road where this vehicle driving from.
	Coord toRoadhead;       ///< the head of road where this vehicle driving to.
	Coord curPosition;      ///< current position of this vehicle.
	Coord curSpeed;         ///< current speed of this vehicle.
	double curAngle;        ///< current driving angle measured in rad of this vehicle.
	double oldAngle;        ///< before turn direction, driving angle measured in rad of this vehicle.

	/** @name containers. */
	///@{
	std::list<int> guidUsed; ///< record GUID used before for recycle purpose.
	std::list<std::pair<double /* simtime */, LAddress::L3Type /* destination */> > routingPlanList; ///< routing plans of all vehicles configured by a xmlfile.
	std::list<std::pair<double /* simtime */, double /* distance */> > warningPlanList; ///< warning plans of all vehicles configured by a xmlfile.
	std::list<std::pair<double /* simtime */, int /* size */> > contentPlanList; ///< content plans of all vehicles configured by a xmlfile.
	std::map<LAddress::L3Type, NeighborInfo*> neighbors; ///< a map from a vehicle to all its neighbor mobility info.
	std::map<int /* GUID */, WaveShortMessage*> messageMemory; ///< a map from a message's GUID to the point to this message.
	std::map<LAddress::L3Type, NeighborInfo*>::iterator itN; ///< an iterator used to traverse container neighbors.
	///@}

	/** @name messages. */
	///@{
	cMessage *sendBeaconEvt;       ///< self message event used to periodically send beacons.
	cMessage *examineNeighborsEvt; ///< self message event used to examine the connectivity with neighbors.
	cMessage *callRoutingEvt;  ///< self message event used to call routing request to certain destination determined by routingPlanList.
	cMessage *callWarningEvt;  ///< self message event used to call warning notify to certain direction determined by warningPlanList.
	cMessage *callContentEvt;  ///< self message event used to call content request for certain size determined by contentPlanList.
	cMessage *forgetMemoryEvt; ///< self message event used to periodically forget message received too long time ago in memory.
	cMessage *recycleGUIDEvt;  ///< self message event used to periodically recycle GUIDs allocated before.
	std::map<simtime_t, PacketExpiredMessage*> packetExpiresEvts; ///< self message events used to discard expired packets which exceed maxStoreTime from receiving it.
	///@}

	/** @name TraCI mobility interfaces. */
	///@{
	Veins::TraCIMobility *mobility;
	Veins::TraCICommandInterface *traci;
	Veins::TraCICommandInterface::Vehicle *traciVehicle;
	Veins::TraCICommandInterface::Lane *traciLane;
	Veins::AnnotationManager *annotations;
	WaveAppToMac1609_4Interface *myMac;
	///@}

	static const simsignalwrap_t mobilityStateChangedSignal;
	static const simsignalwrap_t parkingStateChangedSignal;
};

#endif /* __BASEWAVEAPPLLAYER_H__ */
