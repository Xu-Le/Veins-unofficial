//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>, 2016-2018 Xu Le <xmutongxinXuLe@163.com>
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
#include "veins/modules/utility/Utils.h"
#include "veins/modules/cellular/BaseStation.h"
#include "veins/modules/messages/BeaconMessage_m.h"
#include "veins/modules/routing/MobilityObserver.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"

/**
 * @brief WAVE application layer base class.
 *
 * @author David Eckhoff, Xu Le
 *
 * @defgroup waveAppLayer application layer over WAVE
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
		FORGET_MEMORY_EVT,
		RECYCLE_GUID_EVT,
		PACKET_EXPIRES_EVT,
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
	virtual void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from lower layer. */
	virtual void handleLowerMsg(cMessage *msg) override;
	/** @brief Handle wireless cellular incoming messages. */
	virtual void handleCellularMsg(CellularMessage *cellularMsg) {}

	/** @brief wave short message factory method. */
	void prepareWSM(WaveShortMessage *wsm, int dataLength, t_channel channel, int priority, LAddress::L2Type recipient=-1);
	/** @brief wave short message send method. */
	void sendWSM(WaveShortMessage *wsm);
	/** @brief send beacon message. */
	void sendBeacon();
	/** @brief beacon message decorate method. */
	virtual void decorateBeacon(BeaconMessage *beaconMsg) {}
	/** @brief call-back method of receiving beacon message. */
	virtual void onBeacon(BeaconMessage *beaconMsg);

	/** @brief examine whether neighbors still in connected. */
	virtual void examineNeighbors();
	/** @brief forget packets received long time ago. */
	virtual void forgetMemory();
	/** @brief continuously failed to receive beacon message from the neighbor. */
	virtual void onNeighborLost() {}

	/** @name mobility relevant methods. */
	///@{
	/** @brief update vehicle's mobility information that is read from SUMO(template method). */
	void handleMobilityUpdate(cObject* obj);
	/** @brief update(register or unregister) vehicle's NIC when it parks or resumes driving. */
	void handleParkingUpdate(cObject* obj);
	///@}

protected:
	/** @brief The class to store neighbor's information collected by beacon message. */
	class NeighborInfo
	{
	public:
		NeighborInfo(Coord& p, Coord& s, simtime_t ra) : type(0), pos(p), speed(s), receivedAt(ra) {}

		short type;   ///< reserved, defined by derived class since concrete protocol may define different types of neighbors, derived class should define an unknown type whose value equals 0.
		Coord pos;    ///< current position of the neighbor.
		Coord speed;  ///< current speed of the neighbor.
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
	bool dataOnSch;        ///< whether send data on service channel.
	int beaconLengthBits;  ///< the length of beacon message measured in bits.
	int beaconPriority;    ///< the priority of beacon message.
	double transmissionRadius; ///< the biggest transmission distance of transmitter.
#if ROUTING_DEBUG_LOG
	double nextBeaconInstant;  ///< time instant of next sendBeaconEvt.
	double nextExamineInstant; ///< time instant of next examineNeighborsEvt.
#endif
	///@}

	simtime_t beaconInterval; ///< the interval of sending beacon message.
	simtime_t examineNeighborsInterval; ///< the interval of examining the connectivity with neighbors.
	simtime_t forgetMemoryInterval; ///< the interval of forgetting message received too long time ago.
	simtime_t neighborElapsed; ///< the maximum time haven't receive message from neighbors leading to assume lose connectivity with it.
	simtime_t memoryElapsed;   ///< the maximum time can a message store in memory.
	simtime_t maxStoreTime; ///< the maximum time a relay vehicle will store a routing message before discarding it.
	simtime_t guidUsedTime; ///< the maximum time from a GUID's allocated time to its recycled time.

	Coord curPosition;      ///< current position of this vehicle.
	Coord curSpeed;         ///< current speed of this vehicle.

	/** @name containers. */
	///@{
	std::list<int> guidUsed; ///< record GUID used before for recycle purpose.
	std::map<LAddress::L3Type, NeighborInfo*> neighbors; ///< a map from a vehicle to all its neighbor mobility info.
	std::map<LAddress::L3Type, NeighborInfo*>::iterator itN; ///< an iterator used to traverse container neighbors.
	std::map<int /* GUID */, WaveShortMessage*> messageMemory; ///< a map from a message's GUID to the point to this message.
	///@}

	/** @name messages. */
	///@{
	cMessage *sendBeaconEvt;       ///< self message event used to periodically send beacons.
	cMessage *examineNeighborsEvt; ///< self message event used to examine the connectivity with neighbors.
	cMessage *forgetMemoryEvt; ///< self message event used to periodically forget message received too long time ago in memory.
	cMessage *recycleGUIDEvt;  ///< self message event used to periodically recycle GUIDs allocated before.
	///@}

	/** @name TraCI mobility interfaces. */
	///@{
	Veins::TraCIMobility *mobility;
	Veins::TraCICommandInterface *traci;
	Veins::TraCICommandInterface::Vehicle *traciVehicle;
	Veins::AnnotationManager *annotations;
	WaveAppToMac1609_4Interface *myMac;
	///@}

	static const simsignalwrap_t mobilityStateChangedSignal;
	static const simsignalwrap_t parkingStateChangedSignal;
};

#endif /* __BASEWAVEAPPLLAYER_H__ */
