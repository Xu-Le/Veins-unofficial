//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __BASERSU_H__
#define __BASERSU_H__

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include "veins/modules/messages/WiredMessage_m.h"
#include "veins/modules/messages/BeaconMessage_m.h"
#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/WarningMessage_m.h"
#include "veins/modules/messages/ContentMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/routing/RoutingUtils.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"
#include "veins/modules/routing/WarningStatisticCollector.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"
#include "veins/modules/world/annotations/AnnotationManager.h"

#define RSU_ADDRESS_OFFSET    10000

/**
 * @brief Base class for RSU design.
 *
 * @author Xu Le
 *
 * @ingroup routingLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 */
class BaseRSU : public BaseApplLayer
{
public:
	/** @brief The message kinds routing layer uses. */
	enum RSUMessageKinds {
		SERVICE_PROVIDER = LAST_BASE_APPL_MESSAGE_KIND,
		EXAMINE_VEHICLES_EVT,
		FORGET_MEMORY_EVT,
		WARNING_NOTIFY_EVT,
		RECYCLE_GUID_EVT,
		LAST_BASE_RSU_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	BaseRSU() : BaseApplLayer() {}
	BaseRSU(unsigned stacksize) : BaseApplLayer(stacksize) {}
	virtual ~BaseRSU() {}
	///@}

	virtual void initialize(int stage) override;
	virtual void finish() override;

protected:
	/** @brief Called every time a message arrives(template method, subclass should not override it). */
	virtual void handleMessage(cMessage *msg) override;

	/** @brief handle self messages. */
	virtual void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from below(template method, subclass should not override it). */
	virtual void handleLowerMsg(cMessage *msg) override;
	/** @brief Handle wired incoming messages. */
	virtual void handleWiredMsg(WiredMessage *wiredMsg) {}
	/** @brief Handle west RSU incoming messages. */
	void handleWestMsg(WiredMessage *wiredMsg) { handleRSUMsg(wiredMsg, 1); }
	/** @brief Handle east RSU messages. */
	void handleEastMsg(WiredMessage *wiredMsg) { handleRSUMsg(wiredMsg, 2); }
	/** @brief Handle north RSU incoming messages(unused yet). */
	void handleNorthMsg(WiredMessage *wiredMsg) { handleRSUMsg(wiredMsg, 3); }
	/** @brief Handle south RSU messages(unused yet). */
	void handleSouthMsg(WiredMessage *wiredMsg) { handleRSUMsg(wiredMsg, 4); }
	/** @brief Handle west/east RSU messages, param 'direction': west is 1, east is 2, north is 3, south is 4. */
	virtual void handleRSUMsg(WiredMessage *wiredMsg, int direction) {}

	/** @brief wave short message factory method(template method, subclass should not override it). */
	virtual WaveShortMessage* prepareWSM(std::string name, int dataLength, t_channel channel, int priority, int serial);
	/** @brief wave short message decorate method. */
	virtual void decorateWSM(WaveShortMessage *wsm);
	/** @brief wave short message send method. */
	virtual void sendWSM(WaveShortMessage *wsm);
	/** @brief call-back method of receiving beacon message. */
	virtual void onBeacon(BeaconMessage *beaconMsg);
	/** @brief call-back method of receiving routing message. */
	virtual void onRouting(RoutingMessage *routingMsg) = 0;
	/** @brief call-back method of receiving content message. */
	virtual void onContent(ContentMessage *contentMsg) = 0;
	/** @brief call-back method of receiving data message. */
	virtual void onData(DataMessage *dataMsg) = 0;

	/** @brief examine whether vehicles still in connected. */
	virtual void examineVehicles();
	/** @brief forget routing packets received long time ago. */
	void forgetMemory();

protected:
	/** @brief which side this RSU locate along the road. */
	enum SideDirection {
		EAST_SIDE,
		WEST_SIDE,
		SOUTH_SIDE,
		NORTH_SIDE
	};

	/** @brief The class to store neighbor's information collected by beacon message. */
	class VehicleInfo
	{
	public:
		VehicleInfo(Coord& p, Coord& s, simtime_t ra) : pos(p), speed(s), receivedAt(ra) {}

		Coord pos;   ///< current position of the vehicle.
		Coord speed; ///< current speed of the vehicle.
		simtime_t receivedAt; ///< the time received the most recently beacon message from the vehicle.
	};

	/** @name gate IDs. */
	///@{
	int wiredIn;    ///< receive packets from file content server.
	int wiredOut;   ///< send packets to file content server.
	int westIn;     ///< receive packets from west neighbor RSU.
	int westOut;    ///< send packets to west neighbor RSU.
	int eastIn;     ///< receive packets from east neighbor RSU.
	int eastOut;    ///< send packets to east neighbor RSU.
	///@}
	int wiredHeaderLength; ///< length of the IP packet header.

	bool helpRoutings; ///< whether this RSU will help vehicle rebroadcast routing messages.
	bool sendWarnings; ///< whether this RSU is at work.
	bool sendContents; ///< whether send data to other vehicles.
	bool dataOnSch;    ///< whether send data on service channel.

	int warningLengthBits; ///< the length of warning message measured in bits.
	int warningPriority;   ///< the priority of warning message.
	int contentLengthBits; ///< the length of content message measured in bits.
	int contentPriority;   ///< the priority of content message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.
	int maxHopConstraint;  ///< the maximum of routing message hop count constraint.
	int whichSide;         ///< which side direction relative to road this RSU locate.

	double examineVehiclesInterval; ///< the interval of examining the connectivity with vehicles.
	double forgetMemoryInterval;    ///< the interval of forgetting message received too long time ago.
	double vehicleElapsed;   ///< the maximum time haven't receive message from vehicles leading to assume lose connectivity with it.
	double memoryElapsed;    ///< the maximum time can a message store in memory.
	double distFromRoadhead; ///< the distance from vertical point to fromRoadhead.

	Coord curPosition;      ///< current position of this RSU.

	cMessage *examineVehiclesEvt; ///< self message event used to examine the connectivity with vehicles.
	cMessage *forgetMemoryEvt;    ///< self message event used to periodically forget message received too long time ago in memory.

	std::map<LAddress::L3Type, VehicleInfo*> vehicles; ///< a map from a vehicle's identifier to all its mobility info.
	std::map<int /* GUID */, simtime_t> messageMemory; ///< a set stores a message's GUID and its received time which is received recently.
	std::map<LAddress::L3Type, VehicleInfo*>::iterator itV; ///< an iterator used to traverse container vehicles.

	WaveAppToMac1609_4Interface *myMac;
	Veins::AnnotationManager *annotations;
};

#endif /* __BASERSU_H__ */
