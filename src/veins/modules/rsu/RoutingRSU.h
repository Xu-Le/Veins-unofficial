//
// Copyright (C) 2016-2019 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __ROUTINGRSU_H__
#define __ROUTINGRSU_H__

#include "veins/modules/rsu/BaseRSU.h"
#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/messages/UavBeaconMessage_m.h"
#include "veins/modules/messages/UavNotifyMessage_m.h"
#include "veins/modules/routing/MobilityObserver.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"

/**
 * @brief A concrete RSU class which is aware of routing protocols.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRSU
 */
class RoutingRSU : public BaseRSU
{
public:
	/** @brief The message kinds RoutingRSU uses. */
	enum RoutingRSUMessageKinds {
		SEND_UAV_BEACON_EVT = LAST_BASE_RSU_MESSAGE_KIND,
		EXAMINE_UAVS_EVT,
		EMIT_UAV_EVT,
		ATTAIN_DENSITY_EVT,
		LAST_ROUTING_RSU_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	RoutingRSU() : BaseRSU() {}
	RoutingRSU(unsigned stacksize) : BaseRSU(stacksize) {}
	~RoutingRSU() {}
	///@}

	void initialize(int stage) override;
	void finish() override;
	void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from lower layer. */
	void handleLowerMsg(cMessage *msg) override;

	/** @brief UAV beacon message send method. */
	void sendUavBeacon();

	/** @brief call-back method of receiving UAV beacon message. */
	void onUavBeacon(UavBeaconMessage *uavBeaconMsg);
	/** @brief call-back method of receiving UAV notify message. */
	void onUavNotify(UavNotifyMessage *uavNotifyMsg);
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg);
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg);

	/** @brief examine whether UAVs still in connected. */
	void examineUAVs();
	/** @brief emit an UAV. */
	void emitUAV();
	/** @brief attain sector density. */
	void attainDensity();

private:
	/** @brief The class to store UAV's information collected by UAV beacon message. */
	class UAVInfo
	{
	public:
		UAVInfo(Coord& p, Coord& s, simtime_t ra) : pos(p), speed(s), receivedAt(ra) {}

		Coord pos;   ///< current position of the UAV.
		Coord speed; ///< current speed of the UAV.
		simtime_t receivedAt; ///< the time received the most recently beacon message from the UAV.
	};

	int beaconLengthBits;  ///< the length of beacon message measured in bits.
	int beaconPriority;    ///< the priority of beacon message.
	int routingLengthBits; ///< the length of routing message measured in bits.
	int routingPriority;   ///< the priority of routing message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.

	int curUavIndex; ///< the index of current UAV to be emitted.
	int totalUavNum; ///< the total number of UAVs to be emitted.
	int sectorNum;   ///< the number of sectors.

	double radTheta; ///< theta measured in rad.
	double averageDensity;  ///< average density of all sectors.
	double densityDivision; ///< maximum sector density division by average sector density.

	double beaconInterval;        ///< the interval of sending beacon message.
	double examineUavsInterval;   ///< the interval of examining the connectivity with UAVs.
	double emitUavInterval;       ///< the interval of emitting UAVs.
	double attainDensityInterval; ///< the interval of attaining sector density.
	double UavElapsed;            ///< the maximum time haven't receive message from UAVs leading to assume lose connectivity with it.

	cMessage *sendUavBeaconEvt; ///< self message event used to periodically send UAV beacons.
	cMessage *examineUavsEvt;   ///< self message event used to examine the connectivity with UAVs.
	cMessage *emitUavEvt;       ///< self message event used to periodically emit UAVs.
	cMessage *attainDensityEvt; ///< self message event used to periodically attain sector density.

	std::map<LAddress::L3Type, UAVInfo*> UAVs; ///< a map from a UAV's identifier to all its mobility info.
	std::map<LAddress::L3Type, UAVInfo*>::iterator itU; ///< an iterator used to traverse container UAVs.
	AccessTable accessTable;    ///< store the next hop on accessing path to the RSU.
	AccessTable::iterator itAT; ///< an iterator used to traverse container accessTable.

	static int uavIndexCounter; ///< self increasing counter.
	static const simsignalwrap_t optimalityCalculationSignal;
};

#endif /* __ROUTINGRSU_H__ */
