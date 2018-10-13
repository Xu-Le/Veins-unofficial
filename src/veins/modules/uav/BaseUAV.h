//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __BASEUAV_H__
#define __BASEUAV_H__

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include "veins/modules/messages/BeaconMessage_m.h"
#include "veins/modules/messages/UavBeaconMessage_m.h"
#include "veins/modules/routing/RoutingUtils.h"
#include "veins/modules/routing/MobilityObserver.h"
#include "veins/modules/mobility/AircraftMobility.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"
#include "veins/modules/world/annotations/AnnotationManager.h"

/**
 * @brief Base class for UAV design.
 *
 * @author Xu Le
 *
 * @ingroup routingLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 */
class BaseUAV : public BaseApplLayer
{
public:
	/** @brief The message kinds BaseUAV uses. */
	enum UAVMessageKinds {
		SEND_UAV_BEACON_EVT = LAST_BASE_APPL_MESSAGE_KIND,
		EXAMINE_VEHICLES_EVT,
		EXAMINE_NEIGHBORS_EVT,
		LAST_BASE_UAV_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	BaseUAV() : BaseApplLayer() {}
	BaseUAV(unsigned stacksize) : BaseApplLayer(stacksize) {}
	virtual ~BaseUAV() {}
	///@}

	virtual void initialize(int stage) override;
	virtual void finish() override;
	virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;

protected:
	/** @brief Called every time a message arrives(template method, subclass should not override it). */
	virtual void handleMessage(cMessage *msg) override;

	/** @brief handle self messages. */
	virtual void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from lower layer. */
	virtual void handleLowerMsg(cMessage *msg) override;

	/** @brief update UAV's mobility information attained from mobility module. */
	void handleMobilityUpdate(cObject *obj);

	/** @brief wave short message factory method. */
	void prepareWSM(WaveShortMessage *wsm, int dataLength, t_channel channel, int priority, int serial);
	/** @brief wave short message send method. */
	void sendWSM(WaveShortMessage *wsm);
	/** @brief UAV beacon message send method. */
	void sendUavBeacon();
	/** @brief UAV beacon message decorate method. */
	virtual void decorateUavBeacon(UavBeaconMessage *uavBeaconMsg) = 0;
	/** @brief call-back method of receiving beacon message. */
	virtual void onBeacon(BeaconMessage *beaconMsg);
	/** @brief call-back method of receiving UAV beacon message. */
	virtual void onUavBeacon(UavBeaconMessage *uavBeaconMsg);

	/** @brief examine whether vehicles still in connected. */
	virtual void examineVehicles();
	/** @brief examine whether neighbors still in connected. */
	virtual void examineNeighbors();

protected:
	/** @brief The class to store vehicle's information collected by beacon message. */
	class VehicleInfo
	{
	public:
		VehicleInfo(Coord& p, Coord& s, simtime_t ra) : pos(p), speed(s), receivedAt(ra) {}

		Coord pos;   ///< current position of the vehicle.
		Coord speed; ///< current speed of the vehicle.
		simtime_t receivedAt; ///< the time received the most recently beacon message from the vehicle.
	};

	/** @brief The base class to store neighbor's information collected by beacon message. */
	class NeighborInfo
	{
	public:
		NeighborInfo(Coord& p, Coord& s, simtime_t ra) : pos(p), speed(s), receivedAt(ra) {}
		virtual ~NeighborInfo() {}

		Coord pos;   ///< current position of the neighbor.
		Coord speed; ///< current speed of the neighbor.
		simtime_t receivedAt; ///< the time received the most recently beacon message from the neighbor.
	};

	bool dataOnSch;   ///< whether send data on service channel.

	int beaconLengthBits;  ///< the length of beacon message measured in bits.
	int beaconPriority;    ///< the priority of beacon message.

	double V2XRadius; ///< the biggest transmission distance of transmitter.
	double U2URadius; ///< the biggest transmission distance of transmitter.
	double beaconInterval;           ///< the interval of sending beacon message.
	double examineVehiclesInterval;  ///< the interval of examining the connectivity with vehicles.
	double examineNeighborsInterval; ///< the interval of examining the connectivity with neighbors.
	double vehicleElapsed;           ///< the maximum time haven't receive message from vehicles leading to assume lose connectivity with it.
	double neighborElapsed;          ///< the maximum time haven't receive message from neighbors leading to assume lose connectivity with it.

	Coord curPosition;     ///< current position of this UAV.
	Coord curSpeed;        ///< current speed of this UAV.

	/** @name messages. */
	///@{
	cMessage *sendUavBeaconEvt;    ///< self message event used to periodically send UAV beacons.
	cMessage *examineVehiclesEvt;  ///< self message event used to examine the connectivity with vehicles.
	cMessage *examineNeighborsEvt; ///< self message event used to examine the connectivity with neighbors.
	///@}

	/** @name containers. */
	///@{
	std::map<LAddress::L3Type, VehicleInfo*> vehicles;       ///< a map from a vehicle's identifier to all its info.
	std::map<LAddress::L3Type, VehicleInfo*>::iterator itV;  ///< an iterator used to traverse container vehicles.
	std::map<LAddress::L3Type, NeighborInfo*> neighbors;     ///< a map from a neighbor's identifier to all its info.
	std::map<LAddress::L3Type, NeighborInfo*>::iterator itN; ///< an iterator used to traverse container neighbors.
	///@}

	/** @name TraCI mobility interfaces. */
	///@{
	AircraftMobility *mobility;
	Veins::AnnotationManager *annotations;
	WaveAppToMac1609_4Interface *myMac;
	///@}

	static const simsignalwrap_t mobilityStateChangedSignal;
};

#endif /* __BASEUAV_H__ */
