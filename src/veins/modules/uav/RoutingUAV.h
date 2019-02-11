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

#ifndef __ROUTINGUAV_H__
#define __ROUTINGUAV_H__

#include "veins/modules/uav/BaseUAV.h"
#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"

/**
 * @brief A concrete UAV class which is aware of routing protocols.
 *
 * @author Xu Le
 * @ingroup waveAppLayer
 * @see BaseUAV
 */
class RoutingUAV : public BaseUAV
{
public:
	/** @brief The message kinds RoutingUAV uses. */
	enum RoutingUAVMessageKinds {
		FLYING_EVT = LAST_BASE_UAV_MESSAGE_KIND,
		DECIDE_EVT,
		LAST_ROUTING_UAV_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	RoutingUAV() : BaseUAV() {}
	RoutingUAV(unsigned stacksize) : BaseUAV(stacksize) {}
	~RoutingUAV() {}
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

	/** @brief UAV beacon message decorate method. */
	void decorateUavBeacon(UavBeaconMessage *uavBeaconMsg) override;

	/** @brief call-back method of receiving UAV beacon message. */
	void onUavBeacon(UavBeaconMessage *uavBeaconMsg) override;
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg);
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg);
	/** @brief call-back method of receiving data control message. */
	void onDataLost(DataMessage *lostDataMsg);

private:
	/** @brief The derived class to store neighbor's information collected by beacon message. */
	class RoutingNeighborInfo : public BaseUAV::NeighborInfo
	{
	public:
		RoutingNeighborInfo(Coord& p, Coord& s, simtime_t ra, int r) : NeighborInfo(p, s, ra), reserved(r) {}

		int reserved; ///< reserved field.
	};

	int routingLengthBits; ///< the length of routing message measured in bits.
	int routingPriority;   ///< the priority of routing message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.

	cMessage *flyingEvt; ///< self message event used to periodically flying a little distance.
	cMessage *decideEvt; ///< self message event used to periodically make a flying decision.
};

#endif /* __ROUTINGUAV_H__ */
