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
#include "veins/modules/messages/UavNotifyMessage_m.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"

/**
 * @brief A concrete UAV class which is aware of routing protocols.
 *
 * @author Xu Le
 * @ingroup routingLayer
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
	/** @brief The derived class to store neighbor's information collected by beacon message. */
	class RoutingNeighborInfo : public BaseUAV::NeighborInfo
	{
	public:
		RoutingNeighborInfo(Coord& p, Coord& s, simtime_t ra, int h, simtime_t da, double hd, double ad, double dd)
			: NeighborInfo(p, s, ra), hop(h), decideAt(da), hopDist(hd), averageDensity(ad), densityDivision(dd) {}

		int hop;            ///< minimum hop count to the RSU.
		simtime_t decideAt; ///< time instant when making the last deployment decision.
		double hopDist;     ///< distance to the next hop.
		double averageDensity;  ///< average density of all sectors.
		double densityDivision; ///< maximum sector density division by average sector density.
	};

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
	/** @brief call-back method of receiving UAV notify message. */
	void onUavNotify(UavNotifyMessage *uavNotifyMsg);
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg);
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg);
	/** @brief call-back method of receiving data control message. */
	void onDataLost(DataMessage *lostDataMsg);
	/** @brief call-back method when neighbors updates. */
	void onNeighborUpdate();

	/** @brief examine whether neighbors still in connected. */
	void examineNeighbors() override;
	/** @brief actually updating position while flying towards a specified direction. */
	void flying();
	/** @brief decide how to move in the following several seconds. */
	void decide();
#ifndef USE_VIRTUAL_FORCE_MODEL
	/** @brief attain the average vehicle density of each sector. */
	void attainSectorDensity();
	/** @brief attain the number of forbidden sectors. */
	int attainForbiddenSector();
#else
	/** @brief attain resultant force which specifies flying direction. */
	void attainResultantForce();
#endif
	/** @brief check whether a flying destination is forbidden. */
	bool isForbidden(Coord dst, std::list<RoutingNeighborInfo*>& nbLess,
								std::list<RoutingNeighborInfo*>& nbEqual,
								std::list<RoutingNeighborInfo*>& nbGreater);
	/** @brief check whether will leave this neighbor. */
	bool willLeave(Coord &self, RoutingNeighborInfo *nbInfo);

private:
	LAddress::L3Type rsuAddr; ///< address of the RSU.

	int routingLengthBits; ///< the length of routing message measured in bits.
	int routingPriority;   ///< the priority of routing message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.
	int hop;         ///< minimum hop count to the RSU.
	int remainingMs; ///< milliseconds left for the next deployment decision.

	double flyingSpeed; ///< constant flying speed.
	double hopDist;  ///< distance to the next hop.
#ifndef USE_VIRTUAL_FORCE_MODEL
	double averageDensity;  ///< average density of all sectors.
	double densityDivision; ///< maximum sector density division by average sector density.
#endif

	Coord curDirection; ///< current direction flying towards.

	SimTime flyingInterval; ///< the interval of actually updating position while flying.
	SimTime decideInterval; ///< the interval of making a decision.
	SimTime nextDecisionAt; ///< the time instant to make the next decision.

	cMessage *flyingEvt; ///< self message event used to periodically flying a little distance.
	cMessage *decideEvt; ///< self message event used to periodically make a deployment decision.

#ifndef USE_VIRTUAL_FORCE_MODEL
	std::vector<double> sectorDensity; ///< average vehicle density of each sector.
	std::vector<int> forbidden;        ///< indicate whether a sector is forbidden.
#endif
	AccessTable accessTable;    ///< store the next hop on accessing path to the RSU.
	AccessTable::iterator itAT; ///< an iterator used to traverse container accessTable.

	static int stopFlyingMs; ///< when remainingMs reach this value, stop flying.
#ifndef USE_VIRTUAL_FORCE_MODEL
	static int expand;       ///< expand to how many adjacent sectors to smooth the density in average.
	static int sectorNum;    ///< the number of sectors.
	static double radTheta;  ///< theta measured in rad.
	static double minGap;    ///< minimum distance between two UAVs.
#else
	static double k_a;       ///< coefficient of attractive force of ground users.
	static double K_r;       ///< coefficient of repulsive force of other UAVs.
	static double R_opt;     ///< distance threshold within which bring about repulsive force.
#endif
};

#endif /* __ROUTINGUAV_H__ */
