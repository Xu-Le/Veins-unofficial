//
// Copyright (C) 2016-2018 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __WARNINGRSU_H__
#define __WARNINGRSU_H__

#include "veins/modules/rsu/BaseRSU.h"
#include "veins/modules/messages/WarningMessage_m.h"
#include "veins/modules/routing/MobilityObserver.h"
#include "veins/modules/routing/WarningStatisticCollector.h"

/**
 * @brief A concrete RSU class which is aware of warning protocols.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRSU
 */
class WarningRSU : public BaseRSU
{
public:
	/** @brief The message kinds WarningRSU uses. */
	enum WarningRSUMessageKinds {
		WARNING_NOTIFY_EVT = LAST_BASE_RSU_MESSAGE_KIND,
		RECYCLE_GUID_EVT,
		LAST_WARNING_RSU_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	WarningRSU() : BaseRSU() {}
	WarningRSU(unsigned stacksize) : BaseRSU(stacksize) {}
	~WarningRSU() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;

	/** @brief call a warning notify to a certain direction determined by warningPlanList. */
	void callWarning(double distance);
	/** @brief warning message decorate method. */
	void decorateWarning(WarningMessage *warningMsg);
	/** @brief initialize warningPlanList through configured xmlfile. */
	void initializeWarningPlanList(cXMLElement *xmlConfig);
	/** @brief return the number of target vehicles in ROI of the warning message. */
	long targetVehicles(bool plusX, Coord xMin, Coord xMax, LAddress::L3Type& farthestOne);

private:
	int warningLengthBits; ///< the length of warning message measured in bits.
	int warningPriority;   ///< the priority of warning message.

	Coord fromRoadhead;    ///< the tail of road where this RSU locate.
	Coord toRoadhead;      ///< the head of road where this RSU locate.
	Coord verticalPoint;   ///< the vertical point from RSU to the road center line.

	simtime_t guidUsedTime;   ///< the maximum time from a GUID's allocated time to its recycled time.
	std::string laneId;       ///< the lane this warning message aims to.

	cMessage *callWarningEvt; ///< self message event used to call warning notify to certain direction determined by warningPlanList.
	cMessage *recycleGUIDEvt; ///< self message event used to periodically recycle GUIDs allocated before.

	std::list<int> guidUsed;  ///< record GUID used before for recycle purpose.
	std::list<std::pair<double /* simtime */, double /* distance */> > warningPlanList; ///< warning plans of all RSUs configured by a xmlfile.
};

#endif /* __WARNINGRSU_H__ */
