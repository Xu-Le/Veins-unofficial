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

#ifndef __DEPLOYSTATISTICCOLLECTOR_H__
#define __DEPLOYSTATISTICCOLLECTOR_H__

#include "veins/base/modules/BaseWorldUtility.h"
#include "veins/modules/routing/MobilityObserver.h"

/**
 * @brief Collect global deployment statistics of UAVs periodically for performance assessment.
 *
 * @author Xu Le
 * @ingroup routingLayer
 */
class DeployStatisticCollector : public ::omnetpp::cSimpleModule
{
public:
	enum DSCMessageKinds {
		CALCULATE_EVT
	};

	/** @name constructor, destructor. */
	///@{
	DeployStatisticCollector() : cSimpleModule() {}
	DeployStatisticCollector(unsigned stacksize) : cSimpleModule(stacksize) {}
	virtual ~DeployStatisticCollector();
	///@}

	void initialize(int stage) override;
	void finish() override;

	/** @brief Called by RSU and UAVs when they receive optimalityCalculationSignal. */
	void importVehicles(std::list<LAddress::L3Type>& vehicleList);

private:
	/** @brief Called every time a message arrives. */
	void handleMessage(omnetpp::cMessage *msg);

	/** @brief Calculate the optimality of UAV deployment and record the results. */
	void calculate();
	/** @brief Calculate global optimal solution theoretically. */
	int doCalculate();

private:
	SimTime calculateInterval; ///< the interval between 2 optimality calculation event.

	cMessage *calculateEvt; ///< self message event used to periodically execute optimality calculation.

	BaseWorldUtility *world; ///< pointing to module BaseWorldUtility.

	cOutVector practicalVec;   ///< same as practical.
	cOutVector theoreticalVec; ///< same as theoretical.

	std::list<int> calculateAt; ///< a series of time instants at which optimality calculation executes.
	std::list<int> practical;   ///< attained solution by our strategy in practical.
	std::list<int> theoretical; ///< global optimal solution theoretically.
	std::list<LAddress::L3Type> vehicleSet; ///< vehicles that covered by RSU or UAVs in practical.

	static const simsignalwrap_t optimalityCalculationSignal;
};

#endif /* __DEPLOYSTATISTICCOLLECTOR_H__ */
