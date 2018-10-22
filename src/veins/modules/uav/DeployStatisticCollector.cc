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

#include <fstream>
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include "veins/modules/routing/RoutingUtils.h"
#include "veins/modules/uav/DeployStatisticCollector.h"

Define_Module(DeployStatisticCollector);

const simsignalwrap_t DeployStatisticCollector::optimalityCalculationSignal = simsignalwrap_t("optimalityCalculation");

void DeployStatisticCollector::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		practicalVec.setName("practical");
		theoreticalVec.setName("theoretical");

		calculateInterval = SimTime(par("calculateInterval").longValue(), SIMTIME_S);

		calculateEvt = new cMessage("calculate evt", DSCMessageKinds::CALCULATE_EVT);
		calculateEvt->setSchedulingPriority(1); // calculate after UAV's decide event
		scheduleAt(SimTime(par("emitFirstUavAt").longValue()+1, SIMTIME_S), calculateEvt);
		EV << "DeployStatisticCollector::initialize() called.\n";
	}
	else
	{
		EV << "DeployStatisticCollector initialized.\n";
	}
}

void DeployStatisticCollector::finish()
{
	EV << "DeployStatisticCollector::finish() called.\n";

	// write statistic to file for figuring in MATLAB
	std::ofstream fout("deployStatistics.csv", std::ios_base::out | std::ios_base::trunc);
	if (!fout.is_open())
		error("cannot open file deployStatistics.csv!");
	std::list<int>::iterator it1 = calculateAt.begin(), it2 = practical.begin(), it3 = theoretical.begin();
	for (; it1 != calculateAt.end(); ++it1, ++it2, ++it3)
		fout << *it1 << ',' << *it2 << ',' << *it3 << "\n";
	fout.close();

	cComponent::finish();
}

void DeployStatisticCollector::importVehicles(std::list<LAddress::L3Type>& vehicleList)
{
	EV << "DeployStatisticCollector::importVehicles() called.\n";
	vehicleSet.splice(vehicleSet.end(), vehicleList);
}

void DeployStatisticCollector::handleMessage(cMessage *msg)
{
	if (!msg->isSelfMessage())
		error("DeployStatisticCollector only accept self messages.");

	switch (msg->getKind())
	{
	case DSCMessageKinds::CALCULATE_EVT:
	{
		vehicleSet.clear();
		cSimulation::getActiveSimulation()->getSystemModule()->emit(optimalityCalculationSignal, this);
		calculate();
		scheduleAt(simTime() + calculateInterval, calculateEvt);
		break;
	}
	default:
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
	}
}

void DeployStatisticCollector::calculate()
{
	EV << "DeployStatisticCollector calculate at " << simTime() << "\n";

	vehicleSet.sort();
	vehicleSet.unique();
	EV << "covered vehicles:";
	for (std::list<LAddress::L3Type>::iterator it = vehicleSet.begin(); it != vehicleSet.end(); ++it)
		EV << ' ' << *it;
	EV << std::endl;
	int practicalNum = static_cast<int>(vehicleSet.size());
	int theoreticalNum = doCalculate();

	calculateAt.push_back(simTime().inUnit(SIMTIME_S));
	practical.push_back(practicalNum);
	theoretical.push_back(theoreticalNum);
	practicalVec.record(practicalNum);
	theoreticalVec.record(theoreticalNum);

	EV << "practical: " << practicalNum << ", theoretical: " << theoreticalNum << std::endl;
}

int DeployStatisticCollector::doCalculate()
{
	std::map<LAddress::L3Type, Coord> &vehPos = MobilityObserver::Instance()->globalPosition;
	std::map<LAddress::L3Type, Coord> &uavPos = MobilityObserver::Instance2()->globalPosition; // including RSUs
	std::map<LAddress::L3Type, Coord>::iterator it = uavPos.lower_bound(UAV_ADDRESS_OFFSET);
	Coord rsuPos = uavPos[RSU_ADDRESS_OFFSET];
	int K = std::distance(it, uavPos.end()); // the number of UAVs
	double r = BaseConnectionManager::maxInterferenceDistance;
	double R = BaseConnectionManager::maxInterferenceDistance2;
	// all required information is attained, now performs algorithm to find global optimal solution

	return rand() % 20;
}

DeployStatisticCollector::~DeployStatisticCollector()
{
	EV << "DeployStatisticCollector::~DeployStatisticCollector() called.\n";
}
