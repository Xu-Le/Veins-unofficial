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
#include "veins/base/utils/FindModule.h"
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include "veins/modules/routing/RoutingUtils.h"
#include "veins/modules/uav/DeployStatisticCollector.h"

Define_Module(DeployStatisticCollector);

const simsignalwrap_t DeployStatisticCollector::optimalityCalculationSignal = simsignalwrap_t("optimalityCalculation");

/**
 * @brief Actual class responsible for global optimal solution calculation.
 *
 * @author Xu Le
 * @ingroup routingLayer
 */
class OptimalCalculator
{
public:
	/** @name constructor, destructor. */
	///@{
	OptimalCalculator(double r, double _R, double d) : r2(r*r), R(_R), d2(d*d) {}
	~OptimalCalculator();
	///@}

	/** Attain positions of all vehicles. */
	void attainVehicles(std::map<LAddress::L3Type, Coord>& _vehicles);
	/** Build the artificial graph whose nodes are intersections of a series of artificial grids. */
	void buildMatrix(double X, double Y, double x0, double y0, const int K, const double g=100.0);
	/** Using DFS to traverse all solutions to find the optimal one. */
	int traverse(int k, int rsu=0);

private:
	/** Find all solutions recursively. */
	void dfs(int u, int k, std::vector<int>& path);
	/** Update container visited when returning from a certain layer in DFS process. */
	void backtrack(int u);

private:
	int n;  ///< the number of nodes in artificial graph including the RSU.
	int vn; ///< the number of vehicles.
	int un; ///< the number of UAVs.
	int maxCovered;  ///< maximum number of vehicles covered.
	const double r2; ///< square of V2XRadius.
	const double R; ///< U2URadius.
	const double d2; ///< square of minimum gap between two UAVs.
	std::vector<int> answer;      ///< record optimal nodes in artificial graph to be deployed.
	std::vector<Coord> vehicles;  ///< positions of all vehicles.
	std::vector<Coord> positions; ///< positions of all nodes in artificial graph.
	std::set<int> tree; ///< current set of nodes selected in artificial graph.
	std::map<int, bool> visited; ///< outer edge nodes of current set of nodes selected in artificial graph.
	std::map<int, bool>::iterator itV;  ///< iterator used to iterate container visited.
	std::vector<std::list<int> > graph; ///< adjacent list of the artificial graph.
	std::vector<std::vector<int> > matrix;  ///< adjacent matrix of the artificial graph.
	std::vector<std::vector<int> > bitmaps; ///< each binary element indicate whether an UAV covers a vehicle.
};

void DeployStatisticCollector::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		practicalVec.setName("practical");
		theoreticalVec.setName("theoretical");

		world = FindModule<BaseWorldUtility*>::findGlobalModule();
		if (world == nullptr)
			error("Could not find BaseWorldUtility module");

		calculateInterval = SimTime(par("calculateInterval").longValue(), SIMTIME_S);

		calculateEvt = new cMessage("calculate evt", DSCMessageKinds::CALCULATE_EVT);
		calculateEvt->setSchedulingPriority(1); // calculate after UAV's decide event
		scheduleAt(SimTime(par("emitFirstUavAt").longValue()+2, SIMTIME_S), calculateEvt);
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
	OptimalCalculator calculator(r, R, r);
	calculator.attainVehicles(vehPos);
	calculator.buildMatrix(world->getPgs()->x, world->getPgs()->y, rsuPos.x, rsuPos.y, K);
	return calculator.traverse(K);
}

DeployStatisticCollector::~DeployStatisticCollector()
{
	EV << "DeployStatisticCollector::~DeployStatisticCollector() called.\n";
}

void OptimalCalculator::attainVehicles(std::map<LAddress::L3Type, Coord>& _vehicles)
{
	vehicles.reserve(_vehicles.size());
	for (std::map<LAddress::L3Type, Coord>::iterator it = _vehicles.begin(); it != _vehicles.end(); ++it)
		vehicles.push_back(it->second);
	vn = static_cast<int>(vehicles.size());
}

void OptimalCalculator::buildMatrix(double X, double Y, double x0, double y0, const int K, const double g)
{
	n = 1;
	un = K;
	const double farthest = K * R, R2 = R * R;
	double x = 0.0, y = 0.0;
	double lowerX = std::max(x0 - farthest, 0.0), lowerY = std::max(y0 - farthest, 0.0);
	double upperX = std::min(x0 + farthest, X), upperY = std::min(y0 + farthest, Y);
	int loopX = 0, loopY = 0;
	for (x = x0; x > lowerX; x -= g)
		++loopX;
	lowerX = x + g;
	for (x = x0 + g; x < upperX; x += g)
		++loopX;
	upperX = x - g;
	for (y = y0; y > lowerY; y -= g)
		++loopY;
	lowerY = y + g;
	for (y = y0 + g; y < upperY; y += g)
		++loopY;
	upperY = y - g;
	positions.reserve(loopX * loopY);
	positions.push_back(Coord(x0, y0));
	double cacheDist2 = 0.0;
	for (double x = lowerX; x < upperX + g/2; x += g)
	{
		for (double y = lowerY; y < upperY + g/2; y += g)
		{
			cacheDist2 = (x - x0) * (x - x0) + (y - y0) * (y - y0);
			if (cacheDist2 > d2 && cacheDist2 < farthest*farthest)
			{
				positions.push_back(Coord(x, y));
				++n;
			}
		}
	}
	matrix.resize(n, std::vector<int>(n, 0));
	bitmaps.resize(n, std::vector<int>(vn, 0));
	for (int i = 0; i < n; ++i)
		for (int j = i+1; j < n; ++j)
			if ((cacheDist2 = positions[i].sqrdist(positions[j])) < R2 && cacheDist2 > d2)
				matrix[i][j] = matrix[j][i] = 1;
	for (int i = 0; i < n; ++i)
		for (int j = 0; j < vn; ++j)
			bitmaps[i][j] = positions[i].sqrdist(vehicles[j]) < r2 ? 1 : 0;
}

int OptimalCalculator::traverse(int k, int rsu)
{
	maxCovered = 0;
	tree.insert(rsu);
	visited.insert(std::pair<int, bool>(rsu, true));
	graph.resize(n, std::list<int>());
	for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
			if (matrix[i][j] == 1)
				graph[i].push_back(j);
	std::vector<int> path(1, rsu);
	dfs(rsu, k, path);
	return maxCovered;
}

void OptimalCalculator::dfs(int u, int k, std::vector<int>& path)
{
	if (k == 0)
	{
		int covered = 0;
		for (int j = 0; j < vn; ++j)
		{
			int i = -1;
			while (++i <= un)
				if (bitmaps[path[i]][j] == 1)
					break;
			if (i <= un)
				++covered;
		}
		if (maxCovered < covered)
		{
			maxCovered = covered;
			answer = path;
		}
		backtrack(u);
		return;
	}
	for (std::list<int>::iterator it = graph[u].begin(); it != graph[u].end(); ++it)
		if (tree.find(*it) == tree.end() && visited.find(*it) == visited.end())
			visited.insert(std::pair<int, bool>(*it, false));
	for (itV = visited.begin(); itV != visited.end(); ++itV)
	{
		if (itV->second)
			continue;
		int nb = itV->first;
		size_t i = 0;
		for (; i < path.size(); ++i)
			if (positions[nb].sqrdist(positions[path[i]]) < d2)
				break;
		if (i < path.size())
			continue;
		tree.insert(nb);
		itV->second = true;
		path.push_back(nb);
		dfs(itV->first, k-1, path);
		path.pop_back();
		itV = visited.find(nb);
	}
	backtrack(u);
}

void OptimalCalculator::backtrack(int u)
{
	tree.erase(u);
	visited[u] = false;
	for (std::list<int>::iterator it = graph[u].begin(); it != graph[u].end(); ++it)
	{
		if (tree.find(*it) == tree.end())
		{
			bool connected = false;
			for (std::set<int>::iterator itT = tree.begin(); itT != tree.end(); ++itT)
			{
				if (matrix[*itT][*it] == 1)
				{
					connected = true;
					break;
				}
			}
			if (connected)
				visited[*it] = false;
			else
				visited.erase(*it);
		}
	}
}

OptimalCalculator::~OptimalCalculator()
{
	answer.clear();
	vehicles.clear();
	positions.clear();
	tree.clear();
	visited.clear();
	graph.clear();
	matrix.clear();
	bitmaps.clear();
}
