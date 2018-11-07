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

#include "deploy.h"

extern double gX;
extern double gY;
extern int numVehicle;
extern Point *vehicleTable;
extern double *velocityTable;
extern double *directionTable;
extern int *coveredTable;

Solution::Solution() : eventSequence(0), rsu(this)
{
	srand(1);
}

int Solution::initialize(int _num)
{
	uavNum = _num;
	uavs.reserve(uavNum);
	accessPaths.resize(uavNum, std::list<int>());
	rsu.setX(gX/2);
	rsu.setY(gY/2);

	rsu.setSelfIndex(uavNum);
	rsu.setRemainingNum(uavNum);
	schedule(10, uavNum, RSU::EventType::CHECKCONN);
	schedule(10, uavNum, RSU::EventType::ATTAINDENSITY);
	schedule(15, uavNum, RSU::EventType::EMITUAV);
	return 10; // first event happens at
}

int Solution::handleEvent(int who, int what, void *data)
{
	if (who < uavNum) // UAV
	{
		uavs[who].handleEvent(what, data);
		return what == UAV::EventType::FLYING ? 1 : 0;
	}
	else // RSU
	{
		rsu.handleEvent(what, data);
		return what == RSU::EventType::EMITUAV ? 2 : 0;
	}
}

void Solution::schedule(int period, int who, int what, int priority, void *data)
{
	Q_ASSERT(period >= 0);
	int when = eventSequence + period;
	EventInfo eventInfo((who << 16) | (what & 0xffff), priority, data);
	eventQ.push(std::pair<int, EventInfo>(when, eventInfo));
}

void Solution::attainVehiclesNearby(const Point& coord, std::list<int>& vehiclesNearby)
{
	vehiclesNearby.clear();
	double r2 = UAV::r * UAV::r;
	for (int i = 0; i < numVehicle; ++i)
		if (math::dist2(vehicleTable[i], coord) < r2)
			vehiclesNearby.push_back(i);
}

void Solution::attainNeighbors(const Point& coord, int selfIndex, std::list<UAV*>& neighbors)
{
	neighbors.clear();
	int curUavNum = static_cast<int>(uavs.size());
	double R2 = UAV::R * UAV::R;
	for (int i = 0; i < curUavNum; ++i)
		if (i != selfIndex && math::dist2(coord, uavs[i].getPos()) < R2)
			neighbors.push_back(&uavs[i]);
}
