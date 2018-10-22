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

#include "veins/modules/uav/RoutingUAV.h"

Define_Module(RoutingUAV);

extern std::map<LAddress::L3Type, LAddress::L3Type> rsuBelongingMap;

int RoutingUAV::sectorNum = 0;
int RoutingUAV::stopFlyingMs = 0;
double RoutingUAV::minGap = 0.0;

void RoutingUAV::initialize(int stage)
{
	BaseUAV::initialize(stage);

	if (stage == 0)
	{
		rsuAddr = rsuBelongingMap[myAddr];

		hop = 1;
		hopDist = 0.0;
		double theta = par("theta").doubleValue();
		radTheta = theta / 180.0 * M_PI;
		sectorNum = 360 / theta;
		stopFlyingMs = par("stopFlyingMs").longValue();
		minGap = par("minGap").doubleValue();

		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();

		curDirection = Coord::ZERO;
		sectorDensity.resize(sectorNum, 0.0);
		forbidden.resize(sectorNum, 0);

		flyingInterval = SimTime(par("flyingInterval").longValue(), SIMTIME_MS);
		decideInterval = SimTime(par("decideInterval").longValue(), SIMTIME_MS);

		flyingEvt = new cMessage("flying evt", RoutingUAVMessageKinds::FLYING_EVT);
		decideEvt = new cMessage("decide evt", RoutingUAVMessageKinds::DECIDE_EVT);
		scheduleAt(simTime() + beaconInterval, decideEvt);
	}
	else if (stage == 1)
		flyingSpeed = mobility->par("flyingSpeed").doubleValue(); // reuse the parameter in mobility module
}

void RoutingUAV::finish()
{
	// clear containers
	sectorDensity.clear();
	forbidden.clear();
	accessTable.clear();

	// delete handle self message
	cancelAndDelete(flyingEvt);
	cancelAndDelete(decideEvt);

	BaseUAV::finish();
}

void RoutingUAV::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case RoutingUAVMessageKinds::FLYING_EVT:
	{
		flying();
		if (remainingMs > stopFlyingMs)
			scheduleAt(simTime() + flyingInterval, flyingEvt);
		break;
	}
	case RoutingUAVMessageKinds::DECIDE_EVT:
	{
		decide();
		nextDecisionAt = simTime() + decideInterval;
		scheduleAt(nextDecisionAt, decideEvt);
		break;
	}
	default:
		BaseUAV::handleSelfMsg(msg);
	}
}

void RoutingUAV::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "uavNotify") == 0)
		DYNAMIC_CAST_CMESSAGE(UavNotify, uavNotify)
	else if (strcmp(msg->getName(), "routing") == 0)
		DYNAMIC_CAST_CMESSAGE(Routing, routing)
	else if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseUAV::handleLowerMsg(msg);
}

void RoutingUAV::decorateUavBeacon(UavBeaconMessage *uavBeaconMsg)
{
	SimTime remainingTime(nextDecisionAt - simTime());
	uavBeaconMsg->setHop(hop);
	uavBeaconMsg->setRemainingMs(remainingTime.inUnit(SIMTIME_MS)); // do not use remainingMs here
	uavBeaconMsg->setHopDist(hopDist);
}

void RoutingUAV::onUavBeacon(UavBeaconMessage *uavBeaconMsg)
{
	EV << logName() << ": onUavBeacon!\n";

	LAddress::L3Type sender = uavBeaconMsg->getSenderAddress(); // alias
	RoutingNeighborInfo *neighborInfo = nullptr;
	bool nbHopUpdates = true;
	if ((itN = neighbors.find(sender)) != neighbors.end()) // update old record
	{
		EV << "    sender [" << sender << "] is an old neighbor, update its info.\n";
		NeighborInfo *baseNbInfo = itN->second;
		neighborInfo = dynamic_cast<RoutingNeighborInfo*>(baseNbInfo);
		nbHopUpdates = neighborInfo->hop != uavBeaconMsg->getHop();
		neighborInfo->pos = uavBeaconMsg->getSenderPos();
		neighborInfo->speed = uavBeaconMsg->getSenderSpeed();
		neighborInfo->receivedAt = simTime();
		neighborInfo->hop = uavBeaconMsg->getHop();
		neighborInfo->remainingMs = uavBeaconMsg->getRemainingMs();
		neighborInfo->hopDist = uavBeaconMsg->getHopDist();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new neighbor, insert its info.\n";
		neighborInfo = new RoutingNeighborInfo(uavBeaconMsg->getSenderPos(), uavBeaconMsg->getSenderSpeed(), simTime(),
				uavBeaconMsg->getHop(), uavBeaconMsg->getRemainingMs(), uavBeaconMsg->getHopDist());
		neighbors.insert(std::pair<LAddress::L3Type, NeighborInfo*>(sender, neighborInfo));
	}
	if (nbHopUpdates)
		onNeighborUpdate();
}

void RoutingUAV::onUavNotify(UavNotifyMessage *uavNotifyMsg)
{
	EV << logName() << ": onUavNotify!\n";

	if (uavNotifyMsg->getReceiver() != myAddr)
		return;

	AccessHopList &path = uavNotifyMsg->getHopList();
	if (uavNotifyMsg->getAcknowledgment())
	{
		path.pop_back();
		if (path.empty())
		{
			EV << "originator receives acknowledgment.\n";
			return;
		}
		else
			uavNotifyMsg->setReceiver(path.back());
	}
	else
	{
		for (AccessHopList::iterator it = path.begin(); it != path.end(); ++it)
			accessTable[*it] = path.back(); // path.back() is previous hop
		path.push_back(myAddr);
		uavNotifyMsg->setReceiver(accessTable[rsuAddr]);
		EV << logName() << " notify at " << simTime() << ", routing table:\n";
		for (itAT = accessTable.begin(); itAT != accessTable.end(); ++itAT)
			EV << "  " << itAT->first << " -> " << itAT->second << std::endl;
	}
	// what to send is a duplication of uavNotifyMsg, because it will be deleted in BaseUAV::handleMessage()
	UavNotifyMessage *dupUavNotifyMsg = new UavNotifyMessage(*uavNotifyMsg);
	sendWSM(dupUavNotifyMsg);
}

void RoutingUAV::onRouting(RoutingMessage *routingMsg)
{
	EV << logName() << ": onRouting!\n";

	// int guid = routingMsg->getGUID(); // alias
}

void RoutingUAV::onData(DataMessage* dataMsg)
{
	EV << logName() << ": onData!\n";
}

void RoutingUAV::onNeighborUpdate()
{
	int minNbHop = 1000, nextHop = -1;
	double minNbDist = 100000.0;
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
	{
		RoutingNeighborInfo *nbInfo = dynamic_cast<RoutingNeighborInfo*>(itN->second);
		accessTable[itN->first] = itN->first;
		double nbDist = curPosition.distance(nbInfo->pos);
		if (minNbHop > nbInfo->hop || (minNbHop == nbInfo->hop && minNbDist > nbDist))
		{
			minNbHop = nbInfo->hop;
			minNbDist = nbDist;
			nextHop = itN->first;
		}
	}
	hopDist = minNbDist;
	accessTable[rsuAddr] = nextHop;
	if (hop != minNbHop + 1)
	{
		UavNotifyMessage *uavNotifyMsg = new UavNotifyMessage("uavNotify");
		prepareWSM(uavNotifyMsg, routingLengthBits, type_CCH, routingPriority, -1);
		uavNotifyMsg->setReceiver(accessTable[rsuAddr]);
		uavNotifyMsg->getHopList().push_back(myAddr);
		sendWSM(uavNotifyMsg);

		EV << logName() << " notify at " << simTime() << ", routing table:\n";
		for (itAT = accessTable.begin(); itAT != accessTable.end(); ++itAT)
			EV << "  " << itAT->first << " -> " << itAT->second << std::endl;
	}
	hop = minNbHop + 1;
}

void RoutingUAV::examineNeighbors()
{
	size_t oldNum = neighbors.size();
	BaseUAV::examineNeighbors();

	if (neighbors.size() != oldNum)
		onNeighborUpdate();
}

void RoutingUAV::flying()
{
	mobility->setMove(curDirection, flyingSpeed);
	remainingMs -= flyingInterval.inUnit(SIMTIME_MS);
}

void RoutingUAV::decide()
{
	EV << logName() << " decide at " << simTime() << std::endl;

	remainingMs = decideInterval.inUnit(SIMTIME_MS);

	attainSectorDensity();
	double averageDensity = sectorDensity[0], maxSectorDensity = sectorDensity[0];
	for (int i = 1; i < sectorNum; ++i)
	{
		averageDensity += sectorDensity[i];
		if (maxSectorDensity < sectorDensity[i])
			maxSectorDensity = sectorDensity[i];
	}
	averageDensity /= sectorNum;
	EV << ", average density: " << averageDensity << "\n";
	if (maxSectorDensity < 2.0*averageDensity && averageDensity > 1.0)
	{
		EV << "sector density division " << maxSectorDensity/averageDensity << " below threshold 2.0" << std::endl;
		return;
	}

	int forbiddenNum = attainForbiddenSector();
	if (forbiddenNum == sectorNum)
	{
		EV << "all sector are forbidden." << std::endl;
		return;
	}

	// select an optimal moving direction
	maxSectorDensity = 0.0;
	int maxI = 0;
	for (int i = 0; i < sectorNum; ++i)
	{
		if (forbidden[i] == 0 && maxSectorDensity < sectorDensity[i])
		{
			maxSectorDensity = sectorDensity[i];
			maxI = i;
		}
	}
	double phi = (maxI + 0.5) * radTheta;
	curDirection.x = cos(phi);
	curDirection.y = sin(phi);
	EV << "pos: " << curPosition.info() << ", phi: " << 180*phi/M_PI << std::endl;

	mobility->setMove(curDirection, flyingSpeed, false, curPosition);
	scheduleAt(simTime() + flyingInterval, flyingEvt);
}

void RoutingUAV::attainSectorDensity()
{
	const double closeMulitplier = 1.0;
	std::fill(sectorDensity.begin(), sectorDensity.end(), 0.0);
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		VehicleInfo *veh = itV->second;
		double angle = acos((veh->pos.x - curPosition.x) / curPosition.distance(veh->pos));
		if (veh->pos.y < curPosition.y)
			angle = 2*M_PI - angle;
		int sector = angle / radTheta;
		double coverCoefficient = 1.0, closeCoefficient = 0.0;
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
			if ((closeCoefficient = veh->pos.distance(itN->second->pos)/V2XRadius) < 1.0)
				coverCoefficient += 1.0 + closeMulitplier*(1.0 - closeCoefficient);
		sectorDensity[sector] += 1.0/coverCoefficient;
	}
	EV << "sector density:";
	for (size_t k = 0; k < sectorDensity.size(); ++k)
		EV << ' ' << sectorDensity[k];
}

int RoutingUAV::attainForbiddenSector()
{
	int i = 0;
	const double minGap2 = minGap * minGap;
	std::list<RoutingNeighborInfo*> nbLess, nbEqual, nbGreater;
	std::list<RoutingNeighborInfo*>::iterator it;
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
	{
		RoutingNeighborInfo *nbInfo = dynamic_cast<RoutingNeighborInfo*>(itN->second);
		if (nbInfo->hop < hop)
			nbLess.push_back(nbInfo);
		else if (nbInfo->hop == hop)
			nbEqual.push_back(nbInfo);
		else // nbInfo->hop > hop
			nbGreater.push_back(nbInfo);
	}
	const Coord rsuPos = MobilityObserver::Instance2()->globalPosition[rsuAddr];
	const double remainingFlyingDist = (remainingMs - stopFlyingMs)*flyingSpeed/1000.0;
	for (i = 0; i < sectorNum; ++i)
	{
		forbidden[i] = 0;
		double phi = (i + 0.5) * radTheta;
		Coord dst(curPosition);
		dst.x += remainingFlyingDist * cos(phi);
		dst.y += remainingFlyingDist * sin(phi);
		double dist2 = curPosition.sqrdist(rsuPos);
		if (dist2 < minGap2) // boot process
		{
			forbidden[i] = dist2 > dst.sqrdist(rsuPos) ? 1 : 0;
			continue;
		}
		// case 1: leave any neighbor with greater hop count
		bool flag = false;
		for (it = nbGreater.begin(); it != nbGreater.end(); ++it)
			flag |= dst.distance((*it)->pos) > U2URadius - ((*it)->remainingMs - stopFlyingMs)*flyingSpeed/1000.0;
		forbidden[i] = flag ? 1 : 0;
		if (flag)
			continue;
		// case 2: leave all neighbors with less hop count
		flag = true;
		for (it = nbLess.begin(); it != nbLess.end(); ++it)
			flag &= dst.distance((*it)->pos) > U2URadius - ((*it)->remainingMs - stopFlyingMs)*flyingSpeed/1000.0;
		if (flag)
		{
			bool closest = true;
			for (it = nbEqual.begin(); it != nbEqual.end(); ++it)
				if (dst.distance((*it)->pos) > U2URadius - ((*it)->remainingMs - stopFlyingMs)*flyingSpeed/1000.0 && (*it)->hopDist <= hopDist)
					closest = false;
			forbidden[i] = closest ? 1 : 0;
			if (closest)
				continue;
		}
		// case 3: move too close to neighbors or rsu
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		{
			if (dst.sqrdist(itN->second->pos) < minGap2)
			{
				forbidden[i] = 1;
				break;
			}
		}
	}
	EV << "forbidden:";
	int forbiddenNum = 0;
	for (i = 0; i < sectorNum; ++i)
	{
		EV << ' ' << forbidden[i];
		forbiddenNum += forbidden[i];
	}
	EV << std::endl;
	return forbiddenNum;
}
