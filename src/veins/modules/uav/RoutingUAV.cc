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

#include <algorithm>
#include "veins/modules/uav/RoutingUAV.h"

Define_Module(RoutingUAV);

extern std::map<LAddress::L3Type, LAddress::L3Type> rsuBelongingMap;

int RoutingUAV::stopFlyingMs = 0;
#ifndef USE_VIRTUAL_FORCE_MODEL
int RoutingUAV::expand = 0;
int RoutingUAV::sectorNum = 0;
double RoutingUAV::radTheta = 0.0;
double RoutingUAV::minGap = 0.0;
#else
double RoutingUAV::k_a = 0.0;
double RoutingUAV::K_r = 0.0;
double RoutingUAV::R_opt = 0.0;
#endif

void RoutingUAV::initialize(int stage)
{
	BaseUAV::initialize(stage);

	if (stage == 0)
	{
		rsuAddr = rsuBelongingMap[myAddr];

		hop = 1;
		hopDist = 0.0;
#ifndef USE_VIRTUAL_FORCE_MODEL
		int theta = par("theta").longValue();
		if (360 % theta != 0)
			throw cRuntimeError("theta %d cannot be divided with no remainder by 360 degree", theta);
		expand = par("expand").longValue();
		if (expand % 2 == 0 || expand * theta > 180)
			throw cRuntimeError("expand must be odd and expand * theta must less than or equal to 180 degree");
		radTheta = M_PI * theta / 180.0;
		sectorNum = 360 / theta;
		averageDensity = 0.0;
		densityDivision = 1.0;
		minGap = par("minGap").doubleValue();
#else
		k_a = par("k_a").doubleValue();
		K_r = par("K_r").doubleValue();
		R_opt = par("R_opt").doubleValue();
#endif
		stopFlyingMs = par("stopFlyingMs").longValue();

		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();

		curDirection = Coord::ZERO;
#ifndef USE_VIRTUAL_FORCE_MODEL
		sectorDensity.resize(sectorNum, 0.0);
		forbidden.resize(sectorNum, 0);
#endif

		flyingInterval = SimTime(par("flyingInterval").longValue(), SIMTIME_MS);
		decideInterval = SimTime(par("decideInterval").longValue(), SIMTIME_MS);
		nextDecisionAt = simTime() + beaconInterval;
		if (stopFlyingMs > par("decideInterval").longValue())
			throw cRuntimeError("stopFlyingMs must less than or equal to decideInterval");

		flyingEvt = new cMessage("flying evt", RoutingUAVMessageKinds::FLYING_EVT);
		decideEvt = new cMessage("decide evt", RoutingUAVMessageKinds::DECIDE_EVT);
		scheduleAt(nextDecisionAt, decideEvt);
	}
	else if (stage == 1)
		flyingSpeed = mobility->par("flyingSpeed").doubleValue(); // reuse the parameter in mobility module
}

void RoutingUAV::finish()
{
	// clear containers
#ifndef USE_VIRTUAL_FORCE_MODEL
	sectorDensity.clear();
	forbidden.clear();
#endif
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

void RoutingUAV::handleLowerControl(cMessage *msg)
{
	DataMessage *dataMsg = dynamic_cast<DataMessage*>(msg);
	ASSERT(dataMsg != nullptr);
	onDataLost(dataMsg);
}

void RoutingUAV::decorateUavBeacon(UavBeaconMessage *uavBeaconMsg)
{
	uavBeaconMsg->setHop(hop);
	uavBeaconMsg->setDecideAt(nextDecisionAt - decideInterval);
	uavBeaconMsg->setHopDist(hopDist);
#ifndef USE_VIRTUAL_FORCE_MODEL
	uavBeaconMsg->setAverageDensity(averageDensity);
	uavBeaconMsg->setDensityDivision(densityDivision);
#endif
}

void RoutingUAV::onUavBeacon(UavBeaconMessage *uavBeaconMsg)
{
	EV << logName() << ": onUavBeacon!\n";

	LAddress::L3Type sender = uavBeaconMsg->getSenderAddress(); // alias
	RoutingNeighborInfo *neighborInfo = nullptr;
	if ((itN = neighbors.find(sender)) != neighbors.end()) // update old record
	{
		EV << "    sender [" << sender << "] is an old neighbor, update its info.\n";
		neighborInfo = dynamic_cast<RoutingNeighborInfo*>(itN->second);
		neighborInfo->pos = uavBeaconMsg->getSenderPos();
		neighborInfo->speed = uavBeaconMsg->getSenderSpeed();
		neighborInfo->receivedAt = simTime();
		neighborInfo->hop = uavBeaconMsg->getHop();
		neighborInfo->decideAt = uavBeaconMsg->getDecideAt();
		neighborInfo->hopDist = uavBeaconMsg->getHopDist();
		neighborInfo->averageDensity = uavBeaconMsg->getAverageDensity();
		neighborInfo->densityDivision = uavBeaconMsg->getDensityDivision();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new neighbor, insert its info.\n";
		neighborInfo = new RoutingNeighborInfo(uavBeaconMsg->getSenderPos(), uavBeaconMsg->getSenderSpeed(), simTime(), uavBeaconMsg->getHop(),
			uavBeaconMsg->getDecideAt(), uavBeaconMsg->getHopDist(), uavBeaconMsg->getAverageDensity(), uavBeaconMsg->getDensityDivision());
		neighbors.insert(std::pair<LAddress::L3Type, NeighborInfo*>(sender, neighborInfo));
	}
	onNeighborUpdate();

	EV << "display all neighbors' information:\n";
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
	{
		NeighborInfo *baseNbInfo = itN->second;
		neighborInfo = dynamic_cast<RoutingNeighborInfo*>(baseNbInfo);
		if (itN->first >= UAV_ADDRESS_OFFSET)
			EV << "uav[" << itN->first - UAV_ADDRESS_OFFSET << "]:  ";
		else
			EV << "rsu[" << itN->first - RSU_ADDRESS_OFFSET << "]:  ";
		EV << "decideAt: " << neighborInfo->decideAt << ", hop: " << neighborInfo->hop << ", hopDist: " << neighborInfo->hopDist
			<< ", density: (" << neighborInfo->averageDensity << ',' << neighborInfo->densityDivision << ")\n";
	}
	EV << std::endl;
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
		LAddress::L3Type prevHop = path.back(); // path.back() is previous hop
		AccessTable &prevAccessTable = uavNotifyMsg->getTable();
		// delete indirect previous hops that cannot be routed from the direct previous hop anymore
		for (itAT = accessTable.begin(); itAT != accessTable.end();)
		{
			if (itAT->second == prevHop && itAT->first != itAT->second && prevAccessTable.find(itAT->first) == prevAccessTable.end())
				accessTable.erase(itAT++);
			else
				++itAT;
		}
		// insert indirect previous hops that can be routed from the direct previous hop
		for (itAT = prevAccessTable.begin(); itAT != prevAccessTable.end(); ++itAT)
		{
			if (itAT->first != rsuAddr && itAT->first != myAddr && accessTable.find(itAT->first) == accessTable.end() && neighbors.find(itAT->first) == neighbors.end())
			{
				int nextHop = neighbors.find(itAT->second) != neighbors.end() ? itAT->second : prevHop;
				accessTable.insert(std::pair<LAddress::L3Type, LAddress::L3Type>(itAT->first, nextHop));
			}
		}
		for (AccessHopList::iterator it = path.begin(); it != path.end(); ++it)
		{
			// any original direct previous hops that are not neighbors anymore need to be updated to this one
			if (accessTable.find(*it) != accessTable.end())
				for (itAT = accessTable.begin(); itAT != accessTable.end(); ++itAT)
					if (itAT->second == accessTable[*it] && neighbors.find(itAT->second) == neighbors.end())
						itAT->second = prevHop;
			accessTable[*it] = prevHop;
		}
		path.push_back(myAddr);
		uavNotifyMsg->setReceiver(accessTable[rsuAddr]);
		if (uavNotifyMsg->getReceiver() == rsuAddr) // RSU do not need previous hop's access table
			uavNotifyMsg->getTable().clear();
		EV << logName() << " relays notify at " << simTime() << ", routing table:\n";
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
}

void RoutingUAV::onData(DataMessage* dataMsg)
{
	EV << logName() << ": onData!\n";
}

void RoutingUAV::onDataLost(DataMessage *lostDataMsg)
{
	EV << logName() << ": onDataLost!\n";
}

void RoutingUAV::onNeighborUpdate()
{
	int minNbHop = 1000, nextHop = -1;
	double minNbDist = 100000.0;
	LAddress::L3Type originalNext = accessTable[rsuAddr];
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
	bool needNotify = hop != minNbHop + 1 || originalNext != nextHop;
	if (needNotify)
	{
		UavNotifyMessage *uavNotifyMsg = new UavNotifyMessage("uavNotify");
		prepareWSM(uavNotifyMsg, routingLengthBits, type_CCH, routingPriority, -1);
		uavNotifyMsg->setReceiver(nextHop);
		uavNotifyMsg->getHopList().push_back(myAddr);
		sendWSM(uavNotifyMsg);
	}
	hop = minNbHop + 1;
	accessTable[rsuAddr] = nextHop;
	if (needNotify)
	{
		EV << logName() << " notify at " << simTime() << ", routing table:\n";
		for (itAT = accessTable.begin(); itAT != accessTable.end(); ++itAT)
			EV << "  " << itAT->first << " -> " << itAT->second << std::endl;
	}
}

void RoutingUAV::examineNeighbors()
{
	size_t oldNum = neighbors.size();
	double curTime = simTime().dbl(); // alias
	for (itN = neighbors.begin(); itN != neighbors.end();)
	{
		if ( curTime - itN->second->receivedAt.dbl() > neighborElapsed )
		{
			EV << logName() << " disconnected from neighbor[" << itN->first << "], delete its info.\n";
			for (itAT = accessTable.begin(); itAT != accessTable.end();)
			{
				if (itAT->first != rsuAddr && itAT->second == itN->first)
					accessTable.erase(itAT++);
				else
					++itAT;
			}
			delete itN->second;
			neighbors.erase(itN++);
		}
		else
			++itN;
	}

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
	EV << logName() << " decide at " << simTime() << "\n";

	remainingMs = decideInterval.inUnit(SIMTIME_MS);

#ifdef ATTAIN_VEHICLE_DENSITY_BY_GOD_VIEW
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		delete itV->second;
	vehicles.clear();
	Coord O;
	std::map<LAddress::L3Type, Coord> &allVeh = MobilityObserver::Instance()->globalPosition;
	for (std::map<LAddress::L3Type, Coord>::iterator it = allVeh.begin(); it != allVeh.end(); ++it)
	{
		if (curPosition.distance(it->second) < V2XRadius)
		{
			VehicleInfo *vehicleInfo = new VehicleInfo(it->second, O, -1, SimTime::ZERO);
			vehicles.insert(std::pair<LAddress::L3Type, VehicleInfo*>(it->first, vehicleInfo));
		}
	}
#endif
#ifndef USE_VIRTUAL_FORCE_MODEL
	attainSectorDensity();

	int forbiddenNum = attainForbiddenSector();
	if (forbiddenNum == sectorNum)
	{
		EV << "all sectors are forbidden." << std::endl;
		return;
	}

	int i = 0;
	double maxSectorDensity = sectorDensity[0];
	averageDensity = sectorDensity[0];
	for (i = 1; i < sectorNum; ++i)
	{
		averageDensity += sectorDensity[i];
		if (maxSectorDensity < sectorDensity[i])
			maxSectorDensity = sectorDensity[i];
	}
	averageDensity /= sectorNum;
	densityDivision = fabs(averageDensity) > Epsilon ? maxSectorDensity/averageDensity : 1000000.0;
	EV << "average density: " << averageDensity << ", density division: " << densityDivision << std::endl;

	const Coord rsuPos = MobilityObserver::Instance2()->globalPosition[rsuAddr];
	if (curPosition.distance(rsuPos) > minGap)
	{
		std::vector<double> nbAverage, nbDivision;
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		{
			RoutingNeighborInfo *nbInfo = dynamic_cast<RoutingNeighborInfo*>(itN->second);
			nbAverage.push_back(nbInfo->averageDensity);
			nbDivision.push_back(nbInfo->densityDivision);
		}
		int halfNbNum = static_cast<int>(nbAverage.size()) / 2;
		std::nth_element(nbAverage.begin(), nbAverage.begin()+halfNbNum, nbAverage.end());
		std::nth_element(nbDivision.begin(), nbDivision.begin()+halfNbNum, nbDivision.end(), std::greater<double>());
		if (averageDensity > nbAverage[halfNbNum] && densityDivision < nbDivision[halfNbNum])
		{
			EV << "nb-mid average density: " << nbAverage[halfNbNum] << ", nb-mid density division: " << nbDivision[halfNbNum] << std::endl;
			return;
		}
	}

	// select an optimal moving direction
	maxSectorDensity = 0.0;
	int maxI = 0;
	for (i = 0; i < sectorNum; ++i)
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
#else
	attainResultantForce();

	double L = curDirection.length();
	if (FWMath::close(L, 0.0))
	{
		EV << "force is zero." << std::endl;
		return;
	}
	curDirection /= L;
	double phi = acos(curDirection.x);
	if (curDirection.y < 0)
		phi = 2*M_PI - phi;
	std::list<RoutingNeighborInfo*> nbLess, nbEqual, nbGreater;
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
	flyingSpeed = mobility->par("flyingSpeed").doubleValue();
	flyingSpeed *= atan(L)*2/M_PI;
	EV << "force: " << L << ", speed: " << flyingSpeed << ", ";
	const double remainingFlyingDist = (remainingMs - stopFlyingMs)/1000.0*flyingSpeed;
	Coord dst(curPosition);
	dst.x += remainingFlyingDist * curDirection.x;
	dst.y += remainingFlyingDist * curDirection.y;
	if (isForbidden(dst, nbLess, nbEqual, nbGreater))
	{
		EV << "dir is forbidden, pos: " << curPosition.info() << ", dst: " << dst.info() << ", phi: " << 180*phi/M_PI << std::endl;
		return;
	}
#endif

	EV << "pos: " << curPosition.info() << ", phi: " << 180*phi/M_PI << std::endl;

	mobility->setMove(curDirection, flyingSpeed, false, curPosition);
	scheduleAt(simTime() + flyingInterval, flyingEvt);
}

#ifndef USE_VIRTUAL_FORCE_MODEL
void RoutingUAV::attainSectorDensity()
{
	const double closeMulitplier = 1.0;
	std::fill(sectorDensity.begin(), sectorDensity.end(), 0.0);
	std::vector<double> sectorDensity0(sectorNum, 0.0);
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		VehicleInfo *veh = itV->second;
		double diffX = veh->pos.x - curPosition.x, diffY = veh->pos.y - curPosition.y;
		double angle = acos(diffX / sqrt(diffX*diffX + diffY*diffY));
		if (veh->pos.y < curPosition.y)
			angle = 2*M_PI - angle;
		int sector = angle / radTheta;
		double coverCoefficient = 1.0, closeCoefficient = 0.0;
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
			if ((closeCoefficient = veh->pos.distance(itN->second->pos)/V2XRadius) < 1.0)
				coverCoefficient += 1.0 + closeMulitplier*(1.0 - closeCoefficient);
		sectorDensity0[sector] += 1.0/coverCoefficient;
	}
	// smooth the density of each sector
	for (int i = 0; i < sectorNum; ++i)
	{
		int rangeFrom = i - expand/2, rangeTo = i + expand/2;
		if (rangeFrom < 0)
			rangeFrom += sectorNum;
		if (rangeTo < rangeFrom)
			rangeTo += sectorNum;
		for (int j = rangeFrom; j <= rangeTo; ++j)
			sectorDensity[i] += sectorDensity0[j < sectorNum ? j : j - sectorNum];
		sectorDensity[i] /= expand;
	}
	EV << "sector density:";
	for (size_t k = 0; k < sectorDensity.size(); ++k)
		EV << ' ' << sectorDensity[k];
	EV << std::endl;
}

int RoutingUAV::attainForbiddenSector()
{
	int i = 0, forbiddenNum = 0;
	std::list<RoutingNeighborInfo*> nbLess, nbEqual, nbGreater;
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
	const double remainingFlyingDist = (remainingMs - stopFlyingMs)/1000.0*flyingSpeed;
	for (i = 0; i < sectorNum; ++i)
	{
		forbidden[i] = 0;
		double phi = (i + 0.5) * radTheta;
		Coord dst(curPosition);
		dst.x += remainingFlyingDist * cos(phi);
		dst.y += remainingFlyingDist * sin(phi);
		forbidden[i] = isForbidden(dst, nbLess, nbEqual, nbGreater) ? 1 : 0;
	}
	EV << "forbidden:";
	for (i = 0; i < sectorNum; ++i)
	{
		EV << ' ' << forbidden[i];
		forbiddenNum += forbidden[i];
	}
	EV << std::endl;
	return forbiddenNum;
}
#else
void RoutingUAV::attainResultantForce()
{
	curDirection = Coord::ZERO;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		Coord &vpos = itV->second->pos;
		Coord force(vpos.x - curPosition.x, vpos.y - curPosition.y);
		double diffX = vpos.x - curPosition.x, diffY = vpos.y - curPosition.y;
		double dist = sqrt(diffX*diffX + diffY*diffY);
		if (dist < 20.0)
			continue;
		force *= k_a / (dist*dist*dist);
		curDirection += force;
	}
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
	{
		Coord &npos = itN->second->pos;
		double diffX = curPosition.x - npos.x, diffY = curPosition.y - npos.y;
		double dist = sqrt(diffX*diffX + diffY*diffY);
		if (dist < R_opt && dist > R_opt/5.0)
		{
			Coord force(curPosition.x - npos.x, curPosition.y - npos.y);
			force *= K_r * (R_opt - dist) / dist;
			curDirection += force;
		}
	}
}
#endif

bool RoutingUAV::isForbidden(Coord dst, std::list<RoutingNeighborInfo*>& nbLess, std::list<RoutingNeighborInfo*>& nbEqual, std::list<RoutingNeighborInfo*>& nbGreater)
{
	bool flag = false;
	std::list<RoutingNeighborInfo*>::iterator it;
#ifndef USE_VIRTUAL_FORCE_MODEL
	const Coord rsuPos = MobilityObserver::Instance2()->globalPosition[rsuAddr];
	const double minGap2 = minGap * minGap, dist2 = curPosition.sqrdist(rsuPos);
	if (dist2 < minGap2) // boot process
	{
		flag = dst.sqrdist(rsuPos) < dist2; // moving more close to the RSU
		if (flag)
			return true;
		for (it = nbGreater.begin(); it != nbGreater.end(); ++it)
			flag |= dst.sqrdist((*it)->pos) < minGap2;
		if (flag)
			return true;
		for (it = nbEqual.begin(); it != nbEqual.end(); ++it)
			flag |= dst.sqrdist((*it)->pos) < minGap2;
		// note that in boot process, RSU is the only neighbor in nbLess, thus no need to check nbLess
		return flag;
	}
#endif
	// case 1: leave any neighbor with greater hop count
	for (it = nbGreater.begin(); it != nbGreater.end(); ++it)
		flag |= willLeave(dst, *it);
	if (flag)
		return true;
	// case 2: leave all neighbors with less hop count
	flag = true;
	for (it = nbLess.begin(); it != nbLess.end(); ++it)
		flag &= willLeave(dst, *it);
	if (flag)
	{
		bool closest = true;
		for (it = nbEqual.begin(); it != nbEqual.end(); ++it)
			if (!willLeave(dst, *it) && (*it)->hopDist < hopDist)
				closest = false;
		if (closest)
			return true;
	}
	flag = false;
#ifndef USE_VIRTUAL_FORCE_MODEL
	// case 3: move too close to neighbors or RSU
	for (it = nbGreater.begin(); it != nbGreater.end(); ++it)
		flag |= dst.sqrdist((*it)->pos) < minGap2;
	if (flag)
		return true;
	for (it = nbEqual.begin(); it != nbEqual.end(); ++it)
		flag |= dst.sqrdist((*it)->pos) < minGap2;
	if (flag)
		return true;
	for (it = nbLess.begin(); it != nbLess.end(); ++it)
		flag |= dst.sqrdist((*it)->pos) < minGap2;
#endif
	return flag;
}

bool RoutingUAV::willLeave(Coord &self, RoutingNeighborInfo *nbInfo)
{
	if (nbInfo->decideAt == SimTime::ZERO) // only RSU do not set variable decideAt in the packet
		return self.distance(nbInfo->pos) >= U2URadius;
	SimTime sinceDecide = simTime() - nbInfo->decideAt;
	if (sinceDecide > decideInterval) // beacon message may not fresh enough, for safety, regard it is dangerous
		return true;
	//    [---------*------|                ][---------*------|                ]
	//    ^         <------>                  <-------->
	// decideAt    curMovingMs               nextMovingMs
	int flyingMs = decideInterval.inUnit(SIMTIME_MS) - stopFlyingMs, nextMovingMs = sinceDecide.inUnit(SIMTIME_MS);
	int curMovingMs = flyingMs > nextMovingMs ? flyingMs - nextMovingMs : 0;
	Coord nb = nbInfo->pos;
	nb.x += nbInfo->speed.x * curMovingMs / 1000.0;
	nb.y += nbInfo->speed.y * curMovingMs / 1000.0;
	double nextMovingDistance = flyingSpeed * std::min(flyingMs, nextMovingMs) / 1000.0;
	return self.distance(nb) + nextMovingDistance >= U2URadius;
}
