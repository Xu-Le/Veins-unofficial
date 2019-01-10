//
// Copyright (C) 2016-2019 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/rsu/RoutingRSU.h"
#include "veins/modules/uav/DeployStatisticCollector.h"

Define_Module(RoutingRSU);

std::map<LAddress::L3Type, LAddress::L3Type> rsuBelongingMap;
#ifndef USE_VIRTUAL_FORCE_MODEL
int RoutingRSU::expand = 0;
int RoutingRSU::sectorNum = 0;
double RoutingRSU::radTheta = 0.0;
#endif
int RoutingRSU::uavIndexCounter = 0;
const simsignalwrap_t RoutingRSU::optimalityCalculationSignal = simsignalwrap_t("optimalityCalculation");

void RoutingRSU::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		MobilityObserver::Instance2()->insert(myAddr, curPosition, Coord::ZERO);
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
#endif
		beaconLengthBits = par("beaconLengthBits").longValue();
		beaconPriority = par("beaconPriority").longValue();
		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();
		curUavIndex = 0;
		totalUavNum = par("totalUavNum").longValue();
		long emitFirstUavAt = par("emitFirstUavAt").longValue();

		beaconInterval = par("beaconInterval").doubleValue();
		examineUavsInterval = par("examineUavsInterval").doubleValue();
		emitUavInterval = par("emitUavInterval").doubleValue();
#ifndef USE_VIRTUAL_FORCE_MODEL
		attainDensityInterval = par("attainDensityInterval").doubleValue();
#endif
		UavElapsed = par("UavElapsed").doubleValue();

		rootModule->subscribe(optimalityCalculationSignal, this);

		sendUavBeaconEvt = new cMessage("send uav beacon evt", RoutingRSUMessageKinds::SEND_UAV_BEACON_EVT);
		examineUavsEvt = new cMessage("examine uavs evt", RoutingRSUMessageKinds::EXAMINE_UAVS_EVT);
		emitUavEvt = new cMessage("emit uav evt", RoutingRSUMessageKinds::EMIT_UAV_EVT);
		emitUavEvt->setSchedulingPriority(1); // emit UAV after attain density event
#ifndef USE_VIRTUAL_FORCE_MODEL
		attainDensityEvt = new cMessage("attain density evt", RoutingRSUMessageKinds::ATTAIN_DENSITY_EVT);
		scheduleAt(SimTime(emitFirstUavAt, SIMTIME_S), attainDensityEvt);
#endif
		scheduleAt(simTime() + dblrand()*beaconInterval, sendUavBeaconEvt);
		scheduleAt(simTime() + dblrand()*examineUavsInterval, examineUavsEvt);
		scheduleAt(SimTime(emitFirstUavAt, SIMTIME_S), emitUavEvt);
		scheduleAt(simTime() + dblrand()*forgetMemoryInterval, forgetMemoryEvt);
	}
}

void RoutingRSU::finish()
{
	MobilityObserver::Instance2()->erase(myAddr);

	// clear containers
	for (itU = UAVs.begin(); itU != UAVs.end(); ++itU)
		delete itU->second;
	UAVs.clear();
	accessTable.clear();

	// delete handle self message
	cancelAndDelete(sendUavBeaconEvt);
	cancelAndDelete(examineUavsEvt);
	cancelAndDelete(emitUavEvt);
#ifndef USE_VIRTUAL_FORCE_MODEL
	cancelAndDelete(attainDensityEvt);
#endif

	rootModule->unsubscribe(optimalityCalculationSignal, this);

	BaseRSU::finish();
}

void RoutingRSU::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
	Enter_Method_Silent();

	if (signalID == optimalityCalculationSignal)
	{
		DeployStatisticCollector *collector = dynamic_cast<DeployStatisticCollector*>(obj);
		std::list<LAddress::L3Type> vehicleList;
		for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
			vehicleList.push_back(itV->first);
		collector->importVehicles(vehicleList);
	}
}

void RoutingRSU::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case RoutingRSUMessageKinds::SEND_UAV_BEACON_EVT:
	{
		sendUavBeacon();
		scheduleAt(simTime() + beaconInterval, sendUavBeaconEvt);
		break;
	}
	case RoutingRSUMessageKinds::EXAMINE_UAVS_EVT:
	{
		examineUAVs();
		scheduleAt(simTime() + examineUavsInterval, examineUavsEvt);
		break;
	}
	case RoutingRSUMessageKinds::EMIT_UAV_EVT:
	{
		emitUAV();
		if (++curUavIndex < totalUavNum)
			scheduleAt(simTime() + emitUavInterval, emitUavEvt);
		break;
	}
#ifndef USE_VIRTUAL_FORCE_MODEL
	case RoutingRSUMessageKinds::ATTAIN_DENSITY_EVT:
	{
		attainDensity();
		scheduleAt(simTime() + attainDensityInterval, attainDensityEvt);
		break;
	}
#endif
	default:
		BaseRSU::handleSelfMsg(msg);
	}
}

void RoutingRSU::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "uavBeacon") == 0)
		DYNAMIC_CAST_CMESSAGE(UavBeacon, uavBeacon)
	else if (strcmp(msg->getName(), "uavNotify") == 0)
		DYNAMIC_CAST_CMESSAGE(UavNotify, uavNotify)
	else if (strcmp(msg->getName(), "routing") == 0)
		DYNAMIC_CAST_CMESSAGE(Routing, routing)
	else if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseRSU::handleLowerMsg(msg);
}

void RoutingRSU::handleLowerControl(cMessage *msg)
{
	DataMessage *dataMsg = dynamic_cast<DataMessage*>(msg);
	ASSERT(dataMsg != nullptr);
	onDataLost(dataMsg);
}

void RoutingRSU::sendUavBeacon()
{
	EV << "Creating UAV Beacon with Priority " << beaconPriority << " at RoutingRSU at " << simTime() << std::endl;
	UavBeaconMessage *uavBeaconMsg = new UavBeaconMessage("uavBeacon");
	prepareWSM(uavBeaconMsg, beaconLengthBits, t_channel::type_CCH, beaconPriority, -1);
#ifndef USE_VIRTUAL_FORCE_MODEL
	uavBeaconMsg->setAverageDensity(averageDensity);
	uavBeaconMsg->setDensityDivision(densityDivision);
#endif
	sendWSM(uavBeaconMsg);
}

void RoutingRSU::onUavBeacon(UavBeaconMessage *uavBeaconMsg)
{
	EV << logName() << ": onUavBeacon!\n";

	LAddress::L3Type sender = uavBeaconMsg->getSenderAddress(); // alias
	UAVInfo *uavInfo = nullptr;
	if ((itU = UAVs.find(sender)) != UAVs.end()) // update old record
	{
		EV << "    sender [" << sender << "] is an old UAV, update its info.\n";
		uavInfo = itU->second;
		uavInfo->pos = uavBeaconMsg->getSenderPos();
		uavInfo->speed = uavBeaconMsg->getSenderSpeed();
		uavInfo->receivedAt = simTime();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new UAV, insert its info.\n";
		uavInfo = new UAVInfo(uavBeaconMsg->getSenderPos(), uavBeaconMsg->getSenderSpeed(), simTime());
		UAVs.insert(std::pair<LAddress::L3Type, UAVInfo*>(sender, uavInfo));
	}
}

void RoutingRSU::onUavNotify(UavNotifyMessage *uavNotifyMsg)
{
	EV << logName() << ": onUavNotify!\n";

	if (uavNotifyMsg->getReceiver() != myAddr)
		return;

	AccessHopList &path = uavNotifyMsg->getHopList();
	for (AccessHopList::iterator it = path.begin(); it != path.end(); ++it)
	{
		// any original direct previous hops that are not neighbors anymore need to be updated to this one
		if (accessTable.find(*it) != accessTable.end())
			for (itAT = accessTable.begin(); itAT != accessTable.end(); ++itAT)
				if (itAT->second == accessTable[*it] && UAVs.find(itAT->second) == UAVs.end())
					itAT->second = path.back();
		accessTable[*it] = path.back(); // path.back() is previous hop
	}
	uavNotifyMsg->setReceiver(path.back());
	uavNotifyMsg->setAcknowledgment(true);
	// what to send is a duplication of uavNotifyMsg, because it will be deleted in BaseRSU::handleMessage()
	UavNotifyMessage *dupUavNotifyMsg = new UavNotifyMessage(*uavNotifyMsg);
	sendWSM(dupUavNotifyMsg);

	EV << logName() << " receives notify at " << simTime() << ", routing table:\n";
	for (itAT = accessTable.begin(); itAT != accessTable.end(); ++itAT)
		EV << "  " << itAT->first << " -> " << itAT->second << std::endl;
}

void RoutingRSU::onRouting(RoutingMessage *routingMsg)
{
	EV << logName() << ": onRouting!\n";
}

void RoutingRSU::onData(DataMessage *dataMsg)
{
	EV << logName() << ": onData!\n";
}

void RoutingRSU::onDataLost(DataMessage *lostDataMsg)
{
	EV << logName() << ": onDataLost!\n";
}

void RoutingRSU::examineUAVs()
{
	double curTime = simTime().dbl(); // alias
	for (itU = UAVs.begin(); itU != UAVs.end();)
	{
		if ( curTime - itU->second->receivedAt.dbl() > UavElapsed )
		{
			EV << logName() << " disconnected from UAV[" << itU->first << "], delete its info.\n";
			// TODO: do something before it is deleted
			delete itU->second;
			UAVs.erase(itU++);
		}
		else
			++itU;
	}
}

void RoutingRSU::emitUAV()
{
	EV << logName() << " emit uav[" << uavIndexCounter << "] at " << simTime() << std::endl;

	cModuleType *moduleType = cModuleType::get("org.car2x.veins.nodes.UAV");
	int numRSU = rootModule->par("numRSU").longValue();
	cModule *mod = moduleType->create("uav", rootModule, numRSU*totalUavNum, uavIndexCounter);
	mod->par("posX").setDoubleValue(curPosition.x);
	mod->par("posY").setDoubleValue(curPosition.y);
	mod->finalizeParameters();
	mod->getDisplayString().parse("b=8,8,oval");
	mod->getDisplayString().setTagArg("r", 0, V2XRadius);
	mod->buildInside();
	rsuBelongingMap[uavIndexCounter+UAV_ADDRESS_OFFSET] = myAddr;
	mod->callInitialize();

	++uavIndexCounter;
}

#ifndef USE_VIRTUAL_FORCE_MODEL
void RoutingRSU::attainDensity()
{
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
	const double closeMulitplier = 1.0;
	std::vector<double> sectorDensity(sectorNum, 0.0);
	std::vector<double> sectorDensity0(sectorNum, 0.0);
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		VehicleInfo *veh = itV->second;
		double angle = acos((veh->pos.x - curPosition.x) / curPosition.distance(veh->pos));
		if (veh->pos.y < curPosition.y)
			angle = 2*M_PI - angle;
		int sector = angle / radTheta;
		double coverCoefficient = 1.0, closeCoefficient = 0.0;
		for (itU = UAVs.begin(); itU != UAVs.end(); ++itU)
			if ((closeCoefficient = veh->pos.distance(itU->second->pos)/V2XRadius) < 1.0)
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

	double maxSectorDensity = sectorDensity[0];
	averageDensity = sectorDensity[0];
	for (int i = 1; i < sectorNum; ++i)
	{
		averageDensity += sectorDensity[i];
		if (maxSectorDensity < sectorDensity[i])
			maxSectorDensity = sectorDensity[i];
	}
	averageDensity /= sectorNum;
	densityDivision = fabs(averageDensity) > Epsilon ? maxSectorDensity/averageDensity : 1000000.0;
	EV << "\naverage density: " << averageDensity << ", density division: " << densityDivision << std::endl;
}
#endif
