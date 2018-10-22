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

int RoutingRSU::uavIndexCounter = 0;
const simsignalwrap_t RoutingRSU::optimalityCalculationSignal = simsignalwrap_t("optimalityCalculation");

void RoutingRSU::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		MobilityObserver::Instance2()->insert(myAddr, curPosition, Coord::ZERO);

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
		UavElapsed = par("UavElapsed").doubleValue();

		rootModule->subscribe(optimalityCalculationSignal, this);

		sendUavBeaconEvt = new cMessage("send uav beacon evt", RoutingRSUMessageKinds::SEND_UAV_BEACON_EVT);
		examineUavsEvt = new cMessage("examine uavs evt", RoutingRSUMessageKinds::EXAMINE_UAVS_EVT);
		emitUavEvt = new cMessage("emit uav evt", RoutingRSUMessageKinds::EMIT_UAV_EVT);
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

void RoutingRSU::sendUavBeacon()
{
	EV << "Creating UAV Beacon with Priority " << beaconPriority << " at RoutingRSU at " << simTime() << std::endl;
	UavBeaconMessage *uavBeaconMsg = new UavBeaconMessage("uavBeacon");
	prepareWSM(uavBeaconMsg, beaconLengthBits, t_channel::type_CCH, beaconPriority, -1);
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
		accessTable[*it] = path.back(); // path.back() is previous hop
	uavNotifyMsg->setReceiver(path.back());
	uavNotifyMsg->setAcknowledgment(true);
	// what to send is a duplication of uavNotifyMsg, because it will be deleted in BaseRSU::handleMessage()
	UavNotifyMessage *dupUavNotifyMsg = new UavNotifyMessage(*uavNotifyMsg);
	sendWSM(dupUavNotifyMsg);

	EV << logName() << " receive notify at " << simTime() << ", routing table:\n";
	for (itAT = accessTable.begin(); itAT != accessTable.end(); ++itAT)
		EV << "  " << itAT->first << " -> " << itAT->second << std::endl;
}

void RoutingRSU::onRouting(RoutingMessage *routingMsg)
{
	EV << logName() << ": onRouting!\n";

	int guid = routingMsg->getGUID(); // alias

	if ( messageMemory.find(guid) != messageMemory.end() )
	{
		EV << "routing message(GUID=" << guid << ") has been rebroadcast recently, discard it.\n";
		return;
	}

	// check if the hop count of the message exceeds
	if ( routingMsg->getHopCount() > maxHopConstraint )
	{
		EV << "routing message(GUID=" << guid << ") exceeds its maximum hop count, discard it.\n";
		return;
	}

	// catch a new routing message
	EV << "catch a routing message(GUID=" << guid << "), help to rebroadcast it.\n";

	messageMemory.insert(std::pair<int, simtime_t>(guid, simTime()));

	RoutingMessage *dupWSM = new RoutingMessage(*routingMsg);
	sendWSM(dupWSM);
}

void RoutingRSU::onData(DataMessage *dataMsg)
{
	EV << logName() << ": onData!\n";
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
