//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "veins/modules/routing/GPCR.h"

Define_Module(GPCR);

void GPCR::initialize(int stage)
{
	BaseWaveApplLayer::initialize(stage);

	if (stage == 0)
	{

	}
}

void GPCR::finish()
{
	// clear containers
	for (RoutingTable::iterator iter = routingTable.begin(); iter != routingTable.end(); ++iter)
		iter->second.clear();
	routingTable.clear();

	BaseWaveApplLayer::finish();
}

void GPCR::handleSelfMsg(cMessage* msg)
{
	BaseWaveApplLayer::handleSelfMsg(msg);
}

void GPCR::decorateWSM(WaveShortMessage* wsm)
{
	if (std::string(wsm->getName()) == "beacon")
	{
		BeaconMessage *beaconMessage = dynamic_cast<BeaconMessage*>(wsm);
		beaconMessage->setSenderFrom(fromRoadhead);
		beaconMessage->setSenderTo(toRoadhead);
		// TODO: left blank now, for adaptive beaconing in the future
	}
	else if (std::string(wsm->getName()) == "routing")
	{
		RoutingMessage *routingMessage = dynamic_cast<RoutingMessage*>(wsm);
		int hopCount = routingMessage->getHopCount();
		++hopCount;
		routingMessage->setHopCount(hopCount);

		HopItems &hopInfo = routingMessage->getHopInfo();
		HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
		hopInfo.push_back(hopItem);
	}
	else if (std::string(wsm->getName()) == "warning")
	{
		EV << "GPCR doesn't decorate warning message since it is a unicast routing protocol.\n";
		delete wsm;
		wsm = nullptr;
	}
	else if (std::string(wsm->getName()) == "data")
	{
		// Unused
	}
	else
	{
		EV << "unknown message (" << wsm->getName() << ")  decorated, delete it\n";
		delete wsm;
		wsm = nullptr;
	}
}

void GPCR::onBeacon(BeaconMessage* wsm)
{
	BaseWaveApplLayer::onBeacon(wsm);
}

void GPCR::onRouting(RoutingMessage* wsm)
{
	// check if I am the chosen next hop relay vehicle determined by the previous hop
	if ( wsm->getNextHop() != myAddr )
	{
		EV << "I am not the chosen next hop relay vehicle determined by the previous hop, discard it.\n";
		++RoutingStatisticCollector::globalDuplications;
		delete wsm;
		wsm = nullptr;
		return;
	}

	if ( wsm->getReceiver() != myAddr )
		EV << "I am the chosen next hop relay vehicle determined by the previous hop.\n";
	else
	{
		EV << "I am the destination vehicle, search for routing path succeed, report it to request vehicle.\n";
		// record routing statistics
		++RoutingStatisticCollector::globalArrivals;
		double timeDifferentia = simTime().dbl() - wsm->getTimestamp().dbl();
		RoutingStatisticCollector::globalDelayAccumulation += timeDifferentia;
		EV << "the routing path has " << wsm->getHopCount() << " hop count, search for it costs time " << timeDifferentia << "s.\n";

		HopItems &hopInfo = wsm->getHopInfo();
		hopInfo.reverse();
		wsm->setNextHop(hopInfo.front().addr);
		HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
		hopInfo.push_front(hopItem);
		wsm->setBackward(true);
		wsm->setRoutingSuccess(true);
		int hopCount = wsm->getHopCount();
		--hopCount;
		wsm->setHopCount(hopCount);
		sendWSM(wsm);
		return;
	}

	if ( wsm->getHopCount() > maxHopConstraint )
	{
		EV << "hop count exceeds max hop constraint, search for routing path failed, report it to request vehicle.\n";

		HopItems &hopInfo = wsm->getHopInfo();
		hopInfo.reverse();
		wsm->setNextHop(hopInfo.front().addr);
		HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
		hopInfo.push_front(hopItem);
		wsm->setBackward(true);
		wsm->setRoutingSuccess(false);
		int hopCount = wsm->getHopCount();
		--hopCount;
		wsm->setHopCount(hopCount);
		sendWSM(wsm);
		return;
	}

	if ( !wsm->getBackward() )
	{
		LAddress::L3Type nextHop = selectNextHop(wsm);
		if (nextHop == -1)
		{
			EV << "no neighbors fits relay condition, search for routing path failed, report it to request vehicle.\n";

			HopItems &hopInfo = wsm->getHopInfo();
			hopInfo.reverse();
			wsm->setNextHop(hopInfo.front().addr);
			HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
			hopInfo.push_front(hopItem);
			wsm->setBackward(true);
			wsm->setRoutingSuccess(false);
			sendWSM(wsm);
			return;
		}
		EV << "the chosen next hop is " << nextHop << ".\n";
		wsm->setNextHop(nextHop);
		// add this hop info to wsm
		decorateWSM(wsm);
	}
	else
	{
		HopItems &hopInfo = wsm->getHopInfo();
		if (hopInfo.back().addr == myAddr)
		{
			if ( wsm->getRoutingSuccess() )
				onRoutingSuccess(wsm);
			else
				onRoutingFail(wsm);
			delete wsm;
			wsm = nullptr;
			return;
		}
		for (HopItems::iterator iter = hopInfo.begin(); iter != hopInfo.end(); ++iter)
		{
			if (iter->addr == myAddr)
			{
				++iter;
				wsm->setNextHop(iter->addr);
				break;
			}
		}
	}

	// send decorated wsm to next hop
	sendWSM(wsm);
}

void GPCR::onWarning(WarningMessage* wsm)
{
	EV << "GPCR doesn't handle warning message since it is a unicast routing protocol.\n";
	delete wsm;
	wsm = nullptr;
}

void GPCR::onData(DataMessage* wsm)
{
	EV << "GPCR doesn't handle data message since it is a unicast routing protocol.\n";
	delete wsm;
	wsm = nullptr;
}

void GPCR::examineNeighbors()
{
	BaseWaveApplLayer::examineNeighbors();
}

LAddress::L3Type GPCR::selectNextHop(RoutingMessage* wsm)
{
	if ( neighbors.empty() )
		return -1;

	if ( neighbors.find(wsm->getReceiver()) != neighbors.end() )
		return wsm->getReceiver();

	std::set<LAddress::L3Type> hopsPassed;
	HopItems &hopInfo = wsm->getHopInfo();
	for (HopItems::iterator it = hopInfo.begin(); it != hopInfo.end(); ++it)
		hopsPassed.insert(it->addr);

	// destination is out of one hop range, choose a farthest neighbor as relay node
	LAddress::L3Type nextHop = -1;
	double maxDistance = 0.0;
	for (std::map<LAddress::L3Type, NeighborInfo*>::iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter)
	{
		if ( hopsPassed.find(iter->first) != hopsPassed.end() )
			continue;
		Coord &nbPos = iter->second->pos; // alias
		// TODO: how to use greedy forwarding without information of the destination vehicle ?
		// double distance = sqrt((curPosition.x - nbPos.x)*(curPosition.x - nbPos.x) + (curPosition.y - nbPos.y)*(curPosition.y - nbPos.y) + (curPosition.z - nbPos.z)*(curPosition.z - nbPos.z));
		double distance = nbPos.x - curPosition.x; // this line should be write instead!
		if (maxDistance < distance)
		{
			maxDistance = distance;
			nextHop = iter->first;
		}
	}
	return nextHop;
}

void GPCR::callRouting(LAddress::L3Type receiver)
{
	bubble("call routing");
	// create message by factory method
	WaveShortMessage *wsm = prepareWSM("routing", routingLengthBits, type_CCH, routingPriority, -1);
	if (wsm == nullptr) return;
	RoutingMessage *routingMessage = dynamic_cast<RoutingMessage*>(wsm);
	// handle utils about message's GUID
	int guid = RoutingUtils::generateGUID();
	EV << "GUID = " << guid << ", receiver = " << receiver << std::endl;
	guidUsed.push_back(guid);
	scheduleAt(simTime() + guidUsedTime, recycleGUIDEvt);
	// set necessary routing info
	routingMessage->setGUID(guid);
	routingMessage->setReceiver(receiver);
	LAddress::L3Type nextHop = selectNextHop(routingMessage);
	if (nextHop == -1)
	{
		EV << "request vehicle has no neighbors, routing failed.\n";
		delete routingMessage;
		routingMessage = nullptr;
		return;
	}
	EV << "the chosen next hop is " << nextHop << ".\n";
	routingMessage->setHopCount(0);
	routingMessage->setNextHop(nextHop);
	routingMessage->setBackward(false);
	// insert into message memory to avoid relaying the message that is rebroadcast back to self by others
	messageMemory.insert(std::pair<int, WaveShortMessage*>(guid, routingMessage));
	// what to send is a duplication of wsm, because it will be encapsulated into Mac80211Pkt as well as be delete when this message memory forgets,
	// however if send the wsm, it will cannot be deleted when memory forgets.
	RoutingMessage *dupWSM = new RoutingMessage(*routingMessage);
	decorateWSM(dupWSM);
	sendWSM(dupWSM);

	++RoutingStatisticCollector::globalRequests;
}

void GPCR::onRoutingSuccess(RoutingMessage* wsm)
{
	EV << logName() << ": succeed to find a routing path to destination " << wsm->getReceiver() << std::endl;

	HopItems &hopItems = wsm->getHopInfo();
	std::list<LAddress::L3Type> routingPath;

	for (HopItems::reverse_iterator iter = hopItems.rbegin(); iter != hopItems.rend(); ++iter)
		routingPath.push_back(iter->addr);

	routingTable.insert(std::pair<LAddress::L3Type, std::list<LAddress::L3Type> >(wsm->getReceiver(), routingPath));

	displayRoutingTable();
}

void GPCR::onRoutingFail(RoutingMessage* wsm)
{
	EV << logName() << ": failed to find a routing path to destination " << wsm->getReceiver() << std::endl;
	// TODO: wait little time then try again?
}

bool GPCR::checkRoutingTable(LAddress::L3Type destination)
{
	return routingTable.find(destination) != routingTable.end();
}

void GPCR::displayRoutingTable()
{
	EV << "display my routing table:\n";
	for (RoutingTable::iterator iter = routingTable.begin(); iter != routingTable.end(); ++iter)
	{
		EV << "    routing path to node[" << iter->first << "] lists as follows:";
		std::list<LAddress::L3Type> &routingPath = iter->second; // alias
		// std::copy(routingPath.begin(), routingPath.end(), std::ostream_iterator<LAddress::L3Type>(std::cout, " "));
		for (std::list<LAddress::L3Type>::iterator it = routingPath.begin(); it != routingPath.end(); ++it)
			EV << ' ' << *it;
		EV << std::endl;
	}
}

void GPCR::onPathBroken(RoutingMessage* wsm)
{
	EV << "GPCR::onPathBroken() unimplemented yet!\n";
}

GPCR::~GPCR()
{

}
