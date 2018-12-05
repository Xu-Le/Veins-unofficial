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
		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();
		maxHopConstraint = par("maxHopConstraint").longValue();

		callRoutings = par("callRoutings").boolValue();
		if (callRoutings)
			initializeRoutingPlanList(par("routingPlan").xmlValue());

		if (sendBeacons && !routingPlanList.empty())
		{
			callRoutingEvt = new cMessage("call routing evt", WaveApplMsgKinds::CALL_ROUTING_EVT);
			scheduleAt(routingPlanList.front().first, callRoutingEvt);
		}
		else
			callRoutingEvt = nullptr;
	}
}

void GPCR::finish()
{
	// clear containers
	for (RoutingTable::iterator iter = routingTable.begin(); iter != routingTable.end(); ++iter)
		iter->second.clear();
	routingTable.clear();
	routingPlanList.clear();

	cancelAndDelete(callRoutingEvt);

	BaseWaveApplLayer::finish();
}

void GPCR::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case WaveApplMsgKinds::CALL_ROUTING_EVT:
	{
		callRouting(routingPlanList.front().second);
		routingPlanList.pop_front();
		if (!routingPlanList.empty())
			scheduleAt(routingPlanList.front().first, callRoutingEvt);
		break;
	}
	default:
		BaseWaveApplLayer::handleSelfMsg(msg);
	}
}

void GPCR::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "routing") == 0)
		DYNAMIC_CAST_CMESSAGE(Routing, routing)
	else if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseWaveApplLayer::handleLowerMsg(msg);
}

void GPCR::handleLowerControl(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		onDataLost(dynamic_cast<DataMessage*>(msg));
}

void GPCR::decorateRouting(RoutingMessage *routingMsg)
{
	int hopCount = routingMsg->getHopCount();
	routingMsg->setHopCount(hopCount + 1);

	HopItems &hopInfo = routingMsg->getHopInfo();
	HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
	hopInfo.push_back(hopItem);
}

void GPCR::onRouting(RoutingMessage *_routingMsg)
{
	EV << "node[" << myAddr << "]: onRouting!\n";

	// what to operate is a duplication of routingMsg, because it will be deleted in BaseWaveApplLayer::handleLowerMsg()
	RoutingMessage *routingMsg = new RoutingMessage(*_routingMsg);

	// check if I am the chosen next hop relay vehicle determined by the previous hop
	if ( routingMsg->getNextHop() != myAddr )
	{
		EV << "I am not the chosen next hop relay vehicle determined by the previous hop, discard it.\n";
		++RoutingStatisticCollector::globalDuplications;
		DELETE_SAFELY(routingMsg);
		return;
	}

	if ( routingMsg->getReceiver() != myAddr )
		EV << "I am the chosen next hop relay vehicle determined by the previous hop.\n";
	else
	{
		EV << "I am the destination vehicle, search for routing path succeed, report it to request vehicle.\n";
		// record routing statistics
		++RoutingStatisticCollector::globalArrivals;
		double timeDifferentia = simTime().dbl() - routingMsg->getTimestamp().dbl();
		RoutingStatisticCollector::globalDelayAccumulation += timeDifferentia;
		EV << "the routing path has " << routingMsg->getHopCount() << " hop count, search for it costs time " << timeDifferentia << "s.\n";

		HopItems &hopInfo = routingMsg->getHopInfo();
		hopInfo.reverse();
		routingMsg->setNextHop(hopInfo.front().addr);
		HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
		hopInfo.push_front(hopItem);
		routingMsg->setBackward(true);
		routingMsg->setRoutingSuccess(true);
		int hopCount = routingMsg->getHopCount();
		--hopCount;
		routingMsg->setHopCount(hopCount);
		sendWSM(routingMsg);
		return;
	}

	if ( routingMsg->getHopCount() > maxHopConstraint )
	{
		EV << "hop count exceeds max hop constraint, search for routing path failed, report it to request vehicle.\n";

		HopItems &hopInfo = routingMsg->getHopInfo();
		hopInfo.reverse();
		routingMsg->setNextHop(hopInfo.front().addr);
		HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
		hopInfo.push_front(hopItem);
		routingMsg->setBackward(true);
		routingMsg->setRoutingSuccess(false);
		int hopCount = routingMsg->getHopCount();
		--hopCount;
		routingMsg->setHopCount(hopCount);
		sendWSM(routingMsg);
		return;
	}

	if ( !routingMsg->getBackward() )
	{
		LAddress::L3Type nextHop = selectNextHop(routingMsg);
		if (nextHop == -1)
		{
			EV << "no neighbors fits relay condition, search for routing path failed, report it to request vehicle.\n";

			HopItems &hopInfo = routingMsg->getHopInfo();
			hopInfo.reverse();
			routingMsg->setNextHop(hopInfo.front().addr);
			HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
			hopInfo.push_front(hopItem);
			routingMsg->setBackward(true);
			routingMsg->setRoutingSuccess(false);
			sendWSM(routingMsg);
			return;
		}
		EV << "the chosen next hop is " << nextHop << ".\n";
		routingMsg->setNextHop(nextHop);
		// add this hop info to routingMsg
		decorateRouting(routingMsg);
	}
	else
	{
		HopItems &hopInfo = routingMsg->getHopInfo();
		if (hopInfo.back().addr == myAddr)
		{
			if ( routingMsg->getRoutingSuccess() )
				onRoutingSuccess(routingMsg);
			else
				onRoutingFail(routingMsg);
			DELETE_SAFELY(routingMsg);
			return;
		}
		for (HopItems::iterator iter = hopInfo.begin(); iter != hopInfo.end(); ++iter)
		{
			if (iter->addr == myAddr)
			{
				++iter;
				routingMsg->setNextHop(iter->addr);
				break;
			}
		}
	}

	// send decorated routingMsg to next hop
	sendWSM(routingMsg);
}

void GPCR::onData(DataMessage *dataMsg)
{
	EV << "node[" << myAddr << "]: onData!\n";
}

void GPCR::onDataLost(DataMessage *lostDataMsg)
{
	EV << "node[" << myAddr << "]: onDataLost!\n";
}

void GPCR::callRouting(LAddress::L3Type receiver)
{
	// create message by factory method
	RoutingMessage *routingMsg = new RoutingMessage("routing");
	prepareWSM(routingMsg, routingLengthBits, type_CCH, routingPriority, -1);

	// handle utils about message's GUID
	int guid = RoutingUtils::generateGUID();
	EV << "GUID = " << guid << ", receiver = " << receiver << std::endl;
	guidUsed.push_back(guid);
	scheduleAt(simTime() + guidUsedTime, recycleGUIDEvt);
	// set necessary routing info
	routingMsg->setGUID(guid);
	routingMsg->setReceiver(receiver);
	LAddress::L3Type nextHop = selectNextHop(routingMsg);
	if (nextHop == -1)
	{
		EV << "request vehicle has no neighbors, routing failed.\n";
		DELETE_SAFELY(routingMsg);
		return;
	}
	EV << "the chosen next hop is " << nextHop << ".\n";
	routingMsg->setHopCount(0);
	routingMsg->setNextHop(nextHop);
	routingMsg->setBackward(false);
	// insert into message memory to avoid relaying the message that is rebroadcast back to self by others
	messageMemory.insert(std::pair<int, WaveShortMessage*>(guid, routingMsg));
	// what to send is a duplication of routingMsg, because it will be encapsulated into Mac80211Pkt as well as be delete when this message memory forgets,
	// however if send the routingMsg, it will cannot be deleted when memory forgets.
	RoutingMessage *duproutingMsg = new RoutingMessage(*routingMsg);
	decorateRouting(duproutingMsg);
	sendWSM(duproutingMsg);

	++RoutingStatisticCollector::globalRequests;
}

void GPCR::initializeRoutingPlanList(cXMLElement *xmlConfig)
{
	if (xmlConfig == 0)
		throw cRuntimeError("No routing plan configuration file specified.");

	cXMLElementList planList = xmlConfig->getElementsByTagName("RoutingPlan");

	if (planList.empty())
		EV << "No routing plan configuration items specified.\n";

	for (cXMLElementList::iterator iter = planList.begin(); iter != planList.end(); ++iter)
	{
		cXMLElement *routingPlan = *iter;

		const char* name = routingPlan->getAttribute("type");

		if (atoi(name) == myAddr)
		{
			cXMLElementList parameters = routingPlan->getElementsByTagName("parameter");

			ASSERT( parameters.size() % 2 == 0 );

			EV << logName() << "'s routing plan list as follows:\n";

			for (size_t i = 0; i < parameters.size(); i += 2)
			{
				double simtime = 0.0;
				long receiver = 0;

				const char *name = parameters[i]->getAttribute("name");
				const char *type = parameters[i]->getAttribute("type");
				const char *value = parameters[i]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value");

				if (strcmp(name, "simtime") == 0 && strcmp(type, "double") == 0)
					simtime = atof(value);

				name = parameters[i+1]->getAttribute("name");
				type = parameters[i+1]->getAttribute("type");
				value = parameters[i+1]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value");

				if (strcmp(name, "receiver") == 0 && strcmp(type, "long") == 0)
					receiver = atoi(value);

				EV << "    simtime: " << simtime << ", receiver: " << receiver << std::endl;
				routingPlanList.push_back(std::pair<double, LAddress::L3Type>(simtime, receiver));
			}

			break;
		}
	}
}

LAddress::L3Type GPCR::selectNextHop(RoutingMessage *routingMsg)
{
	if ( neighbors.empty() )
		return -1;

	if ( neighbors.find(routingMsg->getReceiver()) != neighbors.end() )
		return routingMsg->getReceiver();

	std::set<LAddress::L3Type> hopsPassed;
	HopItems &hopInfo = routingMsg->getHopInfo();
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

void GPCR::onRoutingSuccess(RoutingMessage *routingMsg)
{
	EV << logName() << ": succeed to find a routing path to destination " << routingMsg->getReceiver() << std::endl;

	HopItems &hopItems = routingMsg->getHopInfo();
	std::list<LAddress::L3Type> routingPath;

	for (HopItems::reverse_iterator iter = hopItems.rbegin(); iter != hopItems.rend(); ++iter)
		routingPath.push_back(iter->addr);

	routingTable.insert(std::pair<LAddress::L3Type, std::list<LAddress::L3Type> >(routingMsg->getReceiver(), routingPath));

	displayRoutingTable();
}

void GPCR::onRoutingFail(RoutingMessage *routingMsg)
{
	EV << logName() << ": failed to find a routing path to destination " << routingMsg->getReceiver() << std::endl;
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

void GPCR::onPathBroken(RoutingMessage *routingMsg)
{
	EV << "GPCR::onPathBroken() unimplemented yet!\n";
}

GPCR::~GPCR()
{

}
