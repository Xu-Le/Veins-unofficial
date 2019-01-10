//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/routing/RouteRepairInterface.h"

int RouteRepairInterface::routingLengthBits = 0;
int RouteRepairInterface::routingPriority = 0;
int RouteRepairInterface::dataLengthBits = 0;
int RouteRepairInterface::dataPriority = 0;
#ifdef USE_RECEIVER_REPORT
int RouteRepairInterface::sendRRPktsThres = 0;
#endif

bool RouteRepairInterface::RecvState::onRecv(int seqno)
{
	itS = segments.begin();
	while (itS != segments.end() && itS->second < seqno)
		++itS;
	if (itS == segments.end())
	{
		segments.push_back(std::pair<int, int>(seqno, seqno+1));
		return false;
	}
	if (itS->second > seqno)
	{
		if (itS->first > seqno)
		{
			if (itS->first - 1 < seqno)
				segments.insert(itS, std::pair<int, int>(seqno, seqno+1));
			else
				--itS->first;
			return false;
		}
		else
			return true;
	}
	// if reach here, we have itS->second == seqno
	itD = itS;
	++itD;
	if (itD == segments.end() || itD->first - itS->second > 1)
		++itS->second;
	else
	{
		itS->second = itD->second;
		segments.erase(itD);
	}
	return false;
}

void RouteRepairInterface::initializeRoutingPlanList(cXMLElement *xmlConfig, LAddress::L3Type myIdentifier)
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

		if (atoi(name) == myIdentifier)
		{
			cXMLElementList parameters = routingPlan->getElementsByTagName("parameter");

			ASSERT( parameters.size() % 4 == 0 );

			EV << "routing plan list as follows:\n";

			for (size_t i = 0; i < parameters.size(); i += 4)
			{
				double simtime = 0.0;
				LAddress::L3Type receiver;
				int totalBytes = 0;
				double calcTime = 0.0;

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
					receiver = atol(value);

				name = parameters[i+2]->getAttribute("name");
				type = parameters[i+2]->getAttribute("type");
				value = parameters[i+2]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value");
				if (strcmp(name, "totalbytes") == 0 && strcmp(type, "int") == 0)
					totalBytes = atoi(value);

				name = parameters[i+3]->getAttribute("name");
				type = parameters[i+3]->getAttribute("type");
				value = parameters[i+3]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value");
				if (strcmp(name, "calctime") == 0 && strcmp(type, "double") == 0)
					calcTime = atof(value);

				EV << "    simtime: " << simtime << ", (" << receiver << ',' << totalBytes << ',' << calcTime << ")." << std::endl;
				routingPlanList.push_back(std::pair<double, PlanEntry>(simtime, PlanEntry(receiver, totalBytes, calcTime)));
			}

			break;
		}
	}
}

bool RouteRepairInterface::insertL2L3Address(LAddress::L2Type addr2, LAddress::L3Type addr3)
{
	std::list<std::pair<LAddress::L2Type, LAddress::L3Type> >::iterator iter = linkTable.begin();
	for (; iter != linkTable.end(); ++iter)
		if (iter->second == addr3)
			return false;
	linkTable.push_front(std::pair<LAddress::L2Type, LAddress::L3Type>(addr2, addr3));
	return true;
}

LAddress::L2Type RouteRepairInterface::lookupL2Address(LAddress::L3Type addr)
{
	std::list<std::pair<LAddress::L2Type, LAddress::L3Type> >::iterator iter = linkTable.begin();
	for (; iter != linkTable.end(); ++iter)
		if (iter->second == addr)
			return iter->first;
	return -1;
}

LAddress::L3Type RouteRepairInterface::lookupL3Address(LAddress::L2Type addr)
{
	std::list<std::pair<LAddress::L2Type, LAddress::L3Type> >::iterator iter = linkTable.begin();
	for (; iter != linkTable.end(); ++iter)
		if (iter->first == addr)
			return iter->second;
	return -1;
}

size_t RouteRepairInterface::pushBufferQueue(DataMessage *dataPkt)
{
	if (bQueueSize < bQueueCap)
	{
		bQueue.push_back(dataPkt);
		return ++bQueueSize;
	}
	else
	{
		delete dataPkt;
		++RoutingStatisticCollector::gPktsOverLost;
		return bQueueSize;
	}
}

bool RouteRepairInterface::findBufferQueue(LAddress::L3Type dest)
{
	for (std::list<DataMessage*>::iterator it = bQueue.begin(); it != bQueue.end(); ++it)
		if ((*it)->getDestination() == dest)
			return true;
	return false;
}

void RouteRepairInterface::clearBufferQueue()
{
	RoutingStatisticCollector::gPktsLinkLost += static_cast<long>(bQueueSize);
	while (!bQueue.empty())
	{
		delete bQueue.front();
		bQueue.pop_front();
	}
}
