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

#ifndef __ROUTEREPAIRINTERFACE_H__
#define __ROUTEREPAIRINTERFACE_H__

#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"

#define INFINITE_HOPS    255

/**
 * @brief Interface class for route repair protocol design and any concrete subclass should private inherit it.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see AOMDV
 */
class RouteRepairInterface
{
protected:
	/** @name constructor, destructor. */
	///@{
	RouteRepairInterface() : callRoutingEvt(nullptr) {}
	virtual ~RouteRepairInterface() { routingPlanList.clear(); senderStates.clear(); receiverStates.clear(); }
	///@}

protected:
	/** @brief link layer notifies that communication link is broken when transmitting. */
	virtual void onLinkBroken(LAddress::L3Type neighbor, DataMessage *dataPkt) = 0;
	/** @brief called when local repair timer is timeout. */
	virtual void onLocalRepairFailure(LAddress::L3Type neighbor) = 0;

	/**
	 * @brief Searches for alternative routes before the communication is disrupted.
	 *
	 * If searched an alternative path, then update old one in the routing table with the new one and return true,
	 * else keep the old one unchanged and return false.
	 */
	// virtual bool seekAlternativePath(std::list<LAddress::L3Type>& existingPath, int destination) { return false; };

	/** @brief Notified that there is a reachable route to a destination. */
	virtual void onRouteReachable(LAddress::L3Type dest) = 0;
	/** @brief Notified that there is no reachable route to a destination. */
	virtual void onRouteUnreachable(LAddress::L3Type dest) = 0;
	/** @brief List all the routing path maintained in the routing table. */
	virtual void printRoutingTable() = 0;

	/** @brief call a routing request to a certain receiver determined by routingPlanList. */
	virtual void callRouting(LAddress::L3Type receiver, int contentSize) = 0;
	/** @brief initialize routingPlanList through configured xmlfile. */
	void initializeRoutingPlanList(cXMLElement *xmlConfig, LAddress::L3Type myIdentifier);
	/** @brief insert a neighbor's L2 address and L3 address pair. */
	bool insertL2L3Address(LAddress::L2Type addr2, LAddress::L3Type addr3);
	/** @brief given a neighbor's L3 address, return its L2 address. */
	LAddress::L2Type lookupL2Address(LAddress::L3Type addr);
	/** @brief given a neighbor's L2 address, return its L3 address. */
	LAddress::L3Type lookupL3Address(LAddress::L2Type addr);
	/** @brief push packet of a destination into send buffer (FIFO). */
	size_t pushBufferQueue(std::queue<DataMessage*>& rQueue, DataMessage *dataPkt);
	/** @brief discard all packets of a destination from send buffer. */
	void clearBufferQueue(std::queue<DataMessage*>& rQueue);

protected:
	class SendState
	{
	public:
		SendState() : totalBytes(0), curSeqno(0), curOffset(0), receiver(0), sendDataEvt(nullptr) {}
		SendState(int size, LAddress::L3Type re) : totalBytes(size), curSeqno(0), curOffset(0), receiver(re), sendDataEvt(nullptr) {}

		int totalBytes;
		int curSeqno;
		int curOffset;
		LAddress::L3Type receiver; ///< usage: sendDataEvt->setContextPointer(&receiver).
		cMessage *sendDataEvt;     ///< self message event used to periodically send data packet to the receiver.
	};

	class RecvState
	{
	public:
		struct PathState {
			int prevHighest;  ///< previous highest sequence number received before last feedback.
			int highestSeqno; ///< highest sequence number received during two feedback, used to calculate packet lost fraction.
			int pktsRecv;  ///< the number of data packers received from this path.
			double delay;  ///< average delay of data packets.
			double jitter; ///< sampled jitter of data packets.
		};

		/** @return whether this data packet is duplicated. */
		bool onRecv(int seqno);

		std::list<std::pair<int, int> > segments; ///< sequence number intervals of received data packets.
		std::list<std::pair<int, int> >::iterator itS, itD; ///< itS is used to traverse container segments, itD is used to operate.
		std::list<std::pair<LAddress::L3Type, PathState> > states; ///< states of each path, first argument is last hop.
	};

	int routingLengthBits; ///< the length of routing message measured in bits.
	int routingPriority;   ///< the priority of routing message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.
	size_t bufferQueueCap; ///< the capacity of data packets buffer queue.

	simtime_t pktTransmitDelay;  ///< time duration of sending a data packet from MAC layer to wireless media.
	simtime_t pktNetLayerDelay;  ///< time duration of sending a data packet from network layer to MAC layer.
	simtime_t pktApplLayerDelay; ///< time duration of sending a data packet from application layer to network layer.

	cMessage *callRoutingEvt; ///< self message event used to call routing request to certain destination determined by routingPlanList.
	std::list<std::pair<double, std::pair<LAddress::L3Type, int> > > routingPlanList; ///< routing plans of all vehicles configured by a xmlfile.
	std::list<std::pair<LAddress::L2Type, LAddress::L3Type> > linkTable; ///< store L2 and L3 address of neighbors for a long time.
	std::map<LAddress::L3Type, SendState> senderStates;   ///< a map from L3 address of the source to its send state.
	std::map<LAddress::L3Type, SendState>::iterator itSS; ///< an iterator used to traverse container senderStates.
	std::map<LAddress::L3Type, RecvState> receiverStates; ///< a map from L3 address of the source to its receive state.
	std::map<LAddress::L3Type, RecvState>::iterator itRS; ///< an iterator used to traverse container receiverStates.
	std::map<LAddress::L3Type, std::queue<DataMessage*> > rQueues; ///< "drop-front" queues used by the routing layer to buffer packets to which it does not have a route.
	std::map<LAddress::L3Type, std::queue<DataMessage*> >::iterator itRQ; ///< an iterator used to traverse container rQueues.
};

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
		}
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

			ASSERT( parameters.size() % 3 == 0 );

			EV << "routing plan list as follows:\n";

			for (size_t i = 0; i < parameters.size(); i += 3)
			{
				double simtime = 0.0;
				std::pair<LAddress::L3Type, int> infoPair;

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
					infoPair.first = atoi(value);

				name = parameters[i+2]->getAttribute("name");
				type = parameters[i+2]->getAttribute("type");
				value = parameters[i+2]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value");

				if (strcmp(name, "contentsize") == 0 && strcmp(type, "int") == 0)
					infoPair.second = atoi(value);

				EV << "    simtime: " << simtime << ", receiver: " << infoPair.first << ", content size: " << infoPair.second << std::endl;
				routingPlanList.push_back(std::pair<double, std::pair<LAddress::L3Type, int> >(simtime, infoPair));
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

size_t RouteRepairInterface::pushBufferQueue(std::queue<DataMessage*>& rQueue, DataMessage *dataPkt)
{
	size_t curSize = rQueue.size();
	if (curSize == bufferQueueCap)
	{
		delete rQueue.front();
		rQueue.pop();
		--curSize;
		++RoutingStatisticCollector::gPktsOverLost;
	}
	rQueue.push(dataPkt);
	return ++curSize;
}

void RouteRepairInterface::clearBufferQueue(std::queue<DataMessage*>& rQueue)
{
	RoutingStatisticCollector::gPktsLinkLost += static_cast<long>(rQueue.size());
	while (!rQueue.empty())
	{
		delete rQueue.front();
		rQueue.pop();
	}
}

#endif /* __ROUTEREPAIRINTERFACE_H__ */
