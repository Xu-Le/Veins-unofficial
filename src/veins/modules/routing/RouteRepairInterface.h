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

#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"

#define USE_XML_CONFIG_FILE
// #define USE_L2_UNICAST_DATA
// #define USE_RECEIVER_REPORT
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
	class PlanEntry
	{
	public:
		PlanEntry(LAddress::L3Type r, int t, simtime_t c) : receiver(r), totalBytes(t), calcTime(c) {}

		LAddress::L3Type receiver;
		int totalBytes;
		simtime_t calcTime;
	};

	/** @name constructor, destructor. */
	///@{
	RouteRepairInterface() : callRoutingEvt(nullptr) {}
	virtual ~RouteRepairInterface() { routingPlanList.clear(); linkTable.clear(); }
	///@}

protected:
	/** @brief link layer notifies that communication link is broken when transmitting. */
	virtual void onLinkBroken(LAddress::L3Type neighbor, DataMessage *dataPkt) = 0;
	/** @brief called when local repair timer is timeout. */
	virtual void onLocalRepairFailure(LAddress::L3Type neighbor) = 0;

	/** @brief Notified that there is a reachable route to a destination. */
	virtual void onRouteReachable(LAddress::L3Type dest) = 0;
	/** @brief Notified that there is no reachable route to a destination. */
	virtual void onRouteUnreachable(LAddress::L3Type dest) = 0;
	/** @brief List all the routing path maintained in the routing table. */
	virtual void printRoutingTable() = 0;

	/** @brief call a routing request to a certain receiver determined by routingPlanList. */
	virtual void callRouting(const PlanEntry& entry) = 0;
#ifdef USE_XML_CONFIG_FILE
	/** @brief initialize routingPlanList through configured xmlfile. */
	void initializeRoutingPlanList(cXMLElement *xmlConfig, LAddress::L3Type myIdentifier);
#endif
	/** @brief insert a neighbor's L2 address and L3 address pair. */
	bool insertL2L3Address(LAddress::L2Type addr2, LAddress::L3Type addr3);
	/** @brief given a neighbor's L3 address, return its L2 address. */
	LAddress::L2Type lookupL2Address(LAddress::L3Type addr);
	/** @brief given a neighbor's L2 address, return its L3 address. */
	LAddress::L3Type lookupL3Address(LAddress::L2Type addr);

	/** @brief push a data packet into send buffer (FIFO) if buffer is not full, or this data packet is discarded. */
	bool pushBufferQueue(DataMessage *dataPkt);
	/** @brief check if there exists any packet for a specific destination in send buffer. */
	bool findBufferQueue(LAddress::L3Type dest);
	/** @brief discard all packets in send buffer. */
	void clearBufferQueue();

protected:
	class SendState
	{
	public:
		SendState() : totalBytes(0), curSeqno(0), curOffset(0), receiver(0), sendDataEvt(nullptr) {}
		SendState(int size, LAddress::L3Type re) : totalBytes(size), curSeqno(0), curOffset(0), receiver(re), sendDataEvt(nullptr) {}
#ifdef USE_RECEIVER_REPORT
		~SendState() { states.clear(); }
		struct PathState
		{
			PathState() : deliverRatio(0.0) { seqnos.reserve(64); }

			double deliverRatio;
			std::vector<int> seqnos;
		};
#endif

		int totalBytes;
		int curSeqno;
		int curOffset;
		LAddress::L3Type receiver; ///< usage: sendDataEvt->setContextPointer(&receiver).
		cMessage *sendDataEvt;     ///< self message event used to periodically send data packet to the receiver.
#ifdef USE_RECEIVER_REPORT
		std::map<LAddress::L3Type, SendState::PathState> states; ///< states of each path, first argument is last hop.
#endif
	};

	class RecvState
	{
	public:
#ifdef USE_RECEIVER_REPORT
		struct PathState
		{
			PathState() : prevHighest(0), highestSeqno(0), pktsRecv(0), jitter(), delay(), lastDelay() {}

			/** called when sending a receiver report. */
			void rotate() { prevHighest = highestSeqno; pktsRecv = 0; delay = SimTime::ZERO; }

			int prevHighest;  ///< previous highest sequence number received before last feedback.
			int highestSeqno; ///< highest sequence number received during two feedback, used to calculate packet lost fraction.
			int pktsRecv;     ///< the number of data packers received from this path.
			simtime_t jitter; ///< sampled jitter of data packets.
			simtime_t delay;  ///< average delay of data packets.
			simtime_t lastDelay; ///< delay of last data packet.
		};
#endif
		/** @return whether this data packet is duplicated. */
		bool onRecv(int seqno);

		std::list<std::pair<int, int> > segments; ///< sequence number intervals of received data packets.
		std::list<std::pair<int, int> >::iterator itS, itD; ///< itS is used to traverse container segments, itD is used to operate.
#ifdef USE_RECEIVER_REPORT
		std::map<LAddress::L3Type, RecvState::PathState> states; ///< states of each path, first argument is last hop.
#endif
	};

	size_t bQueueSize; ///< the current size of data packets buffer queue.
	size_t bQueueCap;  ///< the capacity of data packets buffer queue.

	simtime_t pktTransmitDelay;  ///< time duration of sending a data packet from MAC layer to wireless media.
	simtime_t pktNetLayerDelay;  ///< time duration of sending a data packet from network layer to MAC layer.
	simtime_t pktApplLayerDelay; ///< time duration of sending a data packet from application layer to network layer.

	cMessage *callRoutingEvt; ///< self message event used to call routing request to certain destination determined by routingPlanList.
	std::list<std::pair<double, PlanEntry> > routingPlanList; ///< routing plans of all vehicles configured by a xmlfile.
	std::list<std::pair<LAddress::L2Type, LAddress::L3Type> > linkTable; ///< store L2 and L3 address of neighbors for a long time.
	std::list<DataMessage*> rQueue; ///< FIFO queue used by the routing protocol to buffer packets when route is not up.
	std::list<DataMessage*>::iterator itRQ; ///< an iterator used to traverse container rQueue.
	std::list<DataMessage*> bQueue; ///< FIFO queue used by the network layer to buffer packets when EDCA queue of MAC layer is full.
	std::map<LAddress::L3Type, SendState> senderStates;   ///< a map from L3 address of the source to its send state.
	std::map<LAddress::L3Type, SendState>::iterator itSS; ///< an iterator used to traverse container senderStates.
	std::map<LAddress::L3Type, RecvState> receiverStates; ///< a map from L3 address of the source to its receive state.
	std::map<LAddress::L3Type, RecvState>::iterator itRS; ///< an iterator used to traverse container receiverStates.

	static int routingLengthBits; ///< the length of routing message measured in bits.
	static int routingPriority;   ///< the priority of routing message.
	static int dataHeaderBits;    ///< the length of data header measured in bits.
	static int dataLengthBits;    ///< the length of data message measured in bits.
	static int dataPriority;      ///< the priority of data message.
#ifdef USE_RECEIVER_REPORT
	static int sendRRPktsThres;   ///< how many data packets received will trigger a receiver report.
#endif
};

#endif /* __ROUTEREPAIRINTERFACE_H__ */
