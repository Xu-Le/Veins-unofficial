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

#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/routing/PAOMBR.h"

Define_Module(PAOMBR);

double PAOMBR::recvPowerThres_dBm = -77.0;

void PAOMBR::initialize(int stage)
{
	BaseWaveApplLayer::initialize(stage);

	if (stage == 0)
	{
		seqno = 0;
		rreqID = 0;

		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();
#ifdef USE_RECEIVER_REPORT
		sendRRPktsThres = par("sendRRPktsThres").longValue();
#endif
		bQueueSize = 0;
		bQueueCap = par("bufferQueueCap").longValue();

		pktTransmitDelay = SimTime(dataLengthBits/6, SIMTIME_US); // assume link layer rate is 6Mb/s
		pktNetLayerDelay = SimTime(par("pktNetLayerDelay").longValue(), SIMTIME_US);
		pktApplLayerDelay = SimTime(par("pktApplLayerDelay").longValue(), SIMTIME_US);

		recvPowerThres_dBm = par("recvPowerThres").doubleValue();

		callRoutings = par("callRoutings").boolValue();
		if (callRoutings)
		{
			initializeRoutingPlanList(par("routingPlan").xmlValue(), myAddr);
		}

		if (sendBeacons)
		{
			if (!routingPlanList.empty())
			{
				callRoutingEvt = new cMessage("call routing evt", WaveApplMsgKinds::CALL_ROUTING_EVT);
				scheduleAt(routingPlanList.front().first, callRoutingEvt);
			}
			sendBufDataEvt = new cMessage("send buf data evt", PAOMBRMsgKinds::SEND_BUF_DATA_EVT);
			sendBufDataEvt->setSchedulingPriority(1);
			purgeRoutingTableEvt = new cMessage("purge routing table evt", PAOMBRMsgKinds::PURGE_ROUTING_TABLE_EVT);
			purgeBroadcastCacheEvt = new cMessage("purge broadcast cache evt", PAOMBRMsgKinds::PURGE_BROADCAST_CACHE_EVT);
			scheduleAt(simTime() + dblrand()*PURGE_ROUTE_PERIOD, purgeRoutingTableEvt);
			scheduleAt(simTime() + dblrand()*PURGE_BCAST_ID_PERIOD, purgeBroadcastCacheEvt);
		}
		else
		{
			sendBufDataEvt = nullptr;
			purgeRoutingTableEvt = nullptr;
			purgeBroadcastCacheEvt = nullptr;
		}
	}
}

void PAOMBR::finish()
{
	// clear containers
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
		delete itRT->second;
	routingTable.clear();
	broadcastCache.clear();
	for (itSS = senderStates.begin(); itSS != senderStates.end(); ++itSS)
		cancelAndDelete(itSS->second.sendDataEvt);
	senderStates.clear();
	receiverStates.clear();
	clearBufferQueue();

	cancelAndDelete(callRoutingEvt);
	cancelAndDelete(sendBufDataEvt);
	cancelAndDelete(purgeRoutingTableEvt);
	cancelAndDelete(purgeBroadcastCacheEvt);

	BaseWaveApplLayer::finish();
}

void PAOMBR::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case PAOMBRMsgKinds::PURGE_ROUTING_TABLE_EVT:
	{
		purgeRoutingTable();
		scheduleAt(simTime() + PURGE_ROUTE_PERIOD, purgeRoutingTableEvt);
		break;
	}
	case PAOMBRMsgKinds::PURGE_BROADCAST_CACHE_EVT:
	{
		purgeBroadcastCache();
		scheduleAt(simTime() + PURGE_BCAST_ID_PERIOD, purgeBroadcastCacheEvt);
		break;
	}
	case PAOMBRMsgKinds::RREP_TIMEOUT_EVT:
	{
		LAddress::L3Type *brokenNb = static_cast<LAddress::L3Type*>(msg->getContextPointer());
		if (*brokenNb != -1) // local repair failure
			onLocalRepairFailure(*brokenNb);
		else
		{
			WaitForRREPMessage *waitRREPMsg = dynamic_cast<WaitForRREPMessage*>(msg);
			sendRREQ(waitRREPMsg->getDestAddr());
		}
		break;
	}
#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
	case PAOMBRMsgKinds::PLRR_TIMEOUT_EVT:
	{
		LAddress::L3Type *breakingNb = static_cast<LAddress::L3Type*>(msg->getContextPointer());
		// For each routing entry
		for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
		{
			PaombrRtEntry *rt = itRT->second;
			if (rt->flags == RTF_IN_PLRR && rt->pathLookup(*breakingNb) != nullptr && rt->connected && rt->pathList.size() == 1)
				rt->flags = RTF_UP;
		}
		break;
	}
	case PAOMBRMsgKinds::LET_TIMEOUT_EVT:
	{
		LAddress::L3Type *breakingNb = static_cast<LAddress::L3Type*>(msg->getContextPointer());
		sendRREQp(*breakingNb);
		break;
	}
#endif
	case PAOMBRMsgKinds::SEND_DATA_EVT:
	{
		simtime_t moment = pktApplLayerDelay;
		if (bQueueSize == bQueueCap)
		{
			EV << "buffer queue is full, wait a moment: " << moment << std::endl;
			scheduleAt(simTime() + moment, msg);
			break;
		}

		LAddress::L3Type *dest = static_cast<LAddress::L3Type*>(msg->getContextPointer());

		PaombrRtEntry *rt = lookupRoutingEntry(*dest);
		ASSERT(rt != nullptr);

		SendState &sendState = senderStates[*dest]; // must exists
		int bytesNum = dataLengthBits / 8;
		if (bytesNum > sendState.totalBytes - sendState.curOffset)
			bytesNum = sendState.totalBytes - sendState.curOffset;

		EV << "packet seqno: " << sendState.curSeqno << ", current offset: " << sendState.curOffset << "\n";
		DataMessage *dataPkt = new DataMessage("data");
		prepareWSM(dataPkt, 8*bytesNum, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority);
		dataPkt->setSource(myAddr);
		dataPkt->setDestination(*dest);
		dataPkt->setSequence(sendState.curSeqno++);
		dataPkt->setBytesNum(bytesNum);
		sendState.curOffset += bytesNum;

		++rt->bufPktsNum;
		if (rt->flags == RTF_UP)
		{
			ASSERT(!rt->pathList.empty());

			pushBufferQueue(dataPkt); // will never overflow
			EV << "RTF_UP, prepare send it, queue size: " << bQueueSize << std::endl;

			if (!sendBufDataEvt->isScheduled())
				scheduleAt(simTime() + pktNetLayerDelay, sendBufDataEvt);
			// we have sent all buffered packets and application layer has no data to send
			if (sendState.curOffset == sendState.totalBytes && rt->bufPktsNum == 1)
			{
				EV << "I have sent all data, purge send state." << std::endl;
				cancelAndDelete(sendState.sendDataEvt);
				senderStates.erase(*dest);
				if (!findBufferQueue(*dest))
					rt->connected = false;
				break;
			}
		}
		else if (rt->flags == RTF_DOWN)
		{
			EV << "RTF_DOWN, buffer and send RREQ.\n";
			rQueue.push_back(dataPkt);
			rt->flags = RTF_IN_REPAIR;
			sendRREQ(*dest);
		}
		else // rt->flags == RTF_IN_REPAIR
		{
			EV << "RTF_IN_REPAIR, just buffer." << std::endl;
			rQueue.push_back(dataPkt);
		}

		if (sendState.curOffset < sendState.totalBytes)
			scheduleAt(simTime() + moment, msg);
		break;
	}
	case PAOMBRMsgKinds::SEND_BUF_DATA_EVT:
	{
		simtime_t moment = pktNetLayerDelay;
		if (bQueue.empty())
		{
			EV << "buffer queue is empty." << std::endl;
			break;
		}
		t_channel chan = dataOnSch ? t_channel::type_SCH : t_channel::type_CCH;
		if (myMac->getEDCAQueueRoom(chan, dataPriority) == 0)
		{
			moment += pktTransmitDelay;
			moment *= myMac->getEDCAQueueRoom(chan, dataPriority, true);
			EV << "EDCA queue is full, wait a moment: " << moment << std::endl;
			scheduleAt(simTime() + moment, msg);
			break;
		}

		DataMessage *dataPkt = bQueue.front();
		bQueue.pop_front();
		--bQueueSize;

		PaombrRtEntry *rt = lookupRoutingEntry(dataPkt->getDestination());
		ASSERT(rt != nullptr);
		if (rt->flags != RTF_UP)
		{
			EV << "RTF_DOWN || RTF_IN_REPAIR, push it into rqueue." << std::endl;
			rQueue.push_back(dataPkt);
			scheduleAt(simTime() + moment, msg);
			break;
		}

		SendState *pss = nullptr;
#ifdef USE_RECEIVER_REPORT
		if (dataPkt->getSource() == myAddr)
			pss = &senderStates[dataPkt->getDestination()]; // must exists
#endif
		PaombrPath *path = rt->pathSelect(pss);
		dataPkt->setSenderAddress(myAddr);
#ifdef USE_L2_UNICAST_DATA
		dataPkt->setRecipientAddress(lookupL2Address(path->nextHop));
#else
		dataPkt->setReceiverAddress(path->nextHop);
#endif

		// Note that this should include not only the bits in the routing control packets,
		// but also the bits in the header of the data packets.	In other words, anything that is
		// not data is control overhead, and should be counted in the control portion of the algorithm.
		RoutingStatisticCollector::gDataBitsTransmitted += dataPkt->getBitLength() - headerLength;
		RoutingStatisticCollector::gCtrlBitsTransmitted += headerLength;
		RoutingStatisticCollector::gDataPktsTransmitted++;
		if (dataPkt->getSource() == myAddr)
		{
			RoutingStatisticCollector::gDataBitsSent += dataPkt->getBitLength() - headerLength;
			RoutingStatisticCollector::gDataPktsSent++;
#ifdef USE_RECEIVER_REPORT
			SendState &sendState = senderStates[dataPkt->getDestination()]; // must exists
			SendState::PathState &pathState = sendState.states[path->lastHop];
			pathState.seqnos.push_back(dataPkt->getSequence());
			EV << path->lastHop << "\n";
#endif
		}
		sendDelayedDown(dataPkt, moment);
		// trick: simulate packet lost without MAC ACK
		Coord &nextHopPos = MobilityObserver::Instance()->globalPosition[path->nextHop];
		if (curPosition.distance(nextHopPos) > transmissionRadius)
		{
			onDataLost(dataPkt->dup());
		}

		if (--rt->bufPktsNum == 0)
		{
			if (dataPkt->getSource() != myAddr)
				EV << "I have forwarded all data." << std::endl;
			else
			{
				SendState &sendState = senderStates[dataPkt->getDestination()]; // must exists
				if (sendState.curOffset < sendState.totalBytes)
					EV << "application layer still has data to send.\n";
				else // we have sent all buffered packets and application layer has no data to send
				{
					EV << "I have sent all data, purge send state." << std::endl;
					cancelAndDelete(sendState.sendDataEvt);
					senderStates.erase(dataPkt->getDestination());
					if (!findBufferQueue(dataPkt->getDestination()))
						rt->connected = false;
				}
			}
		}

		scheduleAt(simTime() + moment, msg);
		break;
	}
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

void PAOMBR::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "routing") == 0)
		DYNAMIC_CAST_CMESSAGE(Routing, routing)
	else if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)
	else if (strcmp(msg->getName(), "uavBeacon") == 0)
		DYNAMIC_CAST_CMESSAGE(Beacon, beacon)

	BaseWaveApplLayer::handleLowerMsg(msg);
}

void PAOMBR::handleLowerControl(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		onDataLost(dynamic_cast<DataMessage*>(msg));
}

//////////////////////////////    RX handlers    //////////////////////////////
void PAOMBR::onBeacon(BeaconMessage *beaconMsg)
{
	EV << "node[" << myAddr << "]: onBeacon!\n";

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias
	NeighborInfo *neighborInfo = nullptr;
	if ((itN = neighbors.find(sender)) != neighbors.end()) // update old record
	{
		EV << "    sender [" << sender << "] is an old neighbor, update its info.\n";
		neighborInfo = itN->second;
		neighborInfo->pos = beaconMsg->getSenderPos();
		neighborInfo->speed = beaconMsg->getSenderSpeed();
		neighborInfo->macAddr = beaconMsg->getRecipientAddress();
		neighborInfo->receivedAt = simTime();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new neighbor, insert its info.\n";
#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
		neighborInfo = new PaombrNeighborInfo(beaconMsg->getSenderPos(), beaconMsg->getSenderSpeed(), beaconMsg->getRecipientAddress(), simTime(), sender);
#else
		neighborInfo = new NeighborInfo(beaconMsg->getSenderPos(), beaconMsg->getSenderSpeed(), beaconMsg->getRecipientAddress(), simTime());
#endif
		neighbors.insert(std::pair<LAddress::L3Type, NeighborInfo*>(sender, neighborInfo));
	}

#if ROUTING_DEBUG_LOG
	EV << "    senderPos: " << beaconMsg->getSenderPos() << ", senderSpeed: " << beaconMsg->getSenderSpeed() << "\n";
	EV << "display all neighbors' information:\n";
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		EV << "neighbor[" << itN->first << "]:  pos:" << itN->second->pos << ", speed:" << itN->second->speed << ", MAC: " << itN->second->macAddr << "\n";
#endif

	// Whenever a node receives a Hello message from a neighbor, the node SHOULD make sure
	// that it has an active route to the neighbor, and	create one if necessary. If a route
	// already exists, then the	Lifetime for the route should be increased, if necessary,
	// to be at	least ALLOWED_HELLO_LOSS * HELLO_INTERVAL.
	insertL2L3Address(beaconMsg->getRecipientAddress(), sender);
	PaombrRtEntry *rt = lookupRoutingEntry(sender);
	if (rt == nullptr)
	{
		EV << "create an entry for the neighbor " << sender << "\n";
		rt = insertRoutingEntry(sender);
		rt->flags = RTF_UP;
		rt->advertisedHops = 1;
		rt->lastHopCount = 1;
		rt->dest = sender;
	}

	PaombrPath *path = rt->pathLookup(sender);
	if (path != nullptr)
		path->expireAt = simTime() + neighborElapsed;
	else
		path = rt->pathInsert(sender, myAddr, 1, neighborElapsed, neighborElapsed);

	// predict link expiration time
	simtime_t LET = calcLET(neighborInfo->pos, neighborInfo->speed);
	EV << "LET: " << LET << "s.\n";

#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
	PaombrNeighborInfo *paombrNb = dynamic_cast<PaombrNeighborInfo*>(neighborInfo);

	// retrieve control information generated by physical layer
	PhyToMacControlInfo *phyInfo = dynamic_cast<PhyToMacControlInfo*>(beaconMsg->removeControlInfo());
	DeciderResult80211 *ctrlInfo = dynamic_cast<DeciderResult80211*>(phyInfo->getDeciderResult());
	EV << "SINR: " << ctrlInfo->getSnr() << ", recv power: " << ctrlInfo->getRecvPower_dBm() << "dBm." << std::endl;
	delete phyInfo;

	// determine whether to execute preemptive local route repair
	bool mayBreak = ctrlInfo->getRecvPower_dBm() < recvPowerThres_dBm && !paombrNb->recvPower_dBm.empty() && paombrNb->recvPower_dBm.back() < recvPowerThres_dBm;
	if (paombrNb->recvPower_dBm.size() == 2*MAX_RECV_POWER_POLYNOMIAL_DEGREE+1)
		paombrNb->recvPower_dBm.pop_front();
	paombrNb->recvPower_dBm.push_back(ctrlInfo->getRecvPower_dBm());
	if (!paombrNb->LETTimeoutEvt->isScheduled() && simTime() > paombrNb->PLRRSuppressUntil)
	{
		if (mayBreak && recvPowerCritical(paombrNb->recvPower_dBm))
			scheduleAt(simTime(), paombrNb->LETTimeoutEvt);
		else if (LET < beaconInterval + PLRR_DISCOVERY_TIME && LET < paombrNb->LET) // initial value of paombrNb->LET is 0
		{
			simtime_t LETTimeoutAt = LET > PLRR_DISCOVERY_TIME ? simTime() + LET - PLRR_DISCOVERY_TIME : simTime();
			EV << "LET timeout at: " << LETTimeoutAt << std::endl;
			scheduleAt(LETTimeoutAt, paombrNb->LETTimeoutEvt);
		}
	}
	paombrNb->LET = LET;
#endif
}

void PAOMBR::onRouting(RoutingMessage *msg)
{
	// EV << "node[" << myAddr << "]: onRouting!\n";

	uint8_t ttl = msg->getTTL();
	msg->setTTL(--ttl);

	EV << "TTL: " << (uint32_t)ttl << "\n";

	// mobility extension, only update old record, it is the duty of beacon messages to maintain neighborhood
	if ((itN = neighbors.find(msg->getSenderAddress())) != neighbors.end())
	{
		itN->second->pos = msg->getSenderPos();
		itN->second->speed = msg->getSenderSpeed();
	}

	switch (msg->getPacketType())
	{
	case AOMDVPacketType::RREQ:
	{
		DYNAMIC_CAST_CMESSAGE(RREQ, rreq)
		break;
	}
	case AOMDVPacketType::RREP:
	{
		DYNAMIC_CAST_CMESSAGE(RREP, rrep)
		break;
	}
#ifdef USE_DESTINATION_AGGREGATION
	case AOMDVPacketType::RREQp:
	{
		DYNAMIC_CAST_CMESSAGE(RREQp, rreqp)
		break;
	}
	case AOMDVPacketType::RREPp:
	{
		DYNAMIC_CAST_CMESSAGE(RREPp, rrepp)
		break;
	}
#endif
	case AOMDVPacketType::RERR:
	{
		DYNAMIC_CAST_CMESSAGE(RERR, rerr)
		break;
	}
	case AOMDVPacketType::RREPACK:
	{
		break;
	}
#ifdef USE_RECEIVER_REPORT
	case AOMDVPacketType::RR:
	{
		DYNAMIC_CAST_CMESSAGE(RR, rr)
		break;
	}
#endif
	case AOMDVPacketType::LR:
	{
		DYNAMIC_CAST_CMESSAGE(LR, lr)
		break;
	}
	default:
		throw cRuntimeError("receive a PAOMBR control packet with invalid packet type");
	}
}

void PAOMBR::onRREQ(RREQMessage *rq)
{
	EV << "node[" << myAddr << "]: onRREQ!\n";

	LAddress::L3Type ipsrc = rq->getSenderAddress(); // alias
	LAddress::L3Type rqsrc = rq->getOriginatorAddr(), rqdst = rq->getDestAddr(); // alias
	EV << "RREQ src: " << rqsrc << ", RREQ dest: " << rqdst << "\n";
	// Discard if:
	//      - I'm the source;
	//      - I'm the breaking neighbor and receives RREQ directly from source (PLRR);
	//      - I'm the previous hop of RREQ source;
	//      - Link expiration time with the sender is too short (PLRR);
	//      - I recently heard this RREQ.
	if (rqsrc == myAddr)
	{
		EV << "This is my own RREQ, discard it." << std::endl;
		return;
	}
	if (rq->getPreemptiveFlag() && rq->getBreakingNb() == myAddr && ipsrc == rqsrc)
	{
		EV << "I am the neighbor from whom RREQ src will disconnect and this is original RREQ, discard it." << std::endl;
		return;
	}
	// Check route entry for RREQ destination
	PaombrRtEntry *rt = lookupRoutingEntry(rqdst);
	if (rt != nullptr && rt->pathLookup(rqsrc) != nullptr)
	{
		EV << "I am the previous hop of RREQ src, discard it." << std::endl;
		return;
	}

	// predict link expiration time
	if ((itN = neighbors.find(ipsrc)) == neighbors.end())
	{
		EV << "Sender is not my neighbor, discard it." << std::endl;
		return;
	}
#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
	PaombrNeighborInfo *paombrNb = dynamic_cast<PaombrNeighborInfo*>(itN->second);
	paombrNb->LET = calcLET(paombrNb->pos, paombrNb->speed);
	EV << "LET: " << paombrNb->LET << "s, rq->LET: " << rq->getMinLET() << "s.\n";
	if (rq->getPreemptiveFlag() && paombrNb->LET < 2*PLRR_DISCOVERY_TIME)
	{
		EV << "Link expiration time too short, discard it." << std::endl;
		return;
	}
	if (rq->getMinLET() > paombrNb->LET)
		rq->setMinLET(paombrNb->LET);
#else
	simtime_t LET = calcLET(itN->second->pos, itN->second->speed);
	EV << "LET: " << LET << "s, rq->LET: " << rq->getMinLET() << "s.\n";
	if (rq->getMinLET() > LET)
		rq->setMinLET(LET);
#endif

	bool propagation = true;
	// If RREQ has already been received - drop it, else remember "RREQ id" <src IP, bcast ID>.
	PaombrBroadcastID *b = lookupBroadcastID(rqsrc, rq->getRreqId());
	if (b == nullptr)
	{
		EV << "receive fresh RREQ, cache its RREQ ID.\n";
		b = insertBroadcastID(rqsrc, rq->getRreqId());
	}
	else
	{
		EV << "receive duplicate RREQ, stop propagating it.\n";
		propagation = false;
	}

	// If I am a neighbor to the RREQ source, make myself first hop on path from source to dest.
	if (rq->getHopCount() == 0)
	{
		EV << "I am a neighbor to the RREQ source, i.e., the last hop on reverse path.\n";
		rq->setFirstHop(myAddr);
	}

	/*
	 * We are either going to forward the REQUEST or generate a REPLY.
	 * Before we do anything, we make sure that the REVERSE route is in the route table.
	 */
	PaombrRtEntry *rt0 = lookupRoutingEntry(rqsrc);
	if (rt0 == nullptr) // create an entry for the reverse route.
	{
		EV << "create an entry for the reverse route.\n";
		rt0 = insertRoutingEntry(rqsrc);
	}
	// store the source for whom I provide help
	rt0->revPathInsert(rqdst, rq->getFirstHop());

	/*
	 * 3.1.1.1. Sufficient Conditions
	 * 1. Sequence number rule: Maintain routes only for the highest known destination sequence number.
	 *    For each destination, we restrict that multiple paths maintained by a node have the same
	 *    destination sequence number. With this restriction, we can maintain a loop freedom invariant
	 *    similar to AODV. Once a route advertisement containing a higher destination sequence number is
	 *    received, all routes corresponding to the older sequence number are discarded.
	 * 2. For the same destination sequence number,
	 *    (a) Route advertisement rule: Never advertise a route shorter than one already advertised.
	 *    (b) Route acceptance rule: Never accept a route longer than one already advertised.
	 * To maintain multiple paths for the same sequence number, PAOMBR uses the notion of an 'advertised
	 * hop count.' Every node maintains a variable called advertised hop count for each destination.
	 * This variable is set to the length of the 'longest' available path for the destination at the time
	 * of first advertisement for a particular destination sequence number. The advertised hop count
	 * remains unchanged until the sequence number changes. Advertising the longest path length permits
	 * more number of alternate paths to be maintained.
	 */
	PaombrPath *reversePath = nullptr;
	uint32_t rqHopCount = rq->getHopCount() + 1;
	// Create/update reverse path (i.e. path back to RREQ source)
	// If RREQ contains more recent seq number than route table entry - update route entry to source.
	if (rt0->seqno < rq->getOriginatorSeqno())
	{
		EV << "Case I: get fresher route (" << rt0->seqno << " < " << rq->getOriginatorSeqno() << ").\n";

		rt0->seqno = rq->getOriginatorSeqno();
		rt0->advertisedHops = INFINITE_HOPS;
		rt0->pathList.clear(); // Delete all previous paths to RREQ source
		rt0->flags = RTF_UP;
		// Insert new path for route entry to source of RREQ.
		// (src addr, hop count + 1, lifetime, last hop (first hop for RREQ))
		reversePath = rt0->pathInsert(ipsrc, rq->getFirstHop(), rq->getHopCount()+1, REVERSE_ROUTE_LIFE, rq->getMinLET());
		rt0->lastHopCount = rq->getHopCount() + 1;
		EV << "new reverse path: " << reversePath->info() << ", last hop count: " << (uint32_t)rt0->lastHopCount << std::endl;
	}
	// If a new path with smaller hop count is received
	// (same seqno, better hop count) - try to insert new path in route table.
	else if (rt0->seqno == rq->getOriginatorSeqno() && rt0->advertisedHops > rq->getHopCount())
	{
		ASSERT(rt0->flags == RTF_UP); // Make sure path is up

		EV << "Case II: get useful route (" << (uint32_t)rt0->advertisedHops << " >= " << rqHopCount << ").\n";

		// If path already exists - adjust the lifetime of the path.
		if ((reversePath = rt0->disjointPathLookup(ipsrc, rq->getFirstHop())) != nullptr)
		{
			ASSERT(reversePath->hopCount == rq->getHopCount() + 1);
			reversePath->expireAt = simTime() + REVERSE_ROUTE_LIFE;
			reversePath->PET = simTime() + rq->getMinLET();
			if (rt0->expireAt < reversePath->expireAt)
				rt0->expireAt = reversePath->expireAt;
			EV << "reverse path already exists, update expiration time to " << reversePath->expireAt << std::endl;
		}
		// Got a new alternate disjoint reverse path - so insert it.
		// I.e. no path exists which has RREQ source as next hop and no
		// path with RREQ first hop as last hop exists for this route entry.
		// Simply stated: no path with the same last hop exists already.
		else if (rt0->disjointPathExists(ipsrc, rq->getFirstHop()) && rt0->pathList.size() < PAOMBR_MAX_PATHS)
		{
			// Only insert new path if not too many paths exists for this destination
			reversePath = rt0->pathInsert(ipsrc, rq->getFirstHop(), rq->getHopCount()+1, REVERSE_ROUTE_LIFE, rq->getMinLET());
			rt0->lastHopCount = rt0->pathGetMaxHopCount();
			EV << "new reverse path: " << reversePath->info() << ", last hop count: " << (uint32_t)rt0->lastHopCount << std::endl;
		}
	}
	// Older seqno (or same seqno with higher hopcount), i.e. I have a more recent route entry - so drop packet.
	else
	{
		if (rt0->seqno > rq->getOriginatorSeqno())
			EV << "get stale route (" << rt0->seqno << " > " << rq->getOriginatorSeqno() << ")." << std::endl;
		else if (rt0->advertisedHops < rq->getHopCount())
			EV << "get useless route (" << (uint32_t)rt0->advertisedHops << " < " << rqHopCount << ")." << std::endl;
		return;
	}
	// End for putting reverse route in routing table

	// Reset the soft state
	rt0->reqTimeout = SimTime::ZERO;
	rt0->reqCount = 0;
	rt0->reqLastTTL = 0;

	// Find out whether any buffered packet can benefit from the reverse route.
	EV << "reverse path becomes RTF_UP, transfer corresponding packets from rQueue to bQueue.\n";
	transferBufPackets(rqsrc);

	// I am the intended receiver of the RREQ - so send a RREP
	if (rqdst == myAddr)
	{
		// If the generating node is the destination itself, it MUST increment its own sequence number
		// by one if the sequence number in the RREQ packet is equal to that incremented value.
		// Otherwise, the destination does not change its sequence number before generating the
		// RREP message. The destination node places its (perhaps newly incremented) sequence number
		// into the Destination Sequence Number field of the RREP, and enters the value zero in the
		// Hop Count field of the RREP.
		// The destination node copies the value MY_ROUTE_TIMEOUT into the Lifetime field of the RREP.
		// Each node MAY reconfigure its value for MY_ROUTE_TIMEOUT, within mild constraints.
		if (seqno <= rq->getDestSeqno())
			seqno = rq->getDestSeqno() + 1;
		// Make sure seq number is even
		if (seqno & 1)
			++seqno;
		EV << "I am RREQ dest, update seqno to " << seqno << std::endl;
		sendRREP(rq, 0, seqno, MY_ROUTE_TIMEOUT, ipsrc);
	}
	// I have a fresh route entry for RREQ destination - so send RREP
	else if (rt != nullptr && rt->flags == RTF_UP && rt->seqno >= rq->getDestSeqno())
	{
		ASSERT(rt->seqno % 2 == 0); // is the seqno even?

		// If the node generating the RREP is not the destination node, but instead is an intermediate
		// hop along the path from the originator to the destination, it copies its known sequence
		// number for the destination into the Destination Sequence Number field in the RREP message.
		// The intermediate node places its distance in hops from the destination (indicated by the
		// hop count in the routing table) Count field in the RREP. The Lifetime field of the RREP
		// is calculated by	subtracting the current time from the expiration time in its route table entry.
		bool incurJoint = false;
		if (rq->getRepairFlag())
		{
			EV << "RREQ used for local repair, check whether reply will incur joint path.\n";
			uint32_t srcCount = rq->getSrcCount();
			for (uint32_t k = 0; k < srcCount; ++k)
			{
				LAddress::L3Type lh = rt->revPathLookup(rq->getSrcs(k));
				if (lh != -1 && lh != rq->getLastHop()) // concat original downstream nodes is allowed, which surely does not incur joint
				{
					incurJoint = true;
					break;
				}
			}
		}
		if (reversePath != nullptr && !incurJoint)
		{
			EV << "I have a fresh route to RREQ dest, b->count: " << b->count << " (" << rt->seqno << " >= " << rq->getDestSeqno() << ')' << std::endl;

			if (b->count == 0)
			{
				b->count = 1;

				// route advertisement
				if (rt->advertisedHops == INFINITE_HOPS)
					rt->advertisedHops = rt->pathGetMaxHopCount();

				PaombrPath *forwardPath = rt->pathSelect();
				rt->error = true;
				rt->connected = true;
				sendRREP(rq, rt->advertisedHops, rt->seqno, forwardPath->expireAt-simTime(), forwardPath->lastHop);
			}
		}
		else if (reversePath != nullptr)
		{
			EV << "Reject to reply because this will incur joint path." << std::endl;
		}
	}
	// RREQ not intended for me and I don't have a fresh enough entry for RREQ dest - so forward the RREQ
	else
	{
#ifdef USE_IRRESPONSIBLE_REBROADCAST
		if (rq->getPreemptiveFlag() && propagation)
		{
			// calculate propagation probability according to breaking direction
			Coord referencePoint(paombrNb->pos + rq->getBreakingDir()*transmissionRadius);
			double distRatio = curPosition.distance(referencePoint) / (2*transmissionRadius);
			double probability = exp(-sqrt(rq->getNbNum())*distRatio);
			ASSERT(probability >= 0 && probability <= 1);
			if (dblrand() > probability)
				propagation = false;
			EV << "rebroadcast probability: " << probability << ", propagation: " << propagation << "\n";
		}
#endif

		if (propagation) // do not propagate a duplicate RREQ
		{
			// Maximum sequence number seen in route
			if (rt != nullptr && rq->getDestSeqno() < rt->seqno)
				rq->setDestSeqno(rt->seqno);

			// route advertisement
			if (rt0->advertisedHops == INFINITE_HOPS)
				rt0->advertisedHops = rt0->pathGetMaxHopCount();
			rq->setHopCount(rt0->advertisedHops);
			EV << "forward RREQ, dest seqno: " << rq->getDestSeqno() << ", hop count: " << (uint32_t)rq->getHopCount() << std::endl;
			forward(nullptr, rq->dup());
		}
	}
}

void PAOMBR::onRREP(RREPMessage *rp)
{
	EV << "node[" << myAddr << "]: onRREP!\n";

	LAddress::L3Type ipsrc = rp->getSenderAddress(), ipdst = rp->getIpDest(); // alias
	LAddress::L3Type rpdst = rp->getDestAddr(); // alias
	EV << "IP src: " << ipsrc << ", IP dest: " << ipdst << ", RREP dest: " << rpdst <<  "\n";
	// If I receive a RREP with myself as source - drop packet (should not occur).
	// rpdst is the source of the RREP, or rather the destination of the RREQ.
	if (rpdst == myAddr)
		return;

	// predict link expiration time
	if ((itN = neighbors.find(ipsrc)) == neighbors.end())
	{
		EV << "Sender is not my neighbor, discard it." << std::endl;
		return;
	}
#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
	PaombrNeighborInfo *paombrNb = dynamic_cast<PaombrNeighborInfo*>(itN->second);
	paombrNb->LET = calcLET(paombrNb->pos, paombrNb->speed);
	EV << "LET: " << paombrNb->LET << "s, rp->LET: " << rp->getMinLET() << "s.\n";
	if (paombrNb->LET < 2*PLRR_DISCOVERY_TIME)
	{
		EV << "Link expiration time too short, discard it." << std::endl;
		return;
	}
	if (rp->getMinLET() > paombrNb->LET)
		rp->setMinLET(paombrNb->LET);
#else
	simtime_t LET = calcLET(itN->second->pos, itN->second->speed);
	EV << "LET: " << LET << "s, rp->LET: " << rp->getMinLET() << "s.\n";
	if (rp->getMinLET() > LET)
		rp->setMinLET(LET);
#endif

	PaombrBroadcastID *b = lookupBroadcastID(ipdst, rp->getRreqId()); // Check for <RREQ src IP, bcast ID> pair
	if (rp->getPreemptiveFlag())
	{
		ASSERT(b != nullptr);
	}

	// Got a reply. So reset the "soft state" maintained for route requests in the request table.
	// We don't really have have a separate request table. It is just a part of the routing table itself.
	PaombrRtEntry *rt = lookupRoutingEntry(rpdst);
	// If I don't have a rt entry to this host... adding
	if (rt == nullptr)
	{
		EV << "create an entry for the forward route.\n";
		rt = insertRoutingEntry(rpdst);
	}
	rt->connected = true;

	if (ipdst == myAddr && rp->getPreemptiveFlag())
		rt->pathDelete(rp->getBreakingNb());

	PaombrPath *forwardPath = nullptr;
	uint32_t rpHopCount = rp->getHopCount() + 1;
	// If RREP contains more recent seqno for (RREQ) destination
	// - delete all old paths and add the new forward path to (RREQ) destination
	if (rt->seqno < rp->getDestSeqno())
	{
		EV << "Case I: get fresher route (" << rt->seqno << " < " << rp->getDestSeqno() << ").\n";

		rt->seqno = rp->getDestSeqno();
		rt->advertisedHops = INFINITE_HOPS;
		rt->pathList.clear();
		rt->flags = RTF_UP;
		// Insert forward path to RREQ destination.
		forwardPath = rt->pathInsert(ipsrc, rp->getFirstHop(), rp->getHopCount()+1, rp->getLifeTime(), rp->getMinLET());
		rt->lastHopCount = rp->getHopCount() + 1;
		EV << "new forward path: " << forwardPath->info() << ", last hop count: " << (uint32_t)rt->lastHopCount << std::endl;
	}
	// If the sequence number in the RREP is the same as for route entry but
	// with a smaller hop count - try to insert new forward path to (RREQ) dest.
	else if (rt->seqno == rp->getDestSeqno())
	{
		if (!rp->getPreemptiveFlag() || ipdst != myAddr)
		{
			if (rt->advertisedHops >= rpHopCount)
				EV << "Case II: get useful route (" << (uint32_t)rt->advertisedHops << " >= " << rpHopCount << ").\n";
			else
			{
				EV << "get useless route (" << (uint32_t)rt->advertisedHops << " < " << rpHopCount << ")." << std::endl;
				return;
			}
		}
		else
		{
			bool ignore = true;
			if (b->count == 0 && rt->advertisedHops+1 >= rp->getHopCount())
			{
				EV << "Case II: get first RREPp (" << (uint32_t)rt->advertisedHops << "+2 >= " << rpHopCount << ").\n";
				ignore = false;
			}
			else if (b->count > 0 && rt->advertisedHops >= rpHopCount)
			{
				EV << "Case III: get useful route (" << (uint32_t)rt->advertisedHops << " >= " << rpHopCount << ").\n";
				ignore = false;
			}
			if (ignore)
			{
				EV << "get useless route (" << (uint32_t)rt->advertisedHops << " < " << rpHopCount << ")." << std::endl;
				return;
			}
		}

		rt->flags = RTF_UP;
		// If the path already exists - increase path lifetime
		if ((forwardPath = rt->disjointPathLookup(ipsrc, rp->getFirstHop())) != nullptr)
		{
			ASSERT(forwardPath->hopCount == rp->getHopCount() + 1);
			forwardPath->expireAt = simTime() + rp->getLifeTime();
			forwardPath->PET = simTime() + rp->getMinLET();
			if (rt->expireAt < forwardPath->expireAt)
				rt->expireAt = forwardPath->expireAt;
			EV << "forward path already exists, update expiration time to " << forwardPath->expireAt << std::endl;
		}
		// If the path does not already exist, and there is room for it - we add the path
		else if (rt->disjointPathExists(ipsrc, rp->getFirstHop()) && rt->pathList.size() < PAOMBR_MAX_PATHS)
		{
			// Insert forward path to RREQ destination.
			forwardPath = rt->pathInsert(ipsrc, rp->getFirstHop(), rp->getHopCount()+1, rp->getLifeTime(), rp->getMinLET());
			rt->lastHopCount = rt->pathGetMaxHopCount();
			EV << "new forward path: " << forwardPath->info() << ", last hop count: " << (uint32_t)rt->lastHopCount << std::endl;
		}
		// Path did not exist nor could it be added - just drop packet.
		else
			return;
	}
	// The received RREP did not contain more recent information than route table - so drop packet
	else
	{
		EV << "get stale route (" << rt->seqno << " > " << rp->getDestSeqno() << ")." << std::endl;
		return;
	}

	// Reset the soft state
	rt->reqTimeout = SimTime::ZERO;
	rt->reqCount = 0;
	rt->reqLastTTL = 0;

	// Find out whether any buffered packet can benefit from the forward route.
	EV << "forward path becomes RTF_UP, transfer corresponding packets from rQueue to bQueue.\n";
	transferBufPackets(rpdst);

	// If I am the intended recipient of the RREP, nothing more needs to be done - so drop packet.
	if (ipdst == myAddr)
	{
		EV << "I am the RREP dest, cancel RREP timeout event.\n";

		if (rp->getPreemptiveFlag())
		{
			if (b->count == 0) // first RREP received
				sendLR(rt, rp);

			rt->advertisedHops = rt->pathGetMaxHopCount();
			b->count = 1;
			paombrNb = dynamic_cast<PaombrNeighborInfo*>(neighbors[rp->getBreakingNb()]);
			RoutingStatisticCollector::gRouteAcquisitionTime += simTime() - paombrNb->PLRRTimeoutEvt->getSendingTime();
			cancelEvent(paombrNb->PLRRTimeoutEvt);
		}
		else
		{
			if (rp->getRepairFlag() && rt->pathList.size() == 1)
				sendLR(rt, rp);
			RoutingStatisticCollector::gRouteAcquisitionTime += simTime() - rt->rrepTimeoutEvt->getSendingTime();
			cancelEvent(rt->rrepTimeoutEvt);
			onRouteReachable(rpdst);
		}
#if ROUTING_DEBUG_LOG
		printRoutingTable();
#endif
		return;
	}

	// If I am not the intended recipient of the RREP
	// - check route table for a path to the RREP dest (i.e. the RREQ source).
	PaombrRtEntry *rt0 = lookupRoutingEntry(ipdst);
	if (rt0 == nullptr || rt0->flags != RTF_UP || b == nullptr || b->count > 0)
		return;

	rt0->connected = true;
	b->count = 1;
	PaombrPath *reversePath = rt0->pathSelect();
	reversePath->expireAt = simTime() + ACTIVE_ROUTE_TIMEOUT;
	EV << "I have an active route to RREQ src, reverse path: " << reversePath->info() << "\n";

	// route advertisement
	if (rt->advertisedHops == INFINITE_HOPS)
		rt->advertisedHops = rt->pathGetMaxHopCount();
	rp->setHopCount(rt->advertisedHops);
	rt->error = true;
	// store the source for whom I provide help
	rt->revPathInsert(ipdst, rp->getFirstHop());

	EV << "forward RREP, last hop: " << rp->getFirstHop() << ", hop count: " << (uint32_t)rp->getHopCount() << std::endl;
	forward(rt0, rp->dup(), rpdst);

#if ROUTING_DEBUG_LOG
	printRoutingTable();
#endif
}

void PAOMBR::onRERR(RERRMessage *re)
{
	EV << "node[" << myAddr << "]: onRERR!\n";

	LAddress::L3Type ipsrc = re->getSenderAddress(); // alias
	RERRMessage *nre = new RERRMessage("routing");
	uint8_t destCount = 0;
	// For each unreachable destination
	for (uint8_t i = 0; i < re->getDestCount(); ++i)
	{
		UnreachableNode &unreach = re->getUnreachableNodes(i);
		PaombrRtEntry *rt = lookupRoutingEntry(unreach.addr);
		// If route entry exists, route is up, a path to the unreachable destination exists
		// through the neighbor from which RERR was received, and my sequence number is not
		// more recent - delete path and add it to the RERR message I will send.
		if (rt != nullptr && rt->flags == RTF_UP && rt->pathLookup(ipsrc) != nullptr && rt->seqno <= unreach.seqno)
		{
			ASSERT(rt->seqno % 2 == 0);

			EV << "delete the path whose next hop is " << ipsrc << " (" << rt->seqno << " <= " << unreach.seqno << ").\n";
			rt->pathDelete(ipsrc);
			if (rt->highestSeqnoHeard < unreach.seqno)
				rt->highestSeqnoHeard = unreach.seqno;
			if (rt->pathList.empty())
			{
				rt->seqno = rt->highestSeqnoHeard;
				downRoutingEntry(rt);
				if (rt->error)
				{
					unreach.seqno = rt->seqno;
					nre->setUnreachableNodesArraySize(++destCount);
					nre->setUnreachableNodes(destCount-1, unreach);
					rt->error = false;
				}
			}
			else
			{
				std::string paths = rt->pathPrint();
				EV << paths.c_str() << std::endl;
			}
		}
	}
	if (destCount > 0)
	{
		nre->setDestCount(destCount);
		sendRERR(nre);
	}
	else
		delete nre;
}

#ifdef USE_DESTINATION_AGGREGATION
void PAOMBR::onRREQp(RREQpMessage *rq)
{
	EV << "node[" << myAddr << "]: onRREQp!\n";

	LAddress::L3Type ipsrc = rq->getSenderAddress(); // alias
	LAddress::L3Type rqsrc = rq->getOriginatorAddr(); // alias
	EV << "RREQp src: " << rqsrc << "\n";
	// Discard if:
	//      - I'm the source;
	//      - I'm the breaking neighbor and receives RREQp directly from source;
	//      - I'm the previous hop of RREQp source;
	//      - Link expiration time with the sender is too short;
	//      - I recently heard this RREQp.
	if (rqsrc == myAddr)
	{
		EV << "This is my own RREQp, discard it." << std::endl;
		return;
	}
	if (rq->getBreakingNb() == myAddr && ipsrc == rqsrc)
	{
		EV << "I am the neighbor from whom RREQp src will disconnect and this is original RREQp, discard it." << std::endl;
		return;
	}
	uint32_t destCount = rq->getDestCount();
	for (uint32_t k = 0; k < destCount; ++k)
	{
		PaombrRtEntry *rt = lookupRoutingEntry(rq->getRepairingNodes(k).addr);
		if (rt != nullptr && rt->pathLookup(rqsrc) != nullptr)
		{
			EV << "I am the previous hop of RREQp src, discard it." << std::endl;
			return;
		}
	}

	// predict link expiration time
	if ((itN = neighbors.find(ipsrc)) == neighbors.end())
	{
		EV << "Sender is not my neighbor, discard it." << std::endl;
		return;
	}
	PaombrNeighborInfo *paombrNb = dynamic_cast<PaombrNeighborInfo*>(itN->second);
	paombrNb->LET = calcLET(paombrNb->pos, paombrNb->speed);
	EV << "LET: " << paombrNb->LET << "s, rq->LET: " << rq->getMinLET() << "s.\n";
	if (paombrNb->LET < 2*PLRR_DISCOVERY_TIME)
	{
		EV << "Link expiration time too short, discard it." << std::endl;
		return;
	}
	if (rq->getMinLET() > paombrNb->LET)
		rq->setMinLET(paombrNb->LET);

	bool propagation = true;
	// If RREQ has already been received - drop it, else remember "RREQ id" <src IP, bcast ID>.
	PaombrBroadcastID *b = lookupBroadcastID(rqsrc, rq->getRreqId());
	if (b == nullptr)
	{
		EV << "receive fresh RREQp, cache its RREQ ID.\n";
		b = insertBroadcastID(rqsrc, rq->getRreqId());
	}
	else
	{
		EV << "receive duplicate RREQp, stop propagating it.\n";
		propagation = false;
	}

	// If I am a neighbor to the RREQ source, make myself first hop on path from source to dest.
	if (rq->getHopCount() == 0)
	{
		EV << "I am a neighbor to the RREQp source, i.e., the last hop on reverse path.\n";
		rq->setFirstHop(myAddr);
	}

	/*
	 * We are either going to forward the REQUEST or generate a REPLY.
	 * Before we do anything, we make sure that the REVERSE route is in the route table.
	 */
	PaombrRtEntry *rt0 = lookupRoutingEntry(rqsrc);
	if (rt0 == nullptr) // create an entry for the reverse route.
	{
		EV << "create an entry for the reverse route.\n";
		rt0 = insertRoutingEntry(rqsrc);
	}

	PaombrPath *reversePath = nullptr;
	uint32_t rqHopCount = rq->getHopCount() + 1;
	// Create/update reverse path (i.e. path back to RREQ source)
	// If RREQ contains more recent seq number than route table entry - update route entry to source.
	if (rt0->seqno < rq->getOriginatorSeqno())
	{
		EV << "Case I: get fresher route (" << rt0->seqno << " < " << rq->getOriginatorSeqno() << ").\n";

		rt0->seqno = rq->getOriginatorSeqno();
		rt0->advertisedHops = INFINITE_HOPS;
		rt0->pathList.clear(); // Delete all previous paths to RREQ source
		rt0->flags = RTF_UP;
		// Insert new path for route entry to source of RREQ.
		// (src addr, hop count + 1, lifetime, last hop (first hop for RREQ))
		reversePath = rt0->pathInsert(ipsrc, rq->getFirstHop(), rq->getHopCount()+1, REVERSE_ROUTE_LIFE, rq->getMinLET());
		rt0->lastHopCount = rq->getHopCount() + 1;
		EV << "new reverse path: " << reversePath->info() << ", last hop count: " << (uint32_t)rt0->lastHopCount << std::endl;
	}
	// If a new path with smaller hop count is received
	// (same seqno, better hop count) - try to insert new path in route table.
	else if (rt0->seqno == rq->getOriginatorSeqno() && (rt0->advertisedHops > rq->getHopCount()))
	{
		ASSERT(rt0->flags == RTF_UP); // Make sure path is up

		EV << "Case II: get useful route (" << (uint32_t)rt0->advertisedHops << " >= " << rqHopCount << ").\n";

		// If path already exists - adjust the lifetime of the path.
		if ((reversePath = rt0->disjointPathLookup(ipsrc, rq->getFirstHop())) != nullptr)
		{
			ASSERT(reversePath->hopCount == rq->getHopCount() + 1);
			reversePath->expireAt = simTime() + REVERSE_ROUTE_LIFE;
			reversePath->PET = simTime() + rq->getMinLET();
			if (rt0->expireAt < reversePath->expireAt)
				rt0->expireAt = reversePath->expireAt;
			EV << "reverse path already exists, update expiration time to " << reversePath->expireAt << std::endl;
		}
		// Got a new alternate disjoint reverse path - so insert it.
		// I.e. no path exists which has RREQ source as next hop and no
		// path with RREQ first hop as last hop exists for this route entry.
		// Simply stated: no path with the same last hop exists already.
		else if (rt0->disjointPathExists(ipsrc, rq->getFirstHop()))
		{
			// Only insert new path if not too many paths exists for this destination
			if (rt0->pathList.size() < PAOMBR_MAX_PATHS)
			{
				reversePath = rt0->pathInsert(ipsrc, rq->getFirstHop(), rq->getHopCount()+1, REVERSE_ROUTE_LIFE, rq->getMinLET());
				rt0->lastHopCount = rt0->pathGetMaxHopCount();
				EV << "new reverse path: " << reversePath->info() << ", last hop count: " << (uint32_t)rt0->lastHopCount << std::endl;
			}
		}
	}
	// Older seqno (or same seqno with higher hopcount), i.e. I have a more recent route entry - so drop packet.
	else
	{
		if (rt0->seqno > rq->getOriginatorSeqno())
			EV << "get stale route (" << rt0->seqno << " > " << rq->getOriginatorSeqno() << ")." << std::endl;
		else if (rt0->advertisedHops < rq->getHopCount())
			EV << "get useless route (" << (uint32_t)rt0->advertisedHops << " < " << rqHopCount << ")." << std::endl;
		return;
	}
	// End for putting reverse route in routing table

	// Reset the soft state
	rt0->reqTimeout = SimTime::ZERO;
	rt0->reqCount = 0;
	rt0->reqLastTTL = 0;

	// Find out whether any buffered packet can benefit from the reverse route.
	EV << "reverse path becomes RTF_UP, transfer corresponding packets from rQueue to bQueue.\n";
	transferBufPackets(rqsrc);

#ifdef USE_IRRESPONSIBLE_REBROADCAST
	if (propagation)
	{
		// calculate propagation probability according to breaking direction
		Coord referencePoint(paombrNb->pos + rq->getBreakingDir()*transmissionRadius);
		double distRatio = curPosition.distance(referencePoint) / (2*transmissionRadius);
		double probability = exp(-sqrt(rq->getNbNum())*distRatio);
		ASSERT(probability >= 0 && probability <= 1);
		if (dblrand() > probability)
			propagation = false;
		EV << "rebroadcast probability: " << probability << ", propagation: " << propagation << "\n";
	}
#endif

	// determine I should send RREPp or forward RREQp
	uint32_t i = 0, j = 0;
	RepairingNode *repairs = new RepairingNode[destCount];
	RREPpMessage *rp = new RREPpMessage("routing");
	for (uint32_t k = 0; k < destCount; ++k)
	{
		LAddress::L3Type rqdst = rq->getRepairingNodes(k).addr;
		uint32_t rqDestSeqno = rq->getRepairingNodes(k).seqno;

		// Check route entry for RREQ destination
		PaombrRtEntry *rt = lookupRoutingEntry(rqdst);
		// I am the intended receiver of the RREQ - so send a RREP
		if (rqdst == myAddr)
		{
			if (seqno <= rqDestSeqno)
				seqno = rqDestSeqno + 1;
			// Make sure seq number is even
			if (seqno & 1)
				++seqno;
			EV << "I am RREQp dest, update seqno to " << seqno << std::endl;

			RepairedNode repair(0, rqdst, seqno, MY_ROUTE_TIMEOUT, ipsrc);
			rp->setRepairedNodesArraySize(++j);
			rp->setRepairedNodes(j-1, repair);
		}
		// I have a fresh route entry for RREQ destination - so send RREP
		else if (rt != nullptr && rt->flags == RTF_UP && rt->seqno >= rqDestSeqno)
		{
			ASSERT(rt->seqno % 2 == 0); // is the seqno even?

			bool incurJoint = false;
			uint32_t srcCount = rq->getRepairingNodes(k).srcCount;
			for (uint32_t k2 = 0; k2 < srcCount; ++k2)
			{
				LAddress::L3Type lh = rt->revPathLookup(rq->getRepairingNodes(k).srcs[k2]);
				if (lh != -1 && lh != rq->getRepairingNodes(k).lastHop) // concat original downstream nodes is allowed, which surely does not incur joint
				{
					incurJoint = true;
					break;
				}
			}
			if (reversePath != nullptr && !incurJoint)
			{
				EV << "I have a fresh route to RREQp dest, b->count: " << b->count << " (" << rt->seqno << " >= " << rqDestSeqno << ')' << std::endl;

				if (b->count == 0)
				{
					b->count = 1;

					// store the source for whom I provide help
					rt->revPathInsert(rqsrc, ipsrc);

					// route advertisement
					if (rt->advertisedHops == INFINITE_HOPS)
						rt->advertisedHops = rt->pathGetMaxHopCount();

					PaombrPath *forwardPath = rt->pathSelect();
					rt->error = true;
					rt->connected = true;

					RepairedNode repair(rt->advertisedHops, rqdst, rt->seqno, forwardPath->expireAt-simTime(), forwardPath->lastHop);
					rp->setRepairedNodesArraySize(++j);
					rp->setRepairedNodes(j-1, repair);
				}
			}
			else if (reversePath != nullptr)
			{
				EV << "Reject to reply because this will incur joint path." << std::endl;
			}
		}
		// RREQ not intended for me and I don't have a fresh enough entry for RREQ dest - so forward the RREQ
		else
		{
			if (propagation) // do not propagate a duplicate RREQ
			{
				repairs[i] = rq->getRepairingNodes(k);
				// Maximum sequence number seen in route
				if (rt != nullptr && rqDestSeqno < rt->seqno)
					repairs[i].seqno = rt->seqno;

				// propagation == true, this is a fresh RREQ, assertion rt0->seqno < rq->getOriginatorSeqno() holds, so we purge stale information
				rt0->revPathDelete(rqdst);
				// route advertisement
				if (rt0->advertisedHops == INFINITE_HOPS)
					rt0->advertisedHops = rt0->pathGetMaxHopCount();
				rq->setHopCount(rt0->advertisedHops);
				EV << "forward RREQp, dest seqno: " << rqDestSeqno << ", hop count: " << (uint32_t)rq->getHopCount() << std::endl;
				++i;
			}
		}
	}
	// I can reply some destinations
	if (j > 0)
	{
		rp->setDestCount(j);
		sendRREPp(rq, rp);
	}
	else
		delete rp;
	// I cannot reply all destinations, so forward it
	if (i > 0)
	{
#ifdef USE_IRRESPONSIBLE_REBROADCAST
		rq->setNbNum(neighbors.size());
#endif
		if (i < destCount) // I cannot reply any destination, no need to modify
		{
			rq->setDestCount(i);
			rq->setRepairingNodesArraySize(i);
			for (j = 0; j < i; ++j)
				rq->setRepairingNodes(j, repairs[j]);
		}
		forward(nullptr, rq->dup());
	}
	delete repairs;
}

void PAOMBR::onRREPp(RREPpMessage *rp)
{
	EV << "node[" << myAddr << "]: onRREPp!\n";

	LAddress::L3Type ipsrc = rp->getSenderAddress(), ipdst = rp->getIpDest(); // alias
	EV << "IP src: " << ipsrc << ", IP dest: " << ipdst << "\n";

	uint32_t destCount = rp->getDestCount(), k = 0;
	ASSERT(destCount > 0);
	for (k = 0; k < destCount; ++k)
		if (rp->getRepairedNodes(k).destAddr == myAddr)
			break;
	if (k < destCount)
	{
		for (uint32_t i = k; i < destCount-1; ++i)
			rp->setRepairedNodes(i, rp->getRepairedNodes(i+1));
		rp->setRepairedNodesArraySize(--destCount);
	}
	// I am the only repairing destination in RREPp - drop packet.
	if (destCount == 0)
		return;

	// predict link expiration time
	PaombrNeighborInfo *paombrNb = dynamic_cast<PaombrNeighborInfo*>(neighbors[ipsrc]);
	paombrNb->LET = calcLET(paombrNb->pos, paombrNb->speed);
	EV << "LET: " << paombrNb->LET << "s, rp->LET: " << rp->getMinLET() << "s.\n";
	if (paombrNb->LET < 2*PLRR_DISCOVERY_TIME)
	{
		EV << "Link expiration time too short, discard it." << std::endl;
		return;
	}
	if (rp->getMinLET() > paombrNb->LET)
		rp->setMinLET(paombrNb->LET);

	PaombrBroadcastID *b = lookupBroadcastID(ipdst, rp->getRreqId()); // Check for <RREQ src IP, bcast ID> pair
	ASSERT(b != nullptr);

	// for each repairing destinations, update forward path
	uint32_t continueCount = 0;
	for (k = 0; k < destCount; ++k)
	{
		RepairedNode &repair = rp->getRepairedNodes(k);

		PaombrRtEntry *rt = lookupRoutingEntry(repair.destAddr);
		// If I don't have a rt entry to this host... adding
		if (rt == nullptr)
		{
			EV << "create an entry for the forward route.\n";
			rt = insertRoutingEntry(repair.destAddr);
		}
		rt->connected = true;
		if (ipdst == myAddr)
			rt->pathDelete(rp->getBreakingNb());

		PaombrPath *forwardPath = nullptr;
		// If RREP contains more recent seqno for (RREQ) destination
		// - delete all old paths and add the new forward path to (RREQ) destination
		if (rt->seqno < repair.destSeqno)
		{
			EV << "Case I: get fresher route (" << rt->seqno << " < " << repair.destSeqno << ").\n";

			rt->seqno = repair.destSeqno;
			rt->advertisedHops = INFINITE_HOPS;
			rt->pathList.clear();
			rt->flags = RTF_UP;
			// Insert forward path to RREQ destination.
			forwardPath = rt->pathInsert(ipsrc, repair.firstHop, repair.hopCount+1, repair.lifeTime, rp->getMinLET());
			rt->lastHopCount = repair.hopCount + 1;
			EV << "new forward path: " << forwardPath->info() << ", last hop count: " << (uint32_t)rt->lastHopCount << std::endl;
		}
		// If the sequence number in the RREP is the same as for route entry but
		// with a smaller hop count - try to insert new forward path to (RREQ) dest.
		else if (rt->seqno == repair.destSeqno)
		{
			uint32_t rpHopCount = repair.hopCount + 1;
			if (ipdst != myAddr)
			{
				if (rt->advertisedHops >= rpHopCount)
					EV << "Case II: get useful route (" << (uint32_t)rt->advertisedHops << " >= " << rpHopCount << ").\n";
				else
				{
					EV << "get useless route (" << (uint32_t)rt->advertisedHops << " < " << rpHopCount << ").\n";
					++continueCount;
					continue;
				}
			}
			else
			{
				bool ignore = true;
				if (b->count == 0 && rt->advertisedHops+1 >= repair.hopCount)
				{
					EV << "Case II: get first RREPp (" << (uint32_t)rt->advertisedHops << "+2 >= " << rpHopCount << ").\n";
					ignore = false;
				}
				else if (b->count > 0 && rt->advertisedHops >= rpHopCount)
				{
					EV << "Case III: get useful route (" << (uint32_t)rt->advertisedHops << " >= " << rpHopCount << ").\n";
					ignore = false;
				}
				if (ignore)
				{
					EV << "get useless route (" << (uint32_t)rt->advertisedHops << " < " << rpHopCount << ").\n";
					++continueCount;
					continue;
				}
			}

			rt->flags = RTF_UP;
			// If the path already exists - increase path lifetime
			if ((forwardPath = rt->disjointPathLookup(ipsrc, repair.firstHop)) != nullptr)
			{
				ASSERT(forwardPath->hopCount == repair.hopCount + 1);
				forwardPath->expireAt = simTime() + repair.lifeTime;
				forwardPath->PET = simTime() + rp->getMinLET();
				if (rt->expireAt < forwardPath->expireAt)
					rt->expireAt = forwardPath->expireAt;
				EV << "forward path already exists, update expiration time to " << forwardPath->expireAt << std::endl;
			}
			// If the path does not already exist, and there is room for it - we add the path
			else if (rt->disjointPathExists(ipsrc, repair.firstHop) && rt->pathList.size() < PAOMBR_MAX_PATHS)
			{
				// Insert forward path to RREQ destination.
				forwardPath = rt->pathInsert(ipsrc, repair.firstHop, repair.hopCount+1, repair.lifeTime, rp->getMinLET());
				rt->lastHopCount = rt->pathGetMaxHopCount();
				EV << "new forward path: " << forwardPath->info() << ", last hop count: " << (uint32_t)rt->lastHopCount << std::endl;
			}
			// Path did not exist nor could it be added - just drop packet.
			else
			{
				++continueCount;
				continue;
			}
		}
		// The received RREP did not contain more recent information than route table - so drop packet
		else
		{
			EV << "get stale route (" << rt->seqno << " > " << repair.destSeqno << ").\n";
			++continueCount;
			continue;
		}

		ASSERT(rt->flags == RTF_UP);
		// Reset the soft state
		rt->reqTimeout = SimTime::ZERO;
		rt->reqCount = 0;
		rt->reqLastTTL = 0;

		// Find out whether any buffered packet can benefit from the forward route.
		EV << "forward path becomes RTF_UP, transfer corresponding packets from rQueue to bQueue.\n";
		transferBufPackets(repair.destAddr);

		if (ipdst == myAddr)
			rt->advertisedHops = rt->pathGetMaxHopCount();
	}
	if (continueCount == destCount)
		return;

	// If I am the intended recipient of the RREP, nothing more needs to be done - so drop packet.
	if (ipdst == myAddr)
	{
		EV << "I am the RREPp dest, cancel PLRR timeout event.\n";

#if ROUTING_DEBUG_LOG
		printRoutingTable();
#endif
		b->count = 1;
		paombrNb = dynamic_cast<PaombrNeighborInfo*>(neighbors[rp->getBreakingNb()]);
		RoutingStatisticCollector::gRouteAcquisitionTime += simTime() - paombrNb->PLRRTimeoutEvt->getSendingTime();
		cancelEvent(paombrNb->PLRRTimeoutEvt);
		return;
	}

	// If I am not the intended recipient of the RREP
	// - check route table for a path to the RREP dest (i.e. the RREQ source).
	PaombrRtEntry *rt0 = lookupRoutingEntry(ipdst);
	if (rt0 == nullptr || rt0->flags != RTF_UP || b->count > 0)
		return;

	rt0->connected = true;
	b->count = 1;
	PaombrPath *reversePath = rt0->pathSelect();
	reversePath->expireAt = simTime() + ACTIVE_ROUTE_TIMEOUT;
	EV << "I have an active route to RREQp src, reverse path: " << reversePath->info() << "\n";

	for (k = 0; k < destCount; ++k)
	{
		RepairedNode &repair = rp->getRepairedNodes(k);

		// store the source for whom I provide help
		rt0->revPathInsert(repair.destAddr, ipsrc);

		PaombrRtEntry *rt = lookupRoutingEntry(repair.destAddr);
		// route advertisement
		if (rt->advertisedHops == INFINITE_HOPS)
			rt->advertisedHops = rt->pathGetMaxHopCount();
		repair.hopCount = rt->advertisedHops;
		repair.firstHop = rt->pathSelect()->lastHop;
		rt->error = true;
	}
	EV << "forward RREPp." << std::endl;
	LAddress::L3Type prevHop = forward(rt0, rp->dup());
	// store the source for whom I provide help
	for (k = 0; k < destCount; ++k)
	{
		PaombrRtEntry *rt = lookupRoutingEntry(rp->getRepairedNodes(k).destAddr);
		rt->revPathInsert(ipdst, prevHop);
	}

#if ROUTING_DEBUG_LOG
	printRoutingTable();
#endif
}
#endif

#ifdef USE_RECEIVER_REPORT
void PAOMBR::onRR(RRMessage *rr)
{
	EV << "node[" << myAddr << "]: onRR!\n";

	if (rr->getIpDest() == myAddr)
	{
		EV << "this receiver report is for me, its info:\n";
		EV << "    last hop " << rr->getFirstHop() << "\n";
		EV << "    lb seqno " << rr->getLbSeqno() << "\n";
		EV << "    ub seqno " << rr->getUbSeqno() << "\n";
		EV << "    delay " << rr->getDelay() << "\n";
		EV << "    jitter " << rr->getJitter() << "\n";
		if ((itSS = senderStates.find(rr->getDestination())) != senderStates.end())
		{
			SendState &sendState = itSS->second;
			SendState::PathState &pathState = sendState.states[rr->getFirstHop()];
			std::vector<int>::iterator itLB = std::lower_bound(pathState.seqnos.begin(), pathState.seqnos.end(), rr->getLbSeqno());
			std::vector<int>::iterator itUB = std::lower_bound(pathState.seqnos.begin(), pathState.seqnos.end(), rr->getUbSeqno());
			int totalPktSent = std::distance(itLB, itUB);
			if (totalPktSent == 0)
				totalPktSent = 1;
			pathState.deliverRatio = static_cast<double>(sendRRPktsThres)/totalPktSent;
			EV << "    deliver ratio " << pathState.deliverRatio << std::endl;
		}
		return;
	}

	PaombrRtEntry *rt = lookupRoutingEntry(rr->getIpDest());
	forward(rt, rr->dup(), rr->getDestination());
}
#endif

void PAOMBR::onLR(LRMessage *lr)
{
	EV << "node[" << myAddr << "]: onLR!\n";

	lr->setHopCount(lr->getHopCount()+1);

	// predict link expiration time
	if ((itN = neighbors.find(lr->getSenderAddress())) == neighbors.end())
	{
		EV << "Sender is not my neighbor, discard it." << std::endl;
		return;
	}
	simtime_t LET = calcLET(itN->second->pos, itN->second->speed);
	EV << "LET: " << LET << "s, lr->LET: " << lr->getMinLET() << "s.\n";
	if (lr->getMinLET() > LET)
		lr->setMinLET(LET);

	if (lr->getIpDest() == myAddr)
	{
		EV << "this last hop report is for me, its info:\n";
		EV << "    destination " << lr->getDestination() << "\n";
		EV << "    old last hop " << lr->getOldLastHop() << "\n";
		EV << "    new last hop " << lr->getNewLastHop() << "\n";
		EV << "    hop count " << (uint32_t)lr->getHopCount() << std::endl;
		PaombrRtEntry *rt = lookupRoutingEntry(lr->getDestination());
		ASSERT(rt != nullptr);
		PaombrPath *path = rt->pathLookupLastHop(lr->getOldLastHop());
		if (path != nullptr)
			path->lastHop = lr->getNewLastHop();
		else
			rt->pathInsert(lr->getSenderAddress(), lr->getNewLastHop(), lr->getHopCount(), lr->getLifeTime(), lr->getMinLET());
		return;
	}

	PaombrRtEntry *rt0 = lookupRoutingEntry(lr->getIpDest());
	forward(rt0, lr->dup(), lr->getDestination());
}

/**
 * 3.2.4. Data packet forwarding
 * For data packet forwarding at a node having multiple paths to a destination, we adopt
 * a simple approach of using a path until it fails and then switch to an alternate path;
 * we use paths in the order of their creation.
 */
void PAOMBR::onData(DataMessage *dataPkt)
{
	EV << "node[" << myAddr << "]: onData!\n";

#ifndef USE_L2_UNICAST_DATA
	if (dataPkt->getReceiverAddress() != myAddr)
		return;

	int altitude = dataPkt->getAltitude() + 1;
	dataPkt->setAltitude(altitude);
#endif

	LAddress::L3Type dest = dataPkt->getDestination(); // alias
	if (dest == myAddr)
	{
		itRS = receiverStates.find(dataPkt->getSource());
		if (itRS == receiverStates.end())
			itRS = receiverStates.insert(std::pair<LAddress::L3Type, RecvState>(dataPkt->getSource(), RecvState())).first;
		RecvState &recvState = itRS->second;
		bool duplicated = recvState.onRecv(dataPkt->getSequence());
		simtime_t curDelay = simTime() - dataPkt->getCreationTime();
		RoutingStatisticCollector::gDataBitsRecv += dataPkt->getBitLength() - headerLength;
		if (!duplicated)
			RoutingStatisticCollector::gDataPktsRecv++;
		else
			RoutingStatisticCollector::gDataPktsDuplicated++;
		RoutingStatisticCollector::gEndtoendDelay += curDelay;
#ifdef USE_RECEIVER_REPORT
		RecvState::PathState &pathState = recvState.states[dataPkt->getSenderAddress()];
		if (pathState.highestSeqno < dataPkt->getSequence())
			pathState.highestSeqno = dataPkt->getSequence();
		pathState.delay += curDelay;
		if (pathState.lastDelay != SimTime::ZERO)
		{
			simtime_t delayDiff = curDelay >= pathState.lastDelay ? curDelay - pathState.lastDelay : pathState.lastDelay - curDelay;
			pathState.jitter = pathState.jitter + (delayDiff - pathState.jitter)/16;
			// see Page 33 of RFC 3550 - RTP: A Transport Protocol for Real-Time Applications
		}
		pathState.lastDelay = curDelay;
		if (++pathState.pktsRecv == sendRRPktsThres) // send a receiver report
		{
			PaombrRtEntry *rt0 = lookupRoutingEntry(dataPkt->getSource());
			ASSERT(rt0 != nullptr);

			EV << "send a receiver report to " << dataPkt->getSource() << ", last hop: " << dataPkt->getSenderAddress() << "\n";

			RRMessage *rr = new RRMessage("routing");
			prepareWSM(rr, routingLengthBits+224, t_channel::type_CCH, routingPriority, lookupL2Address(dataPkt->getSenderAddress()));
			rr->setIpDest(dataPkt->getSource());
			rr->setTTL(rt0->pathGetMaxHopCount()+1); // ensure TTL is large enough
			rr->setPacketType(AOMDVPacketType::RR);
			rr->setDestination(myAddr);
			rr->setFirstHop(dataPkt->getSenderAddress());
			rr->setLbSeqno(pathState.prevHighest);
			rr->setUbSeqno(pathState.highestSeqno);
			rr->setDelay(pathState.delay/sendRRPktsThres);
			rr->setJitter(pathState.jitter);
			sendWSM(rr);

			pathState.rotate();
		}
#endif
		EV << "this data packet is for me, seqno: " << dataPkt->getSequence() << ", seqno intervals:\n";
		for (recvState.itS = recvState.segments.begin(); recvState.itS != recvState.segments.end(); ++recvState.itS)
			EV << '[' << recvState.itS->first << ',' << recvState.itS->second << "), ";
		EV << std::endl;
		return;
	}

	PaombrRtEntry *rt = lookupRoutingEntry(dest);
	// A node initiates processing for a RERR message in three situations:
	// (ii) if it gets a data packet destined to a node for which it does not have
	//      an active route and is not repairing (if using local repair), or
	if (rt == nullptr || rt->flags == RTF_DOWN)
	{
		if (rt != nullptr)
		{
			ASSERT(rt->advertisedHops == INFINITE_HOPS && rt->pathList.empty());
			EV << "RTF_DOWN, send RERR." << std::endl;
		}
		else
			EV << "no route entry, send RERR." << std::endl;

		++RoutingStatisticCollector::gPktsLinkLost;

		RERRMessage *re = new RERRMessage("routing");
		re->setDestCount(1);
		UnreachableNode unreach;
		unreach.addr = dest;
		unreach.seqno = rt != nullptr ? rt->seqno : INFINITE_HOPS;
		re->setUnreachableNodesArraySize(1);
		re->setUnreachableNodes(0, unreach);
		sendRERR(re);
	}
	else if (rt->flags == RTF_UP || rt->flags == RTF_IN_PLRR)
	{
		size_t queueSize = pushBufferQueue(dataPkt->dup());
		if (queueSize > 0 && !sendBufDataEvt->isScheduled())
			scheduleAt(simTime(), sendBufDataEvt);

		ASSERT(rt->lastHopCount != INFINITE_HOPS && !rt->pathList.empty());
		LAddress::L3Type nextHop = rt->pathSelect()->nextHop;

		EV << "RTF_UP, prepare forward it to next hop: " << nextHop << ", queue size: " << queueSize << std::endl;
	}
	else // rt->flags == RTF_IN_REPAIR
	{
		rQueue.push_back(dataPkt->dup());

		EV << "RTF_IN_REPAIR, buffer it in rQueue." << std::endl;
	}
}

void PAOMBR::onDataLost(DataMessage *lostDataPkt)
{
	EV << "node[" << myAddr << "]: onDataLost!\n";

#ifdef USE_L2_UNICAST_DATA
	LAddress::L3Type nextHop = lookupL3Address(lostDataPkt->getRecipientAddress());
#else
	LAddress::L3Type nextHop = lostDataPkt->getReceiverAddress();
#endif
	ASSERT(nextHop != -1);

	onLinkBroken(nextHop, lostDataPkt);
}

//////////////////////////////    TX functions    //////////////////////////////
LAddress::L3Type PAOMBR::forward(PaombrRtEntry *rt, RoutingMessage *pkt, LAddress::L3Type dest)
{
	LAddress::L3Type nextHop = LAddress::L3BROADCAST();
	if (pkt->getTTL() == 0)
	{
		EV << "TTL becomes zero, discard the packet." << std::endl;
		delete pkt;
		return nextHop;
	}

	if (rt != nullptr) // if it is not a broadcast packet
	{
		if (pkt->getPacketType() != AOMDVPacketType::RR && pkt->getPacketType() != AOMDVPacketType::LR)
			ASSERT(rt->flags == RTF_UP || rt->flags == RTF_IN_PLRR);
		else if (rt->flags == RTF_DOWN || rt->flags == RTF_IN_REPAIR)
		{
			EV << "rt->flags == RTF_DOWN || rt->flags == RTF_IN_REPAIR, discard it." << std::endl;
			delete pkt;
			return nextHop;
		}

		LAddress::L3Type lastHop = rt->revPathLookup(dest); // first hop from RREQ src to dest
		PaombrPath *path = rt->pathLookupLastHop(lastHop);
		ASSERT(path != nullptr);
		path->expireAt = simTime() + ACTIVE_ROUTE_TIMEOUT;
		pkt->setRecipientAddress(lookupL2Address(path->nextHop));
		nextHop = path->nextHop;
	}
	else
		ASSERT(pkt->getIpDest() == LAddress::L3BROADCAST());

	pkt->setSenderAddress(myAddr);
	pkt->setSenderPos(curPosition);
	pkt->setSenderSpeed(curSpeed);

	RoutingStatisticCollector::gCtrlBitsTransmitted += pkt->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	sendWSM(pkt);
	return nextHop;
}

void PAOMBR::sendRREQ(LAddress::L3Type dest, uint8_t TTL, LAddress::L3Type lastHop)
{
	PaombrRtEntry *rt = lookupRoutingEntry(dest);
	ASSERT(rt != nullptr);

	// Rate limit sending of Route Requests. We are very conservative about sending out route requests.
	if (rt->flags == RTF_UP)
	{
		ASSERT(rt->lastHopCount != INFINITE_HOPS);
		EV << "RTF_UP, no need to send RREQ." << std::endl;
		return;
	}

	if (rt->reqTimeout > simTime())
	{
		EV << "last RREQ has not timeout yet, no need to send RREQ." << std::endl;
		return;
	}

	// rt->reqCount is the no. of times we did network-wide broadcast
	// RREQ_RETRIES is the maximum number allowed for network-wide broadcast
	if (rt->reqCount > RREQ_RETRIES)
	{
		rt->reqTimeout = SimTime::ZERO;
		rt->reqCount = 0;
		ASSERT(rt->reqLastTTL == NET_DIAMETER);
		rt->flags = RTF_UP; // avoid instantly returning from downRoutingEntry()
		downRoutingEntry(rt);
		onRouteUnreachable(dest);
		return;
	}

	seqno += 2;
	ASSERT(seqno % 2 == 0);

	// The Destination Sequence Number field in the RREQ message is the last
	// known destination sequence number for this destination and is copied
	// from the Destination Sequence Number field in the routing table. If
	// no sequence number is known, the unknown sequence number flag MUST be
	// set. The Originator Sequence Number in the RREQ message is the
	// node’s own sequence number, which is incremented prior to insertion
	// in a RREQ. The RREQ ID field is incremented by one from the last
	// RREQ ID used by the current node. Each node maintains only one RREQ
	// ID. The Hop Count field is set to zero.
	RREQMessage *rq = new RREQMessage("routing");
	prepareWSM(rq, routingLengthBits+224, t_channel::type_CCH, routingPriority);
	rq->setSenderPos(curPosition);
	rq->setSenderSpeed(curSpeed);
	// rq->setIpDest(LAddress::L3BROADCAST());
	// rq->setHopCount(0);
	rq->setPacketType(AOMDVPacketType::RREQ);
	rq->setRreqId(++rreqID);
	rq->setOriginatorAddr(myAddr);
	rq->setOriginatorSeqno(seqno);
	rq->setDestAddr(dest);
	rq->setDestSeqno(rt->seqno);
	rq->setMinLET(SimTime(3600, SIMTIME_S));

	if (TTL == INFINITE_HOPS)
	{
		rt->brokenNb = -1; // when RREP timeout event happens, indicates it is not a local repair failure event
		// Determine the TTL to be used this time. Dynamic TTL evaluation - SRD
		if (rt->reqLastTTL < rt->lastHopCount && rt->lastHopCount != INFINITE_HOPS)
			rt->reqLastTTL = rt->lastHopCount;
		if (rt->reqLastTTL == 0) // first time query broadcast
			rq->setTTL(TTL_START);
		else // Expanding ring search.
		{
			if (rt->reqLastTTL < TTL_THRESHOLD)
				rq->setTTL(rt->reqLastTTL + TTL_INCREMENT);
			else // network-wide broadcast
			{
				rq->setTTL(NET_DIAMETER);
				++rt->reqCount;
			}
		}
	}
	else // local repair
	{
		rq->setRepairFlag(true);
		rq->setTTL(TTL);
		rq->setLastHop(lastHop);
		uint32_t srcCount = 0;
		for (rt->itRPL = rt->revPathList.begin(); rt->itRPL != rt->revPathList.end(); ++rt->itRPL)
		{
			rq->setSrcsArraySize(++srcCount);
			rq->setSrcs(srcCount-1, rt->itRPL->first);
		}
		rq->setSrcCount(srcCount);
	}
	// remember the TTL used  for the next time
	rt->reqLastTTL = rq->getTTL();

	rt->reqTimeout = 2 * rq->getTTL() * NODE_TRAVERSAL_TIME;
	if (rt->reqCount > 1)
		rt->reqTimeout *= rt->reqCount;
	if (rt->reqTimeout > RREP_WAIT_TIME)
		rt->reqTimeout = RREP_WAIT_TIME;
	rt->reqTimeout += simTime();

	if (rt->rrepTimeoutEvt == nullptr)
	{
		rt->rrepTimeoutEvt = new WaitForRREPMessage("rrep timeout evt", PAOMBRMsgKinds::RREP_TIMEOUT_EVT);
		rt->rrepTimeoutEvt->setContextPointer(&rt->brokenNb);
		rt->rrepTimeoutEvt->setDestAddr(dest);
	}
	ASSERT(!rt->rrepTimeoutEvt->isScheduled());
	scheduleAt(rt->reqTimeout, rt->rrepTimeoutEvt);

	EV << "send RREQ to dest " << dest << ", TTL: " << (uint32_t)rq->getTTL() << ", timeout: " << rt->reqTimeout << std::endl;

	RoutingStatisticCollector::gCtrlBitsTransmitted += rq->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	RoutingStatisticCollector::gRREQs++;
	sendWSM(rq);
}

void PAOMBR::sendRREP(RREQMessage *rq, uint8_t hopCount, uint32_t rpseq, simtime_t lifetime, LAddress::L3Type lastHop)
{
	RREPMessage *rp = new RREPMessage("routing");
	int rrepLengthBits = rq->getPreemptiveFlag() ? 288 : 256;
	prepareWSM(rp, routingLengthBits+rrepLengthBits, t_channel::type_CCH, routingPriority, lookupL2Address(rq->getSenderAddress()));
	rp->setSenderPos(curPosition);
	rp->setSenderSpeed(curSpeed);
	rp->setIpDest(rq->getOriginatorAddr());
	rp->setTTL(rq->getHopCount()+1);
	rp->setPacketType(AOMDVPacketType::RREP);
	rp->setRepairFlag(rq->getRepairFlag());
	rp->setPreemptiveFlag(rq->getPreemptiveFlag());
	rp->setHopCount(hopCount);
	rp->setDestAddr(rq->getDestAddr());
	rp->setDestSeqno(rpseq);
	rp->setOriginatorAddr(rq->getOriginatorAddr());
	rp->setLifeTime(lifetime);
	rp->setMinLET(SimTime(3600, SIMTIME_S));
	rp->setRreqId(rq->getRreqId());
	rp->setFirstHop(lastHop);
	rp->setLastHop(rq->getLastHop());
	if (rq->getPreemptiveFlag())
		rp->setBreakingNb(rq->getBreakingNb());

	EV << "send RREP to src " << rp->getIpDest() << ", TTL: " << (uint32_t)rp->getTTL() << ", dest seqno: " << rp->getDestSeqno() << std::endl;

	RoutingStatisticCollector::gCtrlBitsTransmitted += rp->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	if (rq->getPreemptiveFlag())
		RoutingStatisticCollector::gRREPps++;
	else
		RoutingStatisticCollector::gRREPs++;
	sendWSM(rp);
}

void PAOMBR::sendRERR(RERRMessage *rerr)
{
	ASSERT(rerr != nullptr);

	int rerrLengthBits = 32 + 64*rerr->getDestCount();
	prepareWSM(rerr, routingLengthBits+rerrLengthBits, t_channel::type_CCH, routingPriority);
	rerr->setSenderPos(curPosition);
	rerr->setSenderSpeed(curSpeed);
	// rerr->setIpDest(LAddress::L3BROADCAST());
	// rerr->setTTL(1);
	rerr->setPacketType(AOMDVPacketType::RERR);
	// DestCount and list of unreachable destinations are already filled

	EV << "send RERR from node " << myAddr << ", dest count: " << (uint32_t)rerr->getDestCount() << std::endl;

	RoutingStatisticCollector::gCtrlBitsTransmitted += rerr->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	RoutingStatisticCollector::gRERRs++;
	sendWSM(rerr);
}

#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
void PAOMBR::sendRREQp(LAddress::L3Type neighbor)
{
	if (routingTable.empty())
		return;

	uint32_t destCount = 0;
	RepairingNode *repairs = new RepairingNode[routingTable.size()];
	// For each routing entry
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
	{
		PaombrRtEntry *rt = itRT->second;
		if (rt->flags == RTF_UP && rt->pathLookup(neighbor) != nullptr)
		{
			// there is no path left if disconnect from this neighbor || there is no path has adequate PET
			if (rt->connected && (rt->pathList.size() == 1 || rt->pathGetMaxPET() - simTime() < SimTime(1, SIMTIME_S)))
			{
				rt->flags = RTF_IN_PLRR;
				repairs[destCount].addr = itRT->first;
				repairs[destCount].seqno = rt->seqno;
				repairs[destCount].lastHop = rt->pathLookup(neighbor)->lastHop;
				uint32_t srcCount = 0;
				for (rt->itRPL = rt->revPathList.begin(); rt->itRPL != rt->revPathList.end(); ++rt->itRPL, ++srcCount)
					repairs[destCount].srcs[srcCount] = rt->itRPL->first;
				repairs[destCount].srcCount = srcCount;
				++destCount;
			}
		}
	}
	if (destCount == 0)
	{
		EV << "No route affected by neighbor " << neighbor << std::endl;
		delete repairs;
		return;
	}

	seqno += 2;
	ASSERT(seqno % 2 == 0);

	PaombrNeighborInfo *paombrNb = dynamic_cast<PaombrNeighborInfo*>(neighbors[neighbor]);
#ifdef USE_IRRESPONSIBLE_REBROADCAST
	Coord breakingDir = paombrNb->pos - curPosition;
	breakingDir /= breakingDir.length();
#endif

#ifndef USE_DESTINATION_AGGREGATION
	for (uint32_t k = 0; k < destCount; ++k)
	{
		RREQMessage *rq = new RREQMessage("routing");
		int rreqLengthBits = 432 + 32*repairs[k].srcCount;
		prepareWSM(rq, routingLengthBits+rreqLengthBits, t_channel::type_CCH, routingPriority);
		rq->setSenderPos(curPosition);
		rq->setSenderSpeed(curSpeed);
		// rq->setIpDest(LAddress::L3BROADCAST());
		rq->setTTL(1 + LOCAL_ADD_TTL);
		rq->setPacketType(AOMDVPacketType::RREQ);
		rq->setPreemptiveFlag(true);
		rq->setDestAddr(repairs[k].addr);
		rq->setDestSeqno(repairs[k].seqno);
		rq->setLastHop(repairs[k].lastHop);
		rq->setSrcCount(repairs[k].srcCount);
		rq->setSrcsArraySize(repairs[k].srcCount);
		for (uint32_t j = 0; j < repairs[k].srcCount; ++j)
			rq->setSrcs(j, repairs[k].srcs[j]);
#else
	RREQpMessage *rq = new RREQpMessage("routing");
	rq->setDestCount(destCount);
	rq->setRepairingNodesArraySize(destCount);
	for (uint32_t k = 0; k < destCount; ++k)
		rq->setRepairingNodes(k, repairs[k]);
	int rreqpLengthBits = 160 + 64*destCount;
	prepareWSM(rq, routingLengthBits+rreqpLengthBits, t_channel::type_CCH, routingPriority);
	rq->setSenderPos(curPosition);
	rq->setSenderSpeed(curSpeed);
	// rq->setIpDest(LAddress::L3BROADCAST());
	rq->setTTL(1 + LOCAL_ADD_TTL);
	rq->setPacketType(AOMDVPacketType::RREQp);
#endif
	rq->setRepairFlag(true);
	rq->setRreqId(++rreqID);
	rq->setOriginatorAddr(myAddr);
	rq->setOriginatorSeqno(seqno);
	rq->setBreakingNb(neighbor);
#ifdef USE_IRRESPONSIBLE_REBROADCAST
	rq->setBreakingDir(breakingDir);
	rq->setNbNum(neighbors.size());
#endif
	rq->setMinLET(SimTime(3600, SIMTIME_S));

	insertBroadcastID(myAddr, rq->getRreqId()); // b->count == 0 indicates first RREPp received

	RoutingStatisticCollector::gCtrlBitsTransmitted += rq->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	RoutingStatisticCollector::gRREQps++;
	sendWSM(rq);
#ifndef USE_DESTINATION_AGGREGATION
	}
#endif

	paombrNb->PLRRTimeout = simTime() + 2*(1+LOCAL_ADD_TTL)*NODE_TRAVERSAL_TIME;
	if (paombrNb->PLRRTimeoutEvt == nullptr)
	{
		paombrNb->PLRRTimeoutEvt = new cMessage("PLRR timeout evt", PAOMBRMsgKinds::PLRR_TIMEOUT_EVT);
		paombrNb->PLRRTimeoutEvt->setContextPointer(&paombrNb->nb);
	}
	ASSERT(!paombrNb->PLRRTimeoutEvt->isScheduled());
	scheduleAt(paombrNb->PLRRTimeout, paombrNb->PLRRTimeoutEvt);
	paombrNb->PLRRSuppressUntil = simTime() + PLRR_SUPPRESSION_TIME;

	EV << "send RREQp, dest count: " << destCount << ", timeout: " << paombrNb->PLRRTimeout << std::endl;
}

#ifdef USE_DESTINATION_AGGREGATION
void PAOMBR::sendRREPp(RREQpMessage *rq, RREPpMessage *rp)
{
	ASSERT(rp != nullptr);

	int rreppLengthBits = 192 + 192*rp->getDestCount();
	prepareWSM(rp, routingLengthBits+rreppLengthBits, t_channel::type_CCH, routingPriority, lookupL2Address(rq->getSenderAddress()));
	rp->setSenderPos(curPosition);
	rp->setSenderSpeed(curSpeed);
	rp->setIpDest(rq->getOriginatorAddr());
	rp->setTTL(rq->getHopCount()+1);
	rp->setPacketType(AOMDVPacketType::RREPp);
	rp->setOriginatorAddr(rq->getOriginatorAddr());
	rp->setRreqId(rq->getRreqId());
	rp->setBreakingNb(rq->getBreakingNb());
	rp->setMinLET(SimTime(3600, SIMTIME_S));
	// DestCount and list of repairing destinations are already filled

	EV << "send RREPp to src " << rp->getIpDest() << ", TTL: " << (uint32_t)rp->getTTL() << std::endl;

	RoutingStatisticCollector::gCtrlBitsTransmitted += rp->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	RoutingStatisticCollector::gRREPps++;
	sendWSM(rp);
}
#endif
#endif

void PAOMBR::sendLR(PaombrRtEntry *rt, RREPMessage *rp)
{
	for (rt->itRPL = rt->revPathList.begin(); rt->itRPL != rt->revPathList.end(); ++rt->itRPL)
	{
		PaombrRtEntry *rt0 = lookupRoutingEntry(rt->itRPL->first);
		ASSERT(rt0 != nullptr);

		LRMessage *lr = new LRMessage("routing");
		prepareWSM(lr, routingLengthBits+144, t_channel::type_CCH, routingPriority, lookupL2Address(rt0->pathSelect()->nextHop));
		lr->setIpDest(rt->itRPL->first);
		lr->setTTL(rt0->pathGetMaxHopCount()+1); // ensure TTL is large enough
		lr->setPacketType(AOMDVPacketType::LR);
		lr->setDestination(rp->getDestAddr());
		lr->setOldLastHop(rp->getLastHop());
		lr->setNewLastHop(rp->getFirstHop());
		lr->setHopCount(rp->getHopCount());
		lr->setLifeTime(rp->getLifeTime());
		lr->setMinLET(rp->getMinLET());

		EV << "send LR to src " << lr->getIpDest() << ", TTL: " << (uint32_t)lr->getTTL() << std::endl;

		RoutingStatisticCollector::gCtrlBitsTransmitted += lr->getBitLength();
		RoutingStatisticCollector::gCtrlPktsTransmitted++;
		sendWSM(lr);
	}
}

void PAOMBR::onNeighborLost(LAddress::L3Type neighbor)
{
	seqno += 2; // Set of neighbors changed
	ASSERT(seqno % 2 == 0);
#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
	PaombrNeighborInfo *paombrNb = dynamic_cast<PaombrNeighborInfo*>(itN->second);
	cancelAndDelete(paombrNb->LETTimeoutEvt);
	cancelAndDelete(paombrNb->PLRRTimeoutEvt);
#endif

	PaombrPath *path = nullptr;
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
	{
		PaombrRtEntry *rt = itRT->second;
		if ((path = rt->pathLookup(neighbor)) != nullptr)
		{
			if (rt->connected && rt->pathList.size() == 1)
			{
				rt->brokenNb = neighbor;
				rt->flags = RTF_IN_REPAIR;
				rt->advertisedHops = INFINITE_HOPS;
				++rt->seqno;
				if (rt->seqno < rt->highestSeqnoHeard)
					rt->seqno = rt->highestSeqnoHeard;
				++RoutingStatisticCollector::gLocalRepairs;
				sendRREQ(itRT->first, path->hopCount + LOCAL_ADD_TTL, rt->pathLookup(neighbor)->lastHop);
			}
			rt->pathDelete(neighbor);
		}
	}
}

/**
 * 6.12. Local Repair
 * When a link break in an active route occurs, the node upstream of that break MAY choose to
 * repair the link locally if the destination was no farther than MAX_REPAIR_TTL hops away.
 * To repair the link break, the node increments the sequence number for the destination and
 * then broadcasts a RREQ for that destination. The TTL of the RREQ should initially be set to
 * the following value:
 *     max(MIN_REPAIR_TTL, 0.5 * #hops) + LOCAL_ADD_TTL,
 * where #hops is the number of hops to the sender (originator) of the currently undeliverable
 * packet. Thus, local repair attempts will often be invisible to the originating node, and
 * will always have TTL >= MIN_REPAIR_TTL + LOCAL_ADD_TTL. The node initiating the repair then
 * waits the discovery period to receive RREPs in response to the RREQ. During local repair
 * data packets SHOULD be buffered. If, at the end of the discovery period, the repairing node
 * has not received a RREP (or other control message creating or updating the route) for that destination,
 * it proceeds as described in Section 6.11 by transmitting a RERR message for that destination.
 */
void PAOMBR::localRepair(PaombrRtEntry *rt, DataMessage *dataPkt, LAddress::L3Type lastHop)
{
	PaombrRtEntry *rt0 = lookupRoutingEntry(dataPkt->getSource());
	ASSERT(rt0 != nullptr && rt0->flags == RTF_UP);

	if (rt->flags == RTF_IN_REPAIR)
		return;
	// mark the route as under repair
	rt->flags = RTF_IN_REPAIR;

	++RoutingStatisticCollector::gLocalRepairs;

	uint8_t MIN_REPAIR_TTL = rt->lastHopCount, halfReverseHop = rt0->lastHopCount/2;
	sendRREQ(dataPkt->getDestination(), std::max(MIN_REPAIR_TTL, halfReverseHop) + LOCAL_ADD_TTL, lastHop);
}

//////////////////////////////    RouteRepairInterface    //////////////////////////////
/**
 * 3.2.3. Route maintenance
 * Like AODV, PAOMBR also uses RERR packets. A node generates or forwards a RERR for a destination
 * when the last path to the destination breaks. PAOMBR also includes an optimization to salvage
 * packets forwarded over failed links by re-forwarding them over alternate paths.
 */
void PAOMBR::onLinkBroken(LAddress::L3Type neighbor, DataMessage *dataPkt)
{
	EV << "link with neighbor " << neighbor << " is broken when transmitting to " << dataPkt->getDestination() << "\n";

	PaombrRtEntry *rt = lookupRoutingEntry(dataPkt->getDestination());
	ASSERT(rt != nullptr);
	LAddress::L3Type lastHop = rt->pathLookup(neighbor)->lastHop;
	rt->brokenNb = neighbor;
	rt->pathDelete(neighbor);
	// salvage the packet using an alternate path if available.
	if (rt->flags == RTF_UP && !rt->pathList.empty())
	{
		PaombrPath *path = rt->pathSelect();
		dataPkt->setSenderAddress(myAddr);
#ifdef USE_L2_UNICAST_DATA
		dataPkt->setRecipientAddress(lookupL2Address(path->nextHop));
#else
		dataPkt->setReceiverAddress(path->nextHop);
#endif

		RoutingStatisticCollector::gDataBitsTransmitted += dataPkt->getBitLength() - headerLength;
		RoutingStatisticCollector::gCtrlBitsTransmitted += headerLength;
		RoutingStatisticCollector::gDataPktsTransmitted++;
		sendDelayedDown(dataPkt->dup(), SimTime::ZERO);

		EV << "salvage this data packet to " << path->nextHop << std::endl;
	}
	else if (rt->lastHopCount <= MAX_REPAIR_TTL)
	{
		if (dataPkt->getSource() != myAddr)
			localRepair(rt, dataPkt, lastHop);
		else
		{
			rt->flags = RTF_IN_REPAIR;
			sendRREQ(dataPkt->getDestination());
		}
		++RoutingStatisticCollector::gPktsLinkLost;
	}
	else
	{
		rt->pathInsert(neighbor, -1, 0, SimTime::ZERO, SimTime::ZERO); // ensure we can enter if block in onLocalRepairFailure()
		onLocalRepairFailure(neighbor);
		++RoutingStatisticCollector::gPktsLinkLost;
	}
}

void PAOMBR::onLocalRepairFailure(LAddress::L3Type neighbor)
{
	EV << "local repair failed, broken neighbor: " << neighbor << std::endl;
#if ROUTING_DEBUG_LOG
	printRoutingTable();
#endif
	RERRMessage *re = new RERRMessage("routing");
	uint8_t destCount = 0;
	// For each routing entry
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
	{
		PaombrRtEntry *rt = itRT->second;
		if (rt->flags == RTF_UP && rt->pathLookup(neighbor) != nullptr)
		{
			rt->pathDelete(neighbor);
			// when all links from a node i leading to a destination d break, then node i
			// locally increments the seqno_i^d and sets the advertised_hop_count_i^d to INFINITE_HOPS.
			if (rt->pathList.empty())
			{
				++rt->seqno;
				if (rt->seqno < rt->highestSeqnoHeard)
					rt->seqno = rt->highestSeqnoHeard;
				downRoutingEntry(rt);
				if (rt->error)
				{
					UnreachableNode unreach;
					unreach.addr = itRT->first;
					unreach.seqno = rt->seqno;
					re->setUnreachableNodesArraySize(++destCount);
					re->setUnreachableNodes(destCount-1, unreach);
					rt->error = false;
				}
			}
		}
	}
	if (destCount > 0)
	{
		re->setDestCount(destCount);
		sendRERR(re);
	}
	else
		delete re;
}

void PAOMBR::onRouteReachable(LAddress::L3Type dest)
{
	EV << "route to dest " << dest << " becomes reachable, list all paths as follows:\n";

	PaombrRtEntry *rt = lookupRoutingEntry(dest);
	std::string paths = rt->pathPrint();
	EV << paths.c_str() << std::endl;

	if (!sendBufDataEvt->isScheduled())
		scheduleAt(simTime(), sendBufDataEvt);
}

void PAOMBR::onRouteUnreachable(LAddress::L3Type dest)
{
	EV << "route to dest " << dest << " becomes unreachable, discard buffered packets." << std::endl;

	for (itRQ = rQueue.begin(); itRQ != rQueue.end();)
	{
		if ((*itRQ)->getDestination() == dest)
		{
			++RoutingStatisticCollector::gPktsLinkLost;
			delete *itRQ;
			itRQ = rQueue.erase(itRQ);
		}
		else
			++itRQ;
	}

	if ((itSS = senderStates.find(dest)) != senderStates.end()) // I am RREQ source
	{
		cancelAndDelete(itSS->second.sendDataEvt);
		senderStates.erase(itSS);
	}
}

void PAOMBR::callRouting(const PlanEntry& entry)
{
	EV << "node[" << myAddr << "]: callRouting!\n";

	LAddress::L3Type receiver = entry.receiver;
#ifndef TEST_ROUTE_REPAIR_PROTOCOL
	itN = neighbors.find(receiver);
	ASSERT(itN != neighbors.end());
#endif

	itSS = senderStates.find(receiver);
	ASSERT(itSS == senderStates.end());
	itSS = senderStates.insert(std::pair<LAddress::L3Type, SendState>(receiver, SendState(entry.totalBytes, receiver))).first;
	SendState &sendState = itSS->second;
	sendState.sendDataEvt = new cMessage("send data evt", PAOMBRMsgKinds::SEND_DATA_EVT);
	sendState.sendDataEvt->setContextPointer(&sendState.receiver);
	scheduleAt(simTime() + entry.calcTime, sendState.sendDataEvt); // only can be cancelled in onRouteUnreachable()

	EV << "receiver: " << receiver << ", total bytes: " << entry.totalBytes << ", calc time: " << entry.calcTime << "\n";

	PaombrRtEntry *rt = lookupRoutingEntry(receiver);
	if (rt == nullptr)
	{
		EV << "create an entry for the forward route.\n";
		rt = insertRoutingEntry(receiver);
	}
	rt->connected = true;

	ASSERT(rt->flags != RTF_IN_REPAIR);
	if (rt->flags == RTF_DOWN)
		sendRREQ(receiver);
	else // rt->flags == RTF_UP || rt-flags == RTF_IN_PLRR
	{
		ASSERT(rt->lastHopCount != INFINITE_HOPS);
		onRouteReachable(receiver);
	}
}

void PAOMBR::printRoutingTable()
{
	EV << "display my routing table:\n";
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
	{
		EV << "  routing path to node[" << itRT->first << "] lists as follows:\n";
		std::string paths = itRT->second->pathPrint();
		EV << paths.c_str() << "  source I help lists as follows:";
		std::string revPaths = itRT->second->revPathPrint();
		EV << revPaths.c_str() << std::endl;
	}
}

//////////////////////////////    routing table functions    //////////////////////////////
PAOMBR::PaombrRtEntry* PAOMBR::insertRoutingEntry(LAddress::L3Type dest)
{
	PaombrRtEntry *entry = new PaombrRtEntry(this, dest);
	routingTable.insert(std::pair<LAddress::L3Type, PaombrRtEntry*>(dest, entry));
	return entry;
}

PAOMBR::PaombrRtEntry* PAOMBR::lookupRoutingEntry(LAddress::L3Type dest)
{
	itRT = routingTable.find(dest);
	return itRT != routingTable.end() ? itRT->second : nullptr;
}

void PAOMBR::downRoutingEntry(PaombrRtEntry *rt)
{
	if (rt->flags == RTF_DOWN)
		return;
	EV << "down routing entry, dest: " << rt->dest << ", broken nb: " << rt->brokenNb << std::endl;
	rt->connected = false;
	rt->flags = RTF_DOWN;
	rt->advertisedHops = INFINITE_HOPS;
	ASSERT(rt->pathList.empty());
	rt->expireAt = simTime() + DELETE_PERIOD;
}

void PAOMBR::purgeRoutingTable()
{
	for (itRT = routingTable.begin(); itRT != routingTable.end();)
	{
		PaombrRtEntry *rt = itRT->second;
		if (rt->flags == RTF_UP)
		{
			ASSERT(rt->lastHopCount != INFINITE_HOPS);
			// if a valid route has expired, invalidate the route.
			rt->pathPurge();
			// when all links from a node i leading to a destination d break, then node i
			// locally increments the seqno_i^d and sets the advertised_hop_count_i^d to INFINITE_HOPS.
			if (rt->pathList.empty())
			{
				EV << "RTF_UP, but all paths are purged.\n";

				++rt->seqno;
				if (rt->seqno < rt->highestSeqnoHeard)
					rt->seqno = rt->highestSeqnoHeard;
				if (rt->seqno % 2 == 0)
					++rt->seqno;
				downRoutingEntry(rt);
				if (senderStates.find(itRT->first) != senderStates.end())
					sendRREQ(itRT->first);
			}
			++itRT;
		}
		else if (rt->flags == RTF_DOWN)
		{
			if (rt->expireAt <= simTime())
			{
				EV << "RTF_DOWN, and expired, dest: " << itRT->first << std::endl;
				onRouteUnreachable(itRT->first);
				delete itRT->second;
				routingTable.erase(itRT++);
			}
			else
			{
				EV << "RTF_DOWN, not expired, dest: " << itRT->first << std::endl;
				// If the route is down and if there is a packet for this destination waiting in
				// the send buffer, then send out route request.
				if (findBufferQueue(itRT->first))
					sendRREQ(itRT->first);
				++itRT;
			}
		}
		else // rt->flags == RTF_IN_REPAIR || rt->flags == RTF_IN_PLRR
			++itRT;
	}
}

//////////////////////////////    BCAST_ID functions    //////////////////////////////
PAOMBR::PaombrBroadcastID* PAOMBR::insertBroadcastID(LAddress::L3Type src, uint32_t bid)
{
	for (itBC = broadcastCache.begin(); itBC != broadcastCache.end(); ++itBC)
	{
		if (itBC->source == src && itBC->rreqId == bid)
		{
			++itBC->count;
			itBC->expireAt = simTime() + BCAST_ID_SAVE;
			return &*itBC;
		}
	}

	broadcastCache.push_front(PaombrBroadcastID(src, bid));
	return &broadcastCache.front();
}

PAOMBR::PaombrBroadcastID* PAOMBR::lookupBroadcastID(LAddress::L3Type src, uint32_t bid)
{
	for (itBC = broadcastCache.begin(); itBC != broadcastCache.end(); ++itBC)
		if (itBC->source == src && itBC->rreqId == bid)
			return &*itBC;
	return nullptr;
}

void PAOMBR::purgeBroadcastCache()
{
	simtime_t curTime = simTime();
	for (itBC = broadcastCache.begin(); itBC != broadcastCache.end();)
	{
		if (itBC->expireAt <= curTime)
			itBC = broadcastCache.erase(itBC);
		else
			++itBC;
	}
}

simtime_t PAOMBR::calcLET(const Coord& nbPos, const Coord& nbSpeed)
{
	if (nbSpeed == curSpeed)
		return SimTime(3600, SIMTIME_S);
	Coord p = nbPos - curPosition, v = nbSpeed - curSpeed;
	double v2 = v.squareLength(), R2 = transmissionRadius*transmissionRadius;
	double xy = square(v.x*p.y + v.y*p.x), yz = square(v.y*p.z + v.z*p.y), zx = square(v.z*p.x + v.x*p.z);
	return (sqrt(v2*R2 - xy - yz - zx) - (v.x*p.x + v.y*p.y + v.z*p.z)) / v2;
}

void PAOMBR::transferBufPackets(LAddress::L3Type dest)
{
	for (itRQ = rQueue.begin(); itRQ != rQueue.end();)
	{
		if ((*itRQ)->getDestination() == dest)
		{
			pushBufferQueue(*itRQ);
			itRQ = rQueue.erase(itRQ);
		}
		else
			++itRQ;
	}
	if (!sendBufDataEvt->isScheduled() && !bQueue.empty())
		scheduleAt(simTime(), sendBufDataEvt);
}

#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
bool PAOMBR::recvPowerCritical(std::list<double>& recvPowers)
{
	int pointsNum = static_cast<int>(recvPowers.size());
	if (pointsNum < 3)
		return true;
	std::vector<double> y; // for convenience
	if (pointsNum % 2 == 0)
	{
		std::list<double>::iterator it = recvPowers.begin();
		y.assign(++it, recvPowers.end());
		--pointsNum;
	}
	else
		y.assign(recvPowers.begin(), recvPowers.end());

	bool critical = false;
	int polyDegree = (pointsNum - 1) / 2;
	if (polyDegree == 4)
	{
		double a40 = (15*(y[0]+y[8]) - 55*(y[1]+y[7]) + 30*(y[2]+y[6]) + 135*(y[3]+y[5]) + 179*y[4]) / 429;
		double a41 = (86*y[0] - 142*y[1] - 193*y[2] - 126*y[3] + 126*y[5] + 193*y[6] + 142*y[7] - 86*y[8]) / 1188;
		double a42 = (-126*(y[0]+y[8]) + 371*(y[1]+y[7]) + 151*(y[2]+y[6]) - 211*(y[3]+y[5]) - 370*y[4]) / 1716;
		double a43 = (-14*y[0] + 7*y[1] + 13*y[2] + 9*y[3] - 9*y[5] - 13*y[6] - 7*y[7] + 14*y[8]) / 198;
		double a44 = (14*(y[0]+y[8]) - 21*(y[1]+y[7]) - 11*(y[2]+y[6]) + 9*(y[3]+y[5]) + 18*y[4]) / 143;
		double b40 = a40, b41 = a41, b42 = a42 / 2, b43 = a43 / 6, b44 = a44 / 24;
		double x5 = b40 + 5*b41 + 25*b42 + 125*b43 + 625*b44;
		double x6 = b40 + 6*b41 + 36*b42 + 216*b43 + 1296*b44;
		double x7 = b40 + 7*b41 + 49*b42 + 343*b43 + 2401*b44;
		EV_DEBUG << y[0] << ',' << y[1] << ',' << y[2] << ',' << y[3] << ',' << y[4] << ',' << y[5] << ',' << y[6] << ',' << y[7] << ',' << y[8] << " -> ";
		EV_DEBUG << x5 << ',' << x6 << ',' << x7 << "\n";
		critical = x5 < recvPowerThres_dBm && x6 < recvPowerThres_dBm && x7 < recvPowerThres_dBm;
	}
	else if (polyDegree == 3)
	{
		double a30 = (-2*y[0] + 3*y[1] + 6*y[2] + 7*y[3] + 6*y[4] + 3*y[5] - 2*y[6]) / 21;
		double a31 = (22*y[0] -67*y[1] -58*y[2] +58*y[4] +67*y[5] -22*y[6]) / 252;
		double a32 = ( 5*y[0] - 3*y[2] - 4*y[3] - 3*y[4] + 5*y[6]) / 42;
		double a33 = (  -y[0] +   y[1] +   y[2] -   y[4] -   y[5] +   y[6]) / 6;
		double b30 = a30, b31 = a31, b32 = a32 / 2, b33 = a33 / 6;
		double x4 = b30 + 4*b31 + 16*b32 + 64*b33;
		double x5 = b30 + 5*b31 + 25*b32 + 125*b33;
		double x6 = b30 + 6*b31 + 36*b32 + 216*b33;
		EV_DEBUG << y[0] << ',' << y[1] << ',' << y[2] << ',' << y[3] << ',' << y[4] << ',' << y[5] << ',' << y[6] << " -> ";
		EV_DEBUG << x4 << ',' << x5 << ',' << x6 << "\n";
		critical = x4 < recvPowerThres_dBm && x5 < recvPowerThres_dBm && x6 < recvPowerThres_dBm;
	}
	else if (polyDegree == 2)
	{
		double a20 = (-3*y[0] + 12*y[1] + 17*y[2] + 12*y[3] - 3*y[4]) / 35;
		double a21 = (-2*y[0] -    y[1] +    y[3] +  2*y[4]) / 10;
		double a22 = ( 2*y[0] -    y[1] -  2*y[2] -    y[3] + 2*y[4]) / 7;
		double b20 = a20, b21 = a21, b22 = a22 / 2;
		double x3 = b20 + 3*b21 + 9*b22;
		double x4 = b20 + 4*b21 + 16*b22;
		double x5 = b20 + 5*b21 + 25*b22;
		EV_DEBUG << y[0] << ',' << y[1] << ',' << y[2] << ',' << y[3] << ',' << y[4] << " -> " << x3 << ',' << x4 << ',' << x5 << "\n";
		critical = x3 < recvPowerThres_dBm && x4 < recvPowerThres_dBm && x5 < recvPowerThres_dBm;
	}
	else // linear fitting
	{
		double b10 = y[1], b11 = (-y[0] + y[2]) / 2;
		double x2 = b10 + 2*b11, x3 = b10 + 3*b11, x4 = b10 + 4*b11;
		EV_DEBUG << y[0] << ',' << y[1] << ',' << y[2] << " -> " << x2 << ',' << x3 << ',' << x4 << "\n";
		critical = x2 < recvPowerThres_dBm && x3 < recvPowerThres_dBm && x4 < recvPowerThres_dBm;
	}
	return critical;
}
#endif

//////////////////////////////    PAOMBR::PaombrRtEntry    //////////////////////////////
PAOMBR::PaombrRtEntry::PaombrRtEntry(PAOMBR *o, LAddress::L3Type d) : owner(o), error(false), connected(false), expireAt(simTime()+DELETE_PERIOD),
		reqTimeout(), reqCount(0), reqLastTTL(0), flags(RTF_DOWN), advertisedHops(INFINITE_HOPS), lastHopCount(INFINITE_HOPS),
		seqno(0), highestSeqnoHeard(0), bufPktsNum(0), dest(d), brokenNb(-1)
{
	rrepTimeoutEvt = rrepAckTimeoutEvt = nullptr;
}

PAOMBR::PaombrRtEntry::~PaombrRtEntry()
{
	owner->cancelAndDelete(rrepTimeoutEvt);
	owner->cancelAndDelete(rrepAckTimeoutEvt);
}

PAOMBR::PaombrPath* PAOMBR::PaombrRtEntry::pathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop, uint8_t hopCount, simtime_t expire, simtime_t PET)
{
	simtime_t _expireAt = simTime() + expire;
	PAOMBR::PaombrPath path(nextHop, lastHop, hopCount, _expireAt, simTime()+PET);
	pathList.push_front(path);
	itSP = pathList.begin();
	if (expireAt < _expireAt)
		expireAt = _expireAt;
	return &pathList.front();
}

PAOMBR::PaombrPath* PAOMBR::PaombrRtEntry::pathSelect(SendState *pss)
{
	if (pss == nullptr)
	{
		if (++itSP == pathList.end()) // round robin all disjoint paths
			itSP = pathList.begin();
		return &*itSP;
	}
	else
	{
		double highestDeliverRatio = -1.0;
		PaombrPath *path = nullptr;
		for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		{
			double deliverRatio = pss->states[itPL->lastHop].deliverRatio;
			if (highestDeliverRatio < deliverRatio)
			{
				highestDeliverRatio = deliverRatio;
				path = &*itPL;
			}
		}
		return path;
	}
}

PAOMBR::PaombrPath* PAOMBR::PaombrRtEntry::pathLookup(LAddress::L3Type nextHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->nextHop == nextHop)
			return &*itPL;
	return nullptr;
}

PAOMBR::PaombrPath* PAOMBR::PaombrRtEntry::pathLookupLastHop(LAddress::L3Type lastHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->lastHop == lastHop)
			return &*itPL;
	return nullptr;
}

PAOMBR::PaombrPath* PAOMBR::PaombrRtEntry::pathLookupMinHop()
{
	uint8_t minHopCount = INFINITE_HOPS;
	PAOMBR::PaombrPath *path = nullptr;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
	{
		if (minHopCount > itPL->hopCount)
		{
			minHopCount = itPL->hopCount;
			path = &*itPL;
		}
	}
	return path;
}

PAOMBR::PaombrPath* PAOMBR::PaombrRtEntry::disjointPathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->nextHop == nextHop && itPL->lastHop == lastHop)
			return &*itPL;
	return nullptr;
}

bool PAOMBR::PaombrRtEntry::disjointPathExists(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->nextHop == nextHop || itPL->lastHop == lastHop)
			return false;
	return true;
}

void PAOMBR::PaombrRtEntry::pathDelete(LAddress::L3Type nextHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
	{
		if (itPL->nextHop == nextHop)
		{
			bool willInvalidate = itPL == itSP;
			itPL = pathList.erase(itPL);
			if (willInvalidate) // avoid invalidating iterator itSP
				itSP = itPL;
			break;
		}
	}
}

void PAOMBR::PaombrRtEntry::pathPurge()
{
	simtime_t curTime = simTime();
	for (itPL = pathList.begin(); itPL != pathList.end();)
	{
		if (itPL->expireAt <= curTime)
		{
			bool willInvalidate = itPL == itSP;
			itPL = pathList.erase(itPL);
			if (willInvalidate) // avoid invalidating iterator itSP
				itSP = itPL;
		}
		else
			++itPL;
	}
}

std::string PAOMBR::PaombrRtEntry::pathPrint()
{
	const char *ws = "        ";
	std::ostringstream oss;
	oss << "    nextHop  lastHop  hopCount    expireAt    PET\n";
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		oss << "      " << itPL->nextHop << ws << itPL->lastHop << ws << (uint32_t)itPL->hopCount << ws << itPL->expireAt.dbl() << "    " << itPL->PET.dbl() << "\n";
	return oss.str();
}

uint8_t PAOMBR::PaombrRtEntry::pathGetMinHopCount()
{
	uint8_t minHopCount = INFINITE_HOPS;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (minHopCount > itPL->hopCount)
			minHopCount = itPL->hopCount;
	return minHopCount;
}

uint8_t PAOMBR::PaombrRtEntry::pathGetMaxHopCount()
{
	uint8_t maxHopCount = 0;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (maxHopCount < itPL->hopCount)
			maxHopCount = itPL->hopCount;
	return maxHopCount;
}

simtime_t PAOMBR::PaombrRtEntry::pathGetMaxExpirationTime()
{
	simtime_t maxExpirationTime = SimTime::ZERO;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (maxExpirationTime < itPL->expireAt)
			maxExpirationTime = itPL->expireAt;
	return maxExpirationTime;
}

simtime_t PAOMBR::PaombrRtEntry::pathGetMaxPET()
{
	simtime_t maxPET = SimTime::ZERO;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (maxPET < itPL->PET)
			maxPET = itPL->PET;
	return maxPET;
}

bool PAOMBR::PaombrRtEntry::revPathInsert(LAddress::L3Type source, LAddress::L3Type lastHop)
{
	for (itRPL = revPathList.begin(); itRPL != revPathList.end(); ++itRPL)
	{
		if (itRPL->first == source)
		{
			itRPL->second = lastHop;
			return false;
		}
	}
	revPathList.push_front(std::pair<LAddress::L3Type, LAddress::L3Type>(source, lastHop));
	return true;
}

LAddress::L3Type PAOMBR::PaombrRtEntry::revPathLookup(LAddress::L3Type source)
{
	for (itRPL = revPathList.begin(); itRPL != revPathList.end(); ++itRPL)
		if (itRPL->first == source)
			return itRPL->second;
	return -1;
}

void PAOMBR::PaombrRtEntry::revPathDelete(LAddress::L3Type source)
{
	for (itRPL = revPathList.begin(); itRPL != revPathList.end(); ++itRPL)
		if (itRPL->first == source)
			break;
	if (itRPL != revPathList.end())
		itRPL = revPathList.erase(itRPL);
}

std::string PAOMBR::PaombrRtEntry::revPathPrint()
{
	std::ostringstream oss;
	for (itRPL = revPathList.begin(); itRPL != revPathList.end(); ++itRPL)
		oss << " (" << itRPL->first << ',' << itRPL->second << ')';
	return oss.str();
}

#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
//////////////////////////////    PAOMBR::PaombrNeighborInfo    //////////////////////////////
PAOMBR::PaombrNeighborInfo::PaombrNeighborInfo(Coord& p, Coord& s, LAddress::L2Type ma, simtime_t ra, LAddress::L3Type _nb)
		: BaseWaveApplLayer::NeighborInfo(p, s, ma, ra), nb(_nb), LET(), PLRRTimeout(), PLRRSuppressUntil()
{
	LETTimeoutEvt = new cMessage("LET timeout evt", PAOMBRMsgKinds::LET_TIMEOUT_EVT);
	LETTimeoutEvt->setContextPointer(&nb);
	PLRRTimeoutEvt = nullptr;
}
#endif
