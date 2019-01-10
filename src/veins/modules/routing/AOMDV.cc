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
#include "veins/modules/routing/AOMDV.h"

Define_Module(AOMDV);

void AOMDV::initialize(int stage)
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
		bQueueSize = 0;
		bQueueCap = par("bufferQueueCap").longValue();

		pktTransmitDelay = SimTime(dataLengthBits/6, SIMTIME_US); // assume link layer rate is 6Mb/s
		pktNetLayerDelay = SimTime(par("pktNetLayerDelay").longValue(), SIMTIME_US);
		pktApplLayerDelay = SimTime(par("pktApplLayerDelay").longValue(), SIMTIME_US);

		callRoutings = par("callRoutings").boolValue();
		if (callRoutings)
		{
			initializeRoutingPlanList(par("routingPlan").xmlValue(), myAddr);
#ifdef TEST_ROUTE_REPAIR_PROTOCOL
			initializeTriggerPlanList(par("triggerPlan").xmlValue());
#endif
		}

		if (sendBeacons)
		{
			if (!routingPlanList.empty())
			{
				callRoutingEvt = new cMessage("call routing evt", WaveApplMsgKinds::CALL_ROUTING_EVT);
				scheduleAt(routingPlanList.front().first, callRoutingEvt);
			}
			sendBufDataEvt = new cMessage("send buf data evt", AOMDVMsgKinds::SEND_BUF_DATA_EVT);
			sendBufDataEvt->setSchedulingPriority(1);
			purgeRoutingTableEvt = new cMessage("purge routing table evt", AOMDVMsgKinds::PURGE_ROUTING_TABLE_EVT);
			purgeBroadcastCacheEvt = new cMessage("purge broadcast cache evt", AOMDVMsgKinds::PURGE_BROADCAST_CACHE_EVT);
#ifdef TEST_ROUTE_REPAIR_PROTOCOL
			if (!triggerPlanList.empty())
			{
				callTriggerEvt = new cMessage("call trigger evt", AOMDVMsgKinds::CALL_TRIGGER_EVT);
				scheduleAt(triggerPlanList.front().first, callTriggerEvt);
			}
			else
				callTriggerEvt = nullptr;
#endif
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

void AOMDV::finish()
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

#ifdef TEST_ROUTE_REPAIR_PROTOCOL
	triggerPlanList.clear();
	cancelAndDelete(callTriggerEvt);
#endif
	cancelAndDelete(callRoutingEvt);
	cancelAndDelete(sendBufDataEvt);
	cancelAndDelete(purgeRoutingTableEvt);
	cancelAndDelete(purgeBroadcastCacheEvt);

	BaseWaveApplLayer::finish();
}

void AOMDV::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case AOMDVMsgKinds::PURGE_ROUTING_TABLE_EVT:
	{
		purgeRoutingTable();
		scheduleAt(simTime() + PURGE_ROUTE_PERIOD, purgeRoutingTableEvt);
		break;
	}
	case AOMDVMsgKinds::PURGE_BROADCAST_CACHE_EVT:
	{
		purgeBroadcastCache();
		scheduleAt(simTime() + PURGE_BCAST_ID_PERIOD, purgeBroadcastCacheEvt);
		break;
	}
	case AOMDVMsgKinds::RREP_TIMEOUT_EVT:
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
	case AOMDVMsgKinds::RREPACK_TIMEOUT_EVT:
	{
		break;
	}
	case AOMDVMsgKinds::LOCAL_REPAIR_TIMEOUT_EVT:
	{

		break;
	}
	case AOMDVMsgKinds::SEND_DATA_EVT:
	{
		simtime_t moment = pktApplLayerDelay;
		if (bQueueSize == bQueueCap)
		{
			EV << "buffer queue is full, wait a moment: " << moment << std::endl;
			scheduleAt(simTime() + moment, msg);
			break;
		}

		LAddress::L3Type *dest = static_cast<LAddress::L3Type*>(msg->getContextPointer());

		AomdvRtEntry *rt = lookupRoutingEntry(*dest);
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
			EV << "RTF_UP, prepare send it to next hop: " << rt->pathSelect()->nextHop << ", queue size: " << bQueueSize << std::endl;

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
	case AOMDVMsgKinds::SEND_BUF_DATA_EVT:
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

		AomdvRtEntry *rt = lookupRoutingEntry(dataPkt->getDestination());
		ASSERT(rt != nullptr);
		if (rt->flags != RTF_UP)
		{
			EV << "RTF_DOWN || RTF_IN_REPAIR, push it into rqueue." << std::endl;
			rQueue.push_back(dataPkt);
			scheduleAt(simTime() + moment, msg);
			break;
		}

		LAddress::L3Type nextHop = rt->pathSelect()->nextHop;
		dataPkt->setSenderAddress(myAddr);
#ifdef USE_L2_UNICAST_DATA
		dataPkt->setRecipientAddress(lookupL2Address(nextHop));
#else
		dataPkt->setReceiverAddress(nextHop);
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
		}
		sendDelayedDown(dataPkt, moment);
		// trick: simulate packet lost without MAC ACK
		Coord &nextHopPos = MobilityObserver::Instance()->globalPosition[nextHop];
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
#ifdef TEST_ROUTE_REPAIR_PROTOCOL
	case AOMDVMsgKinds::CALL_TRIGGER_EVT:
	{
		callTrigger(triggerPlanList.front().second);
		triggerPlanList.pop_front();
		if (!triggerPlanList.empty())
			scheduleAt(triggerPlanList.front().first, callTriggerEvt);
		break;
	}
#endif
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

void AOMDV::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "routing") == 0)
		DYNAMIC_CAST_CMESSAGE(Routing, routing)
	else if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseWaveApplLayer::handleLowerMsg(msg);
}

void AOMDV::handleLowerControl(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		onDataLost(dynamic_cast<DataMessage*>(msg));
}

//////////////////////////////    RX handlers    //////////////////////////////
void AOMDV::onBeacon(BeaconMessage *beaconMsg)
{
	BaseWaveApplLayer::onBeacon(beaconMsg);

	// Whenever a node receives a Hello message from a neighbor, the node SHOULD make sure
	// that it has an active route to the neighbor, and	create one if necessary. If a route
	// already exists, then the	Lifetime for the route should be increased, if necessary,
	// to be at	least ALLOWED_HELLO_LOSS * HELLO_INTERVAL. The route to the	neighbor, if it
	// exists, MUST subsequently contain the latest	Destination Sequence Number from the
	// Hello message. The current node can now begin using this route to forward data packets.
	// Routes that are created by hello messages and not used by any other active routes
	// will have empty precursor lists and would not trigger a RERR message	if the neighbor
	// moves away and a neighbor timeout occurs.
	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias
	insertL2L3Address(beaconMsg->getRecipientAddress(), sender);
	AomdvRtEntry *rt = lookupRoutingEntry(sender);
	if (rt == nullptr)
	{
		EV << "create an entry for the neighbor " << sender << "\n";
		rt = insertRoutingEntry(sender);
		rt->flags = RTF_UP;
		rt->advertisedHops = 1;
		rt->lastHopCount = 1;
		rt->dest = sender;
	}

	AomdvPath *path = rt->pathLookup(sender);
	if (path != nullptr)
		path->expireAt = simTime() + neighborElapsed;
	else
		path = rt->pathInsert(sender, myAddr, 1, neighborElapsed);
}

void AOMDV::onRouting(RoutingMessage *msg)
{
	// EV << "node[" << myAddr << "]: onRouting!\n";

	uint8_t ttl = msg->getTTL();
	msg->setTTL(--ttl);

	EV << "TTL: " << (uint32_t)ttl << "\n";

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
	case AOMDVPacketType::RERR:
	{
		DYNAMIC_CAST_CMESSAGE(RERR, rerr)
		break;
	}
	case AOMDVPacketType::RREPACK:
	{
		onRREPACK(msg);
		break;
	}
	default:
		throw cRuntimeError("receive a AOMDV control packet with invalid packet type");
	}
}

/**
 * 3.2.2. Route discovery
 * As in AODV, when a traffic source needs a route to a destination, the source initiates
 * a route discovery process by generating a RREQ. Since the RREQ is flooded network-wide,
 * a node may receive several copies of the same RREQ. In AODV, only the first copy of the RREQ
 * is used to form reverse paths; the duplicate copies that arrive later are simply discarded.
 * Note that some of these duplicate copies can be gainfully used to form alternate reverse paths.
 * Thus, all duplicate copies are examined in AOMDV for potential alternate reverse paths,
 * but reverse paths are formed only using those copies that preserve loop-freedom and
 * disjointness among the resulting set of paths to the source.
 * When an intermediate node obtains a reverse path via a RREQ copy, it checks whether there are
 * one or more valid forward paths to the destination. If so, the node generates a RREP and sends
 * it back to the source along the reverse path; the RREP includes a forward path that was not
 * used in any previous RREPs for this route discovery. In this case, the intermediate node does not
 * propagate the RREQ further. Otherwise, the node re-broadcasts the RREQ copy if it has not
 * previously forwarded any other copy of this RREQ and this copy resulted in the formation/updation
 * of a reverse path.
 * When the destination receives RREQ copies, it also forms reverse paths in the same way as
 * intermediate nodes. However, it adopts a somewhat 'looser' policy for generating a RREP.
 * Specifically, the destination generates a RREP in response to every RREQ copy that arrives via
 * a loop-free path to the source even though it forms reverse paths using only RREQ copies that
 * arrive via loop-free and disjoint alternate paths to the source. The reason behind the looser
 * RREP generation policy at the destination is as follows. The RREQ flooding mechanism, where
 * each node locally broadcasts a RREQ once, suppresses some RREQ copies at intermediate nodes
 * and duplicates other RREQ copies. We call this the 'route cutoff' problem. Clearly, the route
 * cutoff problem prevents the discovery of all disjoint reverse paths. This in turn would severely
 * limit the number of disjoint forward paths found at the source if the destination sends RREPs
 * only along disjoint reverse paths. Therefore, we let the destination send back a RREP along each
 * loop-free reverse path even though it is not disjoint with previously established reverse paths.
 * Such additional RREPs alleviate the route cutoff problem and increase the possibility of finding
 * more disjoint forward paths.
 */
void AOMDV::onRREQ(RREQMessage *rq)
{
	EV << "node[" << myAddr << "]: onRREQ!\n";

	LAddress::L3Type ipsrc = rq->getSenderAddress(); // alias
	LAddress::L3Type rqsrc = rq->getOriginatorAddr(), rqdst = rq->getDestAddr(); // alias
	EV << "RREQ src: " << rqsrc << ", RREQ dest: " << rqdst << "\n";
	// Drop if:
	//      - I'm the source
	//      - I recently heard this request.
	if (rqsrc == myAddr)
	{
		EV << "This is my own RREQ, discard it." << std::endl;
		return;
	}

	bool propagation = true;
	// If RREQ has already been received - drop it, else remember "RREQ id" <src IP, bcast ID>.
	AomdvBroadcastID *b = lookupBroadcastID(rqsrc, rq->getRreqId());
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
	AomdvRtEntry *rt0 = lookupRoutingEntry(rqsrc);
	if (rt0 == nullptr) // create an entry for the reverse route.
	{
		EV << "create an entry for the reverse route.\n";
		rt0 = insertRoutingEntry(rqsrc);
	}
	rt0->connected = true;

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
	 * To maintain multiple paths for the same sequence number, AOMDV uses the notion of an 'advertised
	 * hop count.' Every node maintains a variable called advertised hop count for each destination.
	 * This variable is set to the length of the 'longest' available path for the destination at the time
	 * of first advertisement for a particular destination sequence number. The advertised hop count
	 * remains unchanged until the sequence number changes. Advertising the longest path length permits
	 * more number of alternate paths to be maintained.
	 */
	AomdvPath *reversePath = nullptr;
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
		reversePath = rt0->pathInsert(ipsrc, rq->getFirstHop(), rq->getHopCount()+1, REVERSE_ROUTE_LIFE);
		rt0->lastHopCount = rq->getHopCount() + 1;
		EV << "new reverse path: " << reversePath->info() << ", last hop count: " << (uint32_t)rt0->lastHopCount << std::endl;
	}
	// If a new path with smaller hop count is received
	// (same seqno, better hop count) - try to insert new path in route table.
	else if (rt0->seqno == rq->getOriginatorSeqno() && rt0->advertisedHops > rq->getHopCount())
	{
		ASSERT(rt0->flags == RTF_UP); // Make sure path is up

		EV << "Case II: get useful route (" << (uint32_t)rt0->advertisedHops << " >= " << rqHopCount << ").\n";

		AomdvPath *erp = nullptr;
		// If path already exists - adjust the lifetime of the path.
		if ((reversePath = rt0->disjointPathLookup(ipsrc, rq->getFirstHop())) != nullptr)
		{
			ASSERT(reversePath->hopCount == rq->getHopCount() + 1);
			reversePath->expireAt = simTime() + REVERSE_ROUTE_LIFE;
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
			// and new path does not differ too much in length compared to previous paths
			if (rt0->pathList.size() < AOMDV_MAX_PATHS && rq->getHopCount() + 1 - rt0->pathGetMinHopCount() <= AOMDV_MAX_PATH_HOP_DIFF)
			{
				reversePath = rt0->pathInsert(ipsrc, rq->getFirstHop(), rq->getHopCount()+1, REVERSE_ROUTE_LIFE);
				rt0->lastHopCount = rt0->pathGetMaxHopCount();
				EV << "new reverse path: " << reversePath->info() << ", last hop count: " << (uint32_t)rt0->lastHopCount << std::endl;
			}
			// If new path differs too much in length compared to previous paths - drop packet.
			if (rq->getHopCount() + 1 - rt0->pathGetMinHopCount() > AOMDV_MAX_PATH_HOP_DIFF)
				return;
		}
		// (RREQ was intended for me) AND
		// ((Path with RREQ first hop as last hop does not exist) OR (The path exists and has less hop count than RREQ)) - drop packet.
		else if (rqdst == myAddr && ((erp = rt0->pathLookupLastHop(rq->getFirstHop())) == nullptr || rq->getHopCount() + 1 > erp->hopCount))
			return;
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

	// Check route entry for RREQ destination
	AomdvRtEntry *rt = lookupRoutingEntry(rqdst);
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
		if (reversePath != nullptr)
		{
			EV << "I have a fresh route to RREQ dest, b->count: " << b->count << " (" << rt->seqno << " >= " << rq->getDestSeqno() << ')' << std::endl;
#ifdef AOMDV_NODE_DISJOINT_PATHS
			if (b->count == 0)
			{
				b->count = 1;

				// route advertisement
				if (rt->advertisedHops == INFINITE_HOPS)
					rt->advertisedHops = rt->pathGetMaxHopCount();

				AomdvPath *forwardPath = rt->pathSelect();
				rt->error = true;
				rt->connected = true;
				sendRREP(rq, rt->advertisedHops, rt->seqno, forwardPath->expireAt-simTime(), forwardPath->lastHop);
			}
#endif
		}
	}
	// RREQ not intended for me and I don't have a fresh enough entry for RREQ dest - so forward the RREQ
	else
	{
		if (propagation) // do not propagate a duplicate RREQ
		{
			// Maximum sequence number seen in route
			if (rt != nullptr && rq->getDestSeqno() < rt->seqno)
				rq->setDestSeqno(rt->seqno);

			// route advertisement
			if (rt0->advertisedHops == INFINITE_HOPS)
				rt0->advertisedHops = rt0->pathGetMaxHopCount();
			rq->setHopCount(rt0->advertisedHops);
#ifdef AOMDV_NODE_DISJOINT_PATHS
			reversePath = rt0->pathSelect();
			rq->setFirstHop(reversePath->lastHop);
#endif
			EV << "forward RREQ, dest seqno: " << rq->getDestSeqno() << ", hop count: " << (uint32_t)rq->getHopCount() << std::endl;
			forward(nullptr, rq->dup());
		}
	}
}

/**
 * 3.2.2. Route discovery
 * When an intermediate node receives a RREP, it follows route update rules in to form a loop-free
 * and disjoint forward path to the destination, if possible; else, the RREP is dropped. Supposing
 * that the intermediate node forms the forward path and has one or more valid reverse paths to the
 * source, it checks if any of those reverse paths was not previously used to send a RREP for this
 * route discovery. If so, it chooses one of those unused reverse paths to forward the current RREP;
 * otherwise, the RREP is simply dropped. Note that our choice of forwarding the RREP along a unique
 * reverse path, as opposed to duplicating it along all available reverse paths, does not hurt AOMDV
 * route discovery latency. This is because the latency of a route discovery is determined by the
 * amount of time source has to wait before it obtains the first route, and RREPs in AOMDV (as with
 * AODV) use fairly reliable ARQ-based unicast MAC layer transmissions. On the contrary, duplicating
 * the RREP will cause a route cutoff problem similar to that mentioned above, reducing the number
 * of disjoint paths found at the source.
 */
void AOMDV::onRREP(RREPMessage *rp)
{
	EV << "node[" << myAddr << "]: onRREP!\n";

	LAddress::L3Type ipsrc = rp->getSenderAddress(), ipdst = rp->getIpDest(); // alias
	LAddress::L3Type rpdst = rp->getDestAddr(); // alias
	EV << "IP src: " << ipsrc << ", IP dest: " << ipdst << ", RREP dest: " << rpdst <<  "\n";
	// If I receive a RREP with myself as source - drop packet (should not occur).
	// rpdst is the source of the RREP, or rather the destination of the RREQ.
	if (rpdst == myAddr)
		return;

	// Got a reply. So reset the "soft state" maintained for route requests in the request table.
	// We don't really have have a separate request table. It is just a part of the routing table itself.
	AomdvRtEntry *rt = lookupRoutingEntry(rpdst);
	// If I don't have a rt entry to this host... adding
	if (rt == nullptr)
	{
		EV << "create an entry for the forward route.\n";
		rt = insertRoutingEntry(rpdst);
	}
	rt->connected = true;

	AomdvPath *forwardPath = nullptr;
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
		forwardPath = rt->pathInsert(ipsrc, rp->getFirstHop(), rp->getHopCount()+1, rp->getLifeTime());
		rt->lastHopCount = rp->getHopCount() + 1;
		EV << "new forward path: " << forwardPath->info() << ", last hop count: " << (uint32_t)rt->lastHopCount << std::endl;
	}
	// If the sequence number in the RREP is the same as for route entry but
	// with a smaller hop count - try to insert new forward path to (RREQ) dest.
	else if (rt->seqno == rp->getDestSeqno() && rt->advertisedHops > rp->getHopCount())
	{
		EV << "Case II: get useful route (" << (uint32_t)rt->advertisedHops << " >= " << rpHopCount << ").\n";

		rt->flags = RTF_UP;
		// If the path already exists - increase path lifetime
		if ((forwardPath = rt->disjointPathLookup(ipsrc, rp->getFirstHop())) != nullptr)
		{
			ASSERT(forwardPath->hopCount == rp->getHopCount() + 1);
			forwardPath->expireAt = simTime() + rp->getLifeTime();
			if (rt->expireAt < forwardPath->expireAt)
				rt->expireAt = forwardPath->expireAt;
			EV << "forward path already exists, update expiration time to " << forwardPath->expireAt << std::endl;
		}
		// If the path does not already exist, there is room for it and
		// it does not differ too much in length - we add the path
		else if (rt->disjointPathExists(ipsrc, rp->getFirstHop()) && rt->pathList.size() < AOMDV_MAX_PATHS
				&& rp->getHopCount() + 1 - rt->pathGetMinHopCount() <= AOMDV_MAX_PATH_HOP_DIFF)
		{
			// Insert forward path to RREQ destination.
			forwardPath = rt->pathInsert(ipsrc, rp->getFirstHop(), rp->getHopCount()+1, rp->getLifeTime());
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
		if (rt->seqno > rp->getDestSeqno())
			EV << "get stale route (" << rt->seqno << " > " << rp->getDestSeqno() << ")." << std::endl;
		else if (rt->advertisedHops < rp->getHopCount())
			EV << "get useless route (" << (uint32_t)rt->advertisedHops << " < " << rpHopCount << ")." << std::endl;
		return;
	}

#if ROUTING_DEBUG_LOG
	printRoutingTable();
#endif

	// If route is up
	if (rt->flags == RTF_UP)
	{
		// Reset the soft state
		rt->reqTimeout = SimTime::ZERO;
		rt->reqCount = 0;
		rt->reqLastTTL = 0;

		// Find out whether any buffered packet can benefit from the forward route.
		EV << "forward path becomes RTF_UP, transfer corresponding packets from rQueue to bQueue.\n";
		transferBufPackets(rpdst);
	}

	// If I am the intended recipient of the RREP, nothing more needs to be done - so drop packet.
	if (ipdst == myAddr)
	{
		EV << "I am the RREP dest, cancel RREP timeout event.\n";

		RoutingStatisticCollector::gRouteAcquisitionTime += simTime() - rt->rrepTimeoutEvt->getSendingTime();
		cancelEvent(rt->rrepTimeoutEvt);
		onRouteReachable(rpdst);
		return;
	}

	// If I am not the intended recipient of the RREP
	// - check route table for a path to the RREP dest (i.e. the RREQ source).
	AomdvRtEntry *rt0 = lookupRoutingEntry(ipdst);
	AomdvBroadcastID *b = lookupBroadcastID(ipdst, rp->getRreqId()); // Check for <RREQ src IP, bcast ID> pair
#ifdef AOMDV_NODE_DISJOINT_PATHS
	if (rt0 == nullptr || rt0->flags != RTF_UP || b == nullptr || b->count > 0)
		return;

	rt0->connected = true;
	b->count = 1;
	AomdvPath *reversePath = rt0->pathSelect();
	reversePath->expireAt = simTime() + ACTIVE_ROUTE_TIMEOUT;
	EV << "I have an active route to RREQ src, reverse path: " << reversePath->info() << "\n";

	// route advertisement
	if (rt->advertisedHops == INFINITE_HOPS)
		rt->advertisedHops = rt->pathGetMaxHopCount();
	rp->setHopCount(rt->advertisedHops);
	forwardPath = rt->pathSelect();
	rp->setFirstHop(forwardPath->lastHop);
	rt->error = true;

	EV << "forward RREP, last hop: " << forwardPath->lastHop << ", hop count: " << (uint32_t)rp->getHopCount() << std::endl;
	forward(rt0, rp->dup());
#endif
}

/**
 * Just before transmitting the RERR, certain updates are made on the routing table that may affect
 * the destination sequence numbers for the unreachable destinations. For each one of these
 * destinations, the corresponding routing table entry is updated as follows:
 * 1. The destination sequence number of this routing entry, if it exists and is valid, is incremented
 *    for cases (i) and (ii) above, and copied from the incoming RERR in case (iii) above.
 * 2. The entry is invalidated by marking the route entry as invalid.
 * 3. The Lifetime field is updated to current time plus DELETE_PERIOD. Before this time, the entry
 *    SHOULD NOT be deleted.
 */
void AOMDV::onRERR(RERRMessage *re)
{
	EV << "node[" << myAddr << "]: onRERR!\n";

	LAddress::L3Type ipsrc = re->getSenderAddress(); // alias
	RERRMessage *nre = new RERRMessage("routing");
	uint8_t destCount = 0;
	// For each unreachable destination
	for (uint8_t i = 0; i < re->getDestCount(); ++i)
	{
		UnreachableNode &unreach = re->getUnreachableNodes(i);
		AomdvRtEntry *rt = lookupRoutingEntry(unreach.addr);
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

void AOMDV::onRREPACK(RoutingMessage *rrepAck)
{
	EV << "node[" << myAddr << "]: onRREPACK!\n";
}

/**
 * 3.2.4. Data packet forwarding
 * For data packet forwarding at a node having multiple paths to a destination, we adopt
 * a simple approach of using a path until it fails and then switch to an alternate path;
 * we use paths in the order of their creation.
 */
void AOMDV::onData(DataMessage *dataPkt)
{
	EV << "node[" << myAddr << "]: onData!\n";

#ifndef USE_L2_UNICAST_DATA
	if (dataPkt->getReceiverAddress() != myAddr)
		return;
#endif

	LAddress::L3Type dest = dataPkt->getDestination(); // alias
	if (dest == myAddr)
	{
		itRS = receiverStates.find(dataPkt->getSource());
		if (itRS == receiverStates.end())
			itRS = receiverStates.insert(std::pair<LAddress::L3Type, RecvState>(dataPkt->getSource(), RecvState())).first;
		RecvState &recvState = itRS->second;
		bool duplicated = recvState.onRecv(dataPkt->getSequence());
		RoutingStatisticCollector::gDataBitsRecv += dataPkt->getBitLength() - headerLength;
		if (!duplicated)
			RoutingStatisticCollector::gDataPktsRecv++;
		else
			RoutingStatisticCollector::gDataPktsDuplicated++;
		RoutingStatisticCollector::gEndtoendDelay += simTime() - dataPkt->getCreationTime();

		EV << "this data packet is for me, seqno: " << dataPkt->getSequence() << ", seqno intervals:\n";
		//for (recvState.itS = recvState.segments.begin(); recvState.itS != recvState.segments.end(); ++recvState.itS)
		//	EV << '[' << recvState.itS->first << ',' << recvState.itS->second << "), ";
		EV << std::endl;
		return;
	}

	AomdvRtEntry *rt = lookupRoutingEntry(dest);
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
	else if (rt->flags == RTF_UP)
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

void AOMDV::onDataLost(DataMessage *lostDataPkt)
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
void AOMDV::forward(AomdvRtEntry *rt, RoutingMessage *pkt)
{
	if (pkt->getTTL() == 0)
	{
		EV << "TTL becomes zero, discard the packet." << std::endl;
		delete pkt;
		return;
	}

	if (rt != nullptr) // if it is not a broadcast packet
	{
		ASSERT(rt->flags == RTF_UP);

		AomdvPath *path = rt->pathSelect();
		ASSERT(path != nullptr);
		path->expireAt = simTime() + ACTIVE_ROUTE_TIMEOUT;
		pkt->setRecipientAddress(lookupL2Address(path->nextHop));
	}
	else
		ASSERT(pkt->getIpDest() == LAddress::L3BROADCAST());

	pkt->setSenderAddress(myAddr);

	RoutingStatisticCollector::gCtrlBitsTransmitted += pkt->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	sendWSM(pkt);
}

void AOMDV::sendRREQ(LAddress::L3Type dest, uint8_t TTL)
{
	AomdvRtEntry *rt = lookupRoutingEntry(dest);
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

	// A destination node increments its own sequence number in two circumstances:
	//   - Immediately before a node originates a route discovery, it MUST increment
	//     its own sequence number. This prevents conflicts with previously established
	//     reverse routes towards the originator of a RREQ.
	//   - Immediately before a destination node originates a RREP in response to a RREQ,
	//     it MUST update its own sequence number to the maximum of its current sequence
	//     number and the destination sequence number in the RREQ packet.
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
	// rq->setIpDest(LAddress::L3BROADCAST());
	// rq->setHopCount(0);
	rq->setPacketType(AOMDVPacketType::RREQ);
	rq->setRreqId(++rreqID);
	rq->setOriginatorAddr(myAddr);
	rq->setOriginatorSeqno(seqno);
	rq->setDestAddr(dest);
	rq->setDestSeqno(rt->seqno);

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
		rt->rrepTimeoutEvt = new WaitForRREPMessage("rrep timeout evt", AOMDVMsgKinds::RREP_TIMEOUT_EVT);
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

void AOMDV::sendRREP(RREQMessage *rq, uint8_t hopCount, uint32_t rpseq, simtime_t lifetime, LAddress::L3Type lastHop)
{
	RREPMessage *rp = new RREPMessage("routing");
	LAddress::L3Type ipsrc = rq->getSenderAddress();
	prepareWSM(rp, routingLengthBits+224, t_channel::type_CCH, routingPriority, lookupL2Address(ipsrc));
	rp->setIpDest(rq->getOriginatorAddr());
	rp->setTTL(rq->getHopCount()+1);
	rp->setPacketType(AOMDVPacketType::RREP);
	rp->setHopCount(hopCount);
	rp->setDestAddr(rq->getDestAddr());
	rp->setDestSeqno(rpseq);
	rp->setOriginatorAddr(rq->getOriginatorAddr());
	rp->setLifeTime(lifetime);
	rp->setRreqId(rq->getRreqId());
	rp->setFirstHop(lastHop);

	EV << "send RREP to src " << rp->getIpDest() << ", TTL: " << (uint32_t)rp->getTTL() << ", dest seqno: " << rp->getDestSeqno() << std::endl;

	RoutingStatisticCollector::gCtrlBitsTransmitted += rp->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	RoutingStatisticCollector::gRREPs++;
	sendWSM(rp);
}

void AOMDV::sendRERR(RERRMessage *rerr)
{
	ASSERT(rerr != nullptr);

	int rerrLengthBits = 32 + 64*rerr->getDestCount();
	prepareWSM(rerr, routingLengthBits+rerrLengthBits, t_channel::type_CCH, routingPriority);
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

void AOMDV::sendRREPACK(RREPMessage *rp)
{
	RoutingMessage *rrepAck = new RoutingMessage("RREPACK");
	prepareWSM(rrepAck, routingLengthBits+16, t_channel::type_CCH, routingPriority);
	rrepAck->setIpDest(rp->getDestAddr());
	rrepAck->setTTL(rp->getTTL()+1);
	rrepAck->setPacketType(AOMDVPacketType::RREPACK);

	EV << "send RREPACK to dest " << rrepAck->getIpDest() << ", TTL: " << (uint32_t)rrepAck->getTTL() << std::endl;

	RoutingStatisticCollector::gCtrlBitsTransmitted += rrepAck->getBitLength();
	RoutingStatisticCollector::gCtrlPktsTransmitted++;
	RoutingStatisticCollector::gRREPACKs++;
	sendWSM(rrepAck);
}

void AOMDV::onNeighborLost(LAddress::L3Type neighbor)
{
	seqno += 2; // Set of neighbors changed
	ASSERT(seqno % 2 == 0);

	AomdvPath *path = nullptr;
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
	{
		AomdvRtEntry *rt = itRT->second;
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
				sendRREQ(itRT->first, path->hopCount + LOCAL_ADD_TTL);
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
void AOMDV::localRepair(AomdvRtEntry *rt, DataMessage *dataPkt)
{
	AomdvRtEntry *rt0 = lookupRoutingEntry(dataPkt->getSource());
	ASSERT(rt0 != nullptr && rt0->flags == RTF_UP);

	if (rt->flags == RTF_IN_REPAIR)
		return;
	// mark the route as under repair
	rt->flags = RTF_IN_REPAIR;

	++RoutingStatisticCollector::gLocalRepairs;

	uint8_t MIN_REPAIR_TTL = rt->lastHopCount, halfReverseHop = rt0->lastHopCount/2;
	sendRREQ(dataPkt->getDestination(), std::max(MIN_REPAIR_TTL, halfReverseHop) + LOCAL_ADD_TTL);
}

//////////////////////////////    RouteRepairInterface    //////////////////////////////
/**
 * 3.2.3. Route maintenance
 * Like AODV, AOMDV also uses RERR packets. A node generates or forwards a RERR for a destination
 * when the last path to the destination breaks. AOMDV also includes an optimization to salvage
 * packets forwarded over failed links by re-forwarding them over alternate paths.
 */
void AOMDV::onLinkBroken(LAddress::L3Type neighbor, DataMessage *dataPkt)
{
	EV << "link with neighbor " << neighbor << " is broken when transmitting to " << dataPkt->getDestination() << "\n";

	AomdvRtEntry *rt = lookupRoutingEntry(dataPkt->getDestination());
	ASSERT(rt != nullptr);
	rt->brokenNb = neighbor;
	rt->pathDelete(neighbor);
	// salvage the packet using an alternate path if available.
	if (rt->flags == RTF_UP && !rt->pathList.empty())
	{
		LAddress::L3Type nextHop = rt->pathSelect()->nextHop;
		dataPkt->setSenderAddress(myAddr);
#ifdef USE_L2_UNICAST_DATA
		dataPkt->setRecipientAddress(lookupL2Address(nextHop));
#else
		dataPkt->setReceiverAddress(nextHop);
#endif

		RoutingStatisticCollector::gDataBitsTransmitted += dataPkt->getBitLength() - headerLength;
		RoutingStatisticCollector::gCtrlBitsTransmitted += headerLength;
		RoutingStatisticCollector::gDataPktsTransmitted++;
		sendDelayedDown(dataPkt->dup(), SimTime::ZERO);

		EV << "salvage this data packet to " << nextHop << std::endl;
	}
	else if (rt->lastHopCount <= MAX_REPAIR_TTL)
	{
		if (dataPkt->getSource() != myAddr)
			localRepair(rt, dataPkt);
		else
		{
			rt->flags = RTF_IN_REPAIR;
			sendRREQ(dataPkt->getDestination());
		}
		++RoutingStatisticCollector::gPktsLinkLost;
	}
	else
	{
		rt->pathInsert(neighbor, -1, 0, SimTime::ZERO); // ensure we can enter if block in onLocalRepairFailure()
		onLocalRepairFailure(neighbor);
		++RoutingStatisticCollector::gPktsLinkLost;
	}
}

void AOMDV::onLocalRepairFailure(LAddress::L3Type neighbor)
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
		AomdvRtEntry *rt = itRT->second;
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

void AOMDV::onRouteReachable(LAddress::L3Type dest)
{
	EV << "route to dest " << dest << " becomes reachable, list all paths as follows:\n";

	AomdvRtEntry *rt = lookupRoutingEntry(dest);
	std::string paths = rt->pathPrint();
	EV << paths.c_str() << std::endl;

	if (!sendBufDataEvt->isScheduled())
		scheduleAt(simTime(), sendBufDataEvt);
}

void AOMDV::onRouteUnreachable(LAddress::L3Type dest)
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

void AOMDV::callRouting(const PlanEntry& entry)
{
	EV << "node[" << myAddr << "]: callRouting!\n";

	LAddress::L3Type receiver = entry.receiver;
	itN = neighbors.find(receiver);
	ASSERT(itN != neighbors.end());

	itSS = senderStates.find(receiver);
	ASSERT(itSS == senderStates.end());
	itSS = senderStates.insert(std::pair<LAddress::L3Type, SendState>(receiver, SendState(entry.totalBytes, receiver))).first;
	SendState &sendState = itSS->second;
	sendState.sendDataEvt = new cMessage("send data evt", AOMDVMsgKinds::SEND_DATA_EVT);
	sendState.sendDataEvt->setContextPointer(&sendState.receiver);
	scheduleAt(simTime() + entry.calcTime, sendState.sendDataEvt); // only can be cancelled in onRouteUnreachable()

	EV << "receiver: " << receiver << ", total bytes: " << entry.totalBytes << ", calc time: " << entry.calcTime << "\n";

	AomdvRtEntry *rt = lookupRoutingEntry(receiver);
	if (rt == nullptr)
	{
		EV << "create an entry for the forward route.\n";
		rt = insertRoutingEntry(receiver);
	}
	rt->connected = true;

	ASSERT(rt->flags != RTF_IN_REPAIR);
	if (rt->flags == RTF_DOWN)
		sendRREQ(receiver);
	else // rt->flags == RTF_UP
	{
		ASSERT(rt->lastHopCount != INFINITE_HOPS);
		onRouteReachable(receiver);
	}
}

void AOMDV::printRoutingTable()
{
	EV << "display my routing table:\n";
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
	{
		EV << "  routing path to node[" << itRT->first << "] lists as follows:\n";
		std::string paths = itRT->second->pathPrint();
		EV << paths.c_str();
	}
	EV << std::endl;
}

//////////////////////////////    routing table functions    //////////////////////////////
AOMDV::AomdvRtEntry* AOMDV::insertRoutingEntry(LAddress::L3Type dest)
{
	AomdvRtEntry *entry = new AomdvRtEntry(this, dest);
	routingTable.insert(std::pair<LAddress::L3Type, AomdvRtEntry*>(dest, entry));
	return entry;
}

AOMDV::AomdvRtEntry* AOMDV::lookupRoutingEntry(LAddress::L3Type dest)
{
	itRT = routingTable.find(dest);
	return itRT != routingTable.end() ? itRT->second : nullptr;
}

void AOMDV::downRoutingEntry(AomdvRtEntry *rt)
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

void AOMDV::purgeRoutingTable()
{
	for (itRT = routingTable.begin(); itRT != routingTable.end();)
	{
		AomdvRtEntry *rt = itRT->second;
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
				// the send buffer, then send out route request. sendRequest will check whether
				// it is time to really send out request or not.
				if (findBufferQueue(itRT->first))
					sendRREQ(itRT->first);
				++itRT;
			}
		}
		else // rt->flags == RTF_IN_REPAIR
			++itRT;
	}
}

//////////////////////////////    BCAST_ID functions    //////////////////////////////
AOMDV::AomdvBroadcastID* AOMDV::insertBroadcastID(LAddress::L3Type src, uint32_t bid)
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

	broadcastCache.push_front(AomdvBroadcastID(src, bid));
	return &broadcastCache.front();
}

AOMDV::AomdvBroadcastID* AOMDV::lookupBroadcastID(LAddress::L3Type src, uint32_t bid)
{
	for (itBC = broadcastCache.begin(); itBC != broadcastCache.end(); ++itBC)
		if (itBC->source == src && itBC->rreqId == bid)
			return &*itBC;
	return nullptr;
}

void AOMDV::purgeBroadcastCache()
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

void AOMDV::transferBufPackets(LAddress::L3Type dest)
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

#ifdef TEST_ROUTE_REPAIR_PROTOCOL
void AOMDV::callTrigger(int eventType)
{
	switch (eventType)
	{
	case STOP_SEND_BEACON_EVENT_TYPE:
	{
		EV << "stop sending beacon from now on." << std::endl;
		cancelEvent(sendBeaconEvt);
		break;
	}
	case CONT_SEND_BEACON_EVENT_TYPE:
	{
		EV << "continue sending beacon from now on." << std::endl;
		if (!sendBeaconEvt->isScheduled())
			scheduleAt(simTime(), sendBeaconEvt);
		break;
	}
	default:
		throw cRuntimeError("Unspecified trigger event happens.");
	}
}

void AOMDV::initializeTriggerPlanList(cXMLElement *xmlConfig)
{
	if (xmlConfig == 0)
		throw cRuntimeError("No trigger plan configuration file specified.");

	cXMLElementList planList = xmlConfig->getElementsByTagName("TriggerPlan");

	if (planList.empty())
		EV << "No trigger plan configuration items specified.\n";

	for (cXMLElementList::iterator iter = planList.begin(); iter != planList.end(); ++iter)
	{
		cXMLElement *triggerPlan = *iter;

		const char* name = triggerPlan->getAttribute("type");

		if (atoi(name) == myAddr)
		{
			cXMLElementList parameters = triggerPlan->getElementsByTagName("parameter");

			ASSERT( parameters.size() % 2 == 0 );

			EV << "trigger plan list as follows:\n";

			for (size_t i = 0; i < parameters.size(); i += 2)
			{
				double simtime = 0.0;
				int eventType = 0;

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

				if (strcmp(name, "eventtype") == 0 && strcmp(type, "int") == 0)
					eventType = atoi(value);

				EV << "    simtime: " << simtime << ", eventType: " << eventType << std::endl;
				triggerPlanList.push_back(std::pair<double, int>(simtime, eventType));
			}

			break;
		}
	}
}
#endif

//////////////////////////////    AOMDV::AomdvRtEntry    //////////////////////////////
AOMDV::AomdvRtEntry::AomdvRtEntry(AOMDV *o, LAddress::L3Type d) : owner(o), error(false), connected(false), expireAt(simTime()+DELETE_PERIOD),
		reqTimeout(), reqCount(0), reqLastTTL(0), flags(RTF_DOWN), advertisedHops(INFINITE_HOPS), lastHopCount(INFINITE_HOPS),
		seqno(0), highestSeqnoHeard(0), bufPktsNum(0), dest(d), brokenNb(-1)
{
	rrepTimeoutEvt = rrepAckTimeoutEvt = nullptr;
}

AOMDV::AomdvRtEntry::~AomdvRtEntry()
{
	owner->cancelAndDelete(rrepTimeoutEvt);
	owner->cancelAndDelete(rrepAckTimeoutEvt);
}

AOMDV::AomdvPath* AOMDV::AomdvRtEntry::pathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop, uint8_t hopCount, simtime_t expire)
{
	simtime_t _expireAt = simTime() + expire;
	AOMDV::AomdvPath path(nextHop, lastHop, hopCount, _expireAt);
	pathList.push_front(path);
	itSP = pathList.begin();
	if (expireAt < _expireAt)
		expireAt = _expireAt;
	return &pathList.front();
}

AOMDV::AomdvPath* AOMDV::AomdvRtEntry::pathSelect()
{
	if (++itSP == pathList.end()) // round robin all disjoint paths
		itSP = pathList.begin();
	return &*itSP;
}

AOMDV::AomdvPath* AOMDV::AomdvRtEntry::pathLookup(LAddress::L3Type nextHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->nextHop == nextHop)
			return &*itPL;
	return nullptr;
}

AOMDV::AomdvPath* AOMDV::AomdvRtEntry::pathLookupLastHop(LAddress::L3Type lastHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->lastHop == lastHop)
			return &*itPL;
	return nullptr;
}

AOMDV::AomdvPath* AOMDV::AomdvRtEntry::pathLookupMinHop()
{
	uint8_t minHopCount = INFINITE_HOPS;
	AOMDV::AomdvPath *path = nullptr;
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

AOMDV::AomdvPath* AOMDV::AomdvRtEntry::disjointPathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->nextHop == nextHop && itPL->lastHop == lastHop)
			return &*itPL;
	return nullptr;
}

bool AOMDV::AomdvRtEntry::disjointPathExists(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (itPL->nextHop == nextHop || itPL->lastHop == lastHop)
			return false;
	return true;
}

void AOMDV::AomdvRtEntry::pathDelete(LAddress::L3Type nextHop)
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

void AOMDV::AomdvRtEntry::pathPurge()
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

std::string AOMDV::AomdvRtEntry::pathPrint()
{
	const char *ws = "        ";
	std::ostringstream oss;
	oss << "    nextHop  lastHop  hopCount  expireAt\n";
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		oss << "      " << itPL->nextHop << ws << itPL->lastHop << ws << (uint32_t)itPL->hopCount << ws << itPL->expireAt.dbl() << "\n";
	return oss.str();
}

uint8_t AOMDV::AomdvRtEntry::pathGetMinHopCount()
{
	uint8_t minHopCount = INFINITE_HOPS;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (minHopCount > itPL->hopCount)
			minHopCount = itPL->hopCount;
	return minHopCount;
}

uint8_t AOMDV::AomdvRtEntry::pathGetMaxHopCount()
{
	uint8_t maxHopCount = 0;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (maxHopCount < itPL->hopCount)
			maxHopCount = itPL->hopCount;
	return maxHopCount;
}

simtime_t AOMDV::AomdvRtEntry::pathGetMaxExpirationTime()
{
	simtime_t maxExpirationTime = SimTime::ZERO;
	for (itPL = pathList.begin(); itPL != pathList.end(); ++itPL)
		if (maxExpirationTime < itPL->expireAt)
			maxExpirationTime = itPL->expireAt;
	return maxExpirationTime;
}

#ifdef AOMDV_LINK_DISJOINT_PATHS
//////////////////////////////    AOMDV::AomdvBroadcastID    //////////////////////////////
void AOMDV::AomdvBroadcastID::forwardPathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	forwardPathList.push_back(AOMDV::AomdvRoute(nextHop, lastHop));
}

void AOMDV::AomdvBroadcastID::reversePathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	reversePathList.push_back(AOMDV::AomdvRoute(nextHop, lastHop));
}

AOMDV::AomdvRoute* AOMDV::AomdvBroadcastID::forwardPathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	for (std::list<AomdvRoute>::iterator it = forwardPathList.begin(); it != forwardPathList.end(); ++it)
		if (it->nextHop == nextHop && it->lastHop == lastHop)
			return &*it;
	return nullptr;
}

AOMDV::AomdvRoute* AOMDV::AomdvBroadcastID::reversePathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop)
{
	for (std::list<AomdvRoute>::iterator it = reversePathList.begin(); it != reversePathList.end(); ++it)
		if (it->nextHop == nextHop && it->lastHop == lastHop)
			return &*it;
	return nullptr;
}
#endif
