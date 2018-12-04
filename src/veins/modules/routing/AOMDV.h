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

#ifndef __AOMDV_H__
#define __AOMDV_H__

#include "veins/modules/wave/BaseWaveApplLayer.h"
#include "veins/modules/messages/AomdvPkt_m.h"
#include "veins/modules/routing/RouteRepairInterface.h"

#define AOMDV_NODE_DISJOINT_PATHS
// #define AOMDV_LINK_DISJOINT_PATHS

#define RTF_DOWN         0
#define RTF_UP           1
#define RTF_IN_REPAIR    2

#define ALLOWED_HELLO_LOSS    2
#define RREQ_RETRIES          2
#define RREQ_RATELIMIT       10
#define AOMDV_MAX_PATHS       3
#define AOMDV_MAX_PATH_HOP_DIFF    1

// Should be set by the user using best guess (conservative)
#define NET_DIAMETER           10
#define NODE_TRAVERSAL_TIME    0.01
#define NET_TRAVERSAL_TIME     (2 * NODE_TRAVERSAL_TIME * NET_DIAMETER)
#define PATH_DISCOVERY_TIME    (2 * NET_TRAVERSAL_TIME)

// TTL_START should be set to at least 2 if Hello messages are used for local connectivity information.
#define TTL_START         2
#define TTL_INCREMENT     2
#define TTL_THRESHOLD     7
#define MAX_REPAIR_TTL    0.3 * NET_DIAMETER
#define LOCAL_ADD_TTL     2

#define PURGE_ROUTE_PERIOD        0.5
// ACTIVE_ROUTE_TIMEOUT SHOULD be set to a longer value (at least 10,000 milliseconds)
// if link-layer indications are used to detect link breakages such as in IEEE 802.11 standard.
#define ACTIVE_ROUTE_TIMEOUT     10.0
// The configured value for MY_ROUTE_TIMEOUT MUST be at least 2 * PATH_DISCOVERY_TIME.
#define MY_ROUTE_TIMEOUT          6.0
#define REVERSE_ROUTE_LIFE        6.0
#define PURGE_BCAST_ID_PERIOD     5.0
#define BCAST_ID_SAVE             5.0
// Must be larger than the time difference between a node propagates a route request and gets the route reply back.
#define RREP_WAIT_TIME            1.0
#define RREPACK_WAIT_TIME         1.0
// If the link layer feedback is used to detect loss of link, DELETE_PERIOD must be at least ACTIVE_ROUTE_TIMEOUT.
#define DELETE_PERIOD    ACTIVE_ROUTE_TIMEOUT

#ifdef TEST_ROUTE_REPAIR_PROTOCOL
#define STOP_SEND_BEACON_EVENT_TYPE    0
#define CONT_SEND_BEACON_EVENT_TYPE    1
#endif

/**
 * @brief A well known routing protocol designed for MANET.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseWaveApplLayer
 * @see RouteRepairInterface
 *
 * M. K. Marina, and S. R. Das, "Ad hoc on-demand multipath distance vector routing," Wirel. Commun. Mob. Comput., 2006; 6:969–988.
 * 3.1. Protocol Overview
 * AOMDV shares several characteristics with AODV. It is based on the distance vector concept and uses
 * hop-by-hop routing approach. Moreover, AOMDV also finds routes on demand using a route discovery procedure.
 * The main difference lies in the number of routes found in each route discovery. In AOMDV, RREQ propagation
 * from the source towards the destination establishes multiple reverse paths both at intermediate nodes
 * as well as the destination. Multiple RREPs traverse these reverse paths back to form multiple forward
 * paths to the destination at the source and intermediate nodes. Note that AOMDV also provides intermediate
 * nodes with alternate paths as they are found to be useful in reducing route discovery frequency.
 * The core of the AOMDV protocol lies in ensuring that multiple paths discovered are loop-free and disjoint,
 * and in efficiently finding such paths using a flood-based route discovery. AOMDV route update rules,
 * applied locally at each node, play a key role in maintaining loop-freedom and disjointness properties.
 * 3.1.2. Disjoint paths
 * The following simple and straightforward observation is the basis of our mechanism to find link disjoint paths:
 * If two paths from a node P to a destination D are link disjoint, then they must have unique next hops
 * as well as unique last hops. Note that the converse of this observation is not necessarily true.
 * However, the converse also holds true in general with an additional restriction: if every node on a path
 * ensures that all paths to the destination from that node differ in their next and last hops. This implication
 * provides us with a tool to determine whether two paths via two unique downstream neighbors are link disjoint.
 * They simply need to have unique last hops.
 */
class AOMDV : public BaseWaveApplLayer, private RouteRepairInterface
{
public:
	/** @brief The message kinds AOMDV uses. */
	enum AOMDVMsgKinds {
		PURGE_ROUTING_TABLE_EVT = LAST_WAVE_APPL_MESSAGE_KIND,
		PURGE_BROADCAST_CACHE_EVT,
		RREP_TIMEOUT_EVT,
		RREPACK_TIMEOUT_EVT,
		LOCAL_REPAIR_TIMEOUT_EVT,
		SEND_DATA_EVT,
		SEND_BUF_DATA_EVT,
#ifdef TEST_ROUTE_REPAIR_PROTOCOL
		CALL_TRIGGER_EVT,
#endif
		LAST_AOMDV_MESSAGE_KIND
	};
#ifdef AOMDV_LINK_DISJOINT_PATHS
	class AomdvRoute
	{
	public:
		AomdvRoute(LAddress::L3Type nh, LAddress::L3Type lh) : nextHop(nh), lastHop(lh) {}

		LAddress::L3Type nextHop; ///< next hop address.
		LAddress::L3Type lastHop; ///< last hop address.
	};
#endif
	/**
	 * AOMDV route table entry has a new field for the advertised hop count. Besides a route list is used
	 * in AOMDV to store additional information for each alternate path including: next hop, last hop,
	 * hop count, and expiration timeout. As already discussed, last hop information is useful in
	 * checking the disjointness of alternate paths.
	 */
	class AomdvPath
	{
	public:
		AomdvPath(LAddress::L3Type nh, LAddress::L3Type lh, uint8_t hc, simtime_t ea) : nextHop(nh), lastHop(lh), hopCount(hc), expireAt(ea) {}

		std::string info()
		{
			std::ostringstream oss;
			oss << '(' << nextHop << ',' << lastHop << ',' << (uint32_t)hopCount << ',' << expireAt.dbl() << ')';
			return oss.str();
		}

		LAddress::L3Type nextHop; ///< next hop address.
		LAddress::L3Type lastHop; ///< last hop address.
		uint8_t hopCount;         ///< hop count through this next hop.
		simtime_t expireAt;       ///< expiration time instant.
	};

	class AomdvRtEntry
	{
	public:
		AomdvRtEntry(AOMDV *o, LAddress::L3Type d);
		~AomdvRtEntry();

		/** @name path operations. */
		///@{
		AomdvPath*  pathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop, uint8_t hopCount, simtime_t expireAt);
		AomdvPath*  pathSelect();
		AomdvPath*  pathLookup(LAddress::L3Type nextHop);
		AomdvPath*  pathLookupLastHop(LAddress::L3Type lastHop);
		AomdvPath*  pathLookupMinHop();
		AomdvPath*  disjointPathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		bool        disjointPathExists(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		void        pathDelete(LAddress::L3Type nextHop);
		void        pathPurge();
		std::string pathPrint();
		uint8_t     pathGetMinHopCount();
		uint8_t     pathGetMaxHopCount();
		simtime_t   pathGetMaxExpirationTime();
		///@}

	private:
		AOMDV *owner; ///< usage: owner->cancelAndDelete(sendBufDataEvt) in destructor.

	public:
		bool error;
		simtime_t expireAt;   ///< expiration time instant of this routing entry.
		simtime_t reqTimeout; ///< when I can send another request.
		uint8_t reqCount;     ///< number of route requests.
		uint8_t reqLastTTL;   ///< last TTL value used.
		uint8_t flags;        ///< state of this table entry.
		uint8_t advertisedHops; ///< advertised hop count.
		uint8_t lastHopCount;   ///< last valid hop count.
		uint32_t seqno;         ///< sequence number of destination.
		uint32_t highestSeqnoHeard; ///< highest sequence number ever heard.
		LAddress::L3Type dest;      ///< usage: sendBufDataEvt->setContextPointer(&dest).
		LAddress::L3Type brokenNb;  ///< usage: rrepTimeoutEvt->setContextPointer(&brokenNb).
		cMessage *sendBufDataEvt;   ///< self message event used to periodically send buffered data packet to the receiver.
		WaitForRREPMessage *rrepTimeoutEvt;    ///< self message event used to wait for RREP (RREP timeout).
		WaitForRREPMessage *rrepAckTimeoutEvt; ///< self message event used to wait for RREP-ACK (RREP-ACK timeout).

		std::list<AomdvPath> pathList; ///< store all disjoint paths to a destination.
		std::list<AomdvPath>::iterator itPL; ///< an iterator used to traverse container pathList.
		std::list<AomdvPath>::iterator itSP; ///< an iterator used to poll paths in pathList when calling pathSelect().
	};

	class AomdvBroadcastID
	{
	public:
		explicit AomdvBroadcastID(uint32_t bid) : rreqId(bid), count(0), expireAt(simTime()+BCAST_ID_SAVE) {}
#ifdef AOMDV_LINK_DISJOINT_PATHS
		void forwardPathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		void reversePathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		AomdvRoute* forwardPathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		AomdvRoute* reversePathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
#endif
		uint32_t rreqId;    ///< RREQ ID of the RREQ source.
		uint32_t count;     ///< how many times I have sent RREP for the same <RREQ src IP, RREQ ID> pair.
		simtime_t expireAt; ///< expiration time instant.
#ifdef AOMDV_LINK_DISJOINT_PATHS
		std::list<AomdvRoute> forwardPathList;
		std::list<AomdvRoute> reversePathList;
#endif
	};

public:
	/** @name constructors, destructor. */
	///@{
	AOMDV() : BaseWaveApplLayer(), RouteRepairInterface() {}
	AOMDV(unsigned stacksize) : BaseWaveApplLayer(stacksize), RouteRepairInterface() {}
	~AOMDV() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from lower layer. */
	void handleLowerMsg(cMessage *msg) override;
	/** @brief handle control messages from lower layer. */
	void handleLowerControl(cMessage *msg) override;

	/** @name packet receiving handlers. */
	///@{
	/** @brief call-back method of receiving beacon message. */
	void onBeacon(BeaconMessage *beaconMsg) override;
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg);
	/** @brief call-back method of receiving AOMDV RREQ message. */
	void onRREQ(RREQMessage *rreq);
	/** @brief call-back method of receiving AOMDV RREP message. */
	void onRREP(RREPMessage *rrep);
	/** @brief call-back method of receiving AOMDV RERR message. */
	void onRERR(RERRMessage *rerr);
	/** @brief call-back method of receiving AOMDV RREPACK message. */
	void onRREPACK(RoutingMessage *rrepAck);
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataPkt);
	/** @brief call-back method of receiving data control message. */
	void onDataLost(DataMessage *lostDataPkt);
	///@}

	/** @name packet sending functions. */
	///@{
	/** @brief forward AOMDV message. */
	void forward(AomdvRtEntry *entry, RoutingMessage *pkt);
	/** @brief broadcast AOMDV RREQ message. */
	void sendRREQ(LAddress::L3Type dest, uint8_t TTL=INFINITE_HOPS);
	/** @brief send AOMDV RREP message. */
	void sendRREP(RREQMessage *rq, uint8_t hopCount, uint32_t rpseq, simtime_t lifetime, LAddress::L3Type lastHop);
	/** @brief send AOMDV RERR message. */
	void sendRERR(RERRMessage *rerr);
	/** @brief send AOMDV RREP message. */
	void sendRREPACK(RREPMessage *rrep);
	///@}

	/** @brief continuously failed to receive beacon message from the neighbor. */
	void onNeighborLost(LAddress::L3Type neighbor) override;
	/** @brief repair route locally. */
	void localRepair(AomdvRtEntry *entry, DataMessage *dataPkt);
	/** @name route repair interface implementation. */
	///@{
	/** @brief link layer notifies that communication link is broken when transmitting. */
	void onLinkBroken(LAddress::L3Type neighbor, DataMessage *dataPkt) override;
	/** @brief called when local repair timer is timeout. */
	void onLocalRepairFailure(LAddress::L3Type neighbor) override;
	/** @brief Notified that there is a reachable route to a destination. */
	void onRouteReachable(LAddress::L3Type dest) override;
	/** @brief Notified that there is no reachable route to a destination. */
	void onRouteUnreachable(LAddress::L3Type dest) override;
	/** @brief call a routing request to a certain receiver determined by routingPlanList. */
	void callRouting(LAddress::L3Type receiver, int contentSize) override;
	/** @brief List all the routing path maintained in the routing table. */
	void printRoutingTable() override;
	///@}

	/** @name routing table management functions. */
	///@{
	AomdvRtEntry* insertRoutingEntry(LAddress::L3Type dest);
	AomdvRtEntry* lookupRoutingEntry(LAddress::L3Type dest);
	void downRoutingEntry(AomdvRtEntry *entry);
	void purgeRoutingTable();
	///@}

	/** @name broadcast ID management functions. */
	///@{
	AomdvBroadcastID* insertBroadcastID(LAddress::L3Type src, uint32_t bid);
	AomdvBroadcastID* lookupBroadcastID(LAddress::L3Type src, uint32_t bid);
	void purgeBroadcastCache();
	///@}

	/** @brief when buffer queue is not empty, try scheduling sendBufDataEvt to send buffered data packets. */
	void trySendBufPackets(AomdvRtEntry *entry);

#ifdef TEST_ROUTE_REPAIR_PROTOCOL
	/** @brief call a trigger event determined by triggerPlanList. */
	void callTrigger(int eventType);
	/** @brief initialize triggerPlanList through configured xmlfile. */
	void initializeTriggerPlanList(cXMLElement *xmlConfig);
#endif

private:
	uint32_t seqno;  ///< Sequence Number.
	uint32_t rreqID; ///< Each node maintains only one RREQ ID.

	cMessage *purgeRoutingTableEvt;   ///< self message event used to periodically purge routing table.
	cMessage *purgeBroadcastCacheEvt; ///< self message event used to periodically purge broadcast cache.

	std::map<LAddress::L3Type, AomdvRtEntry*> routingTable;   ///< a map from L3 address of the destination to its entry in the routing table.
	std::map<LAddress::L3Type, AomdvRtEntry*>::iterator itRT; ///< an iterator used to traverse container routingTable.
	std::map<LAddress::L3Type, AomdvBroadcastID*> broadcastCache; ///< a map from L3 address of the source to its entry in the broadcast cache.
	std::map<LAddress::L3Type, AomdvBroadcastID*>::iterator itBC; ///< an iterator used to traverse container broadcastCache.
#ifdef TEST_ROUTE_REPAIR_PROTOCOL
	cMessage *callTriggerEvt; ///< self message event used to call trigger events determined by triggerPlanList.
	std::list<std::pair<double, int> > triggerPlanList; ///< trigger plans of all vehicles configured by a xmlfile.
#endif
};

#endif /* __AOMDV_H__ */
