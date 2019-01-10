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

#ifndef __PAOMBR_H__
#define __PAOMBR_H__

#include "veins/modules/wave/BaseWaveApplLayer.h"
#include "veins/modules/messages/AomdvPkt_m.h"
#include "veins/modules/routing/RouteRepairInterface.h"

#define RTF_DOWN         0
#define RTF_UP           1
#define RTF_IN_REPAIR    2
#define RTF_IN_PLRR      3

#define RREQ_RETRIES        2
#define PAOMBR_MAX_PATHS    3

// TTL_START should be set to at least 2 if Hello messages are used for local connectivity information.
#define TTL_START         2
#define TTL_INCREMENT     1
#define TTL_THRESHOLD     7
#define MAX_REPAIR_TTL    5
#define LOCAL_ADD_TTL     1

// Should be set by the user using best guess (conservative)
#define NET_DIAMETER           10
// In worst cases, CCH -> SCH happens just before packet arrives at MAC layer,
// and note that CCH -> SCH worst case happens at a probability of 50%, thus expected delay is 25ms
#define NODE_TRAVERSAL_TIME    0.025
// round trip time of 2 hop count (1 + LOCAL_ADD_TTL)
#define PLRR_DISCOVERY_TIME    4*NODE_TRAVERSAL_TIME

#define PURGE_ROUTE_PERIOD        0.5

#define ACTIVE_ROUTE_TIMEOUT      6.0
#define MY_ROUTE_TIMEOUT          6.0
#define REVERSE_ROUTE_LIFE        6.0
#define PURGE_BCAST_ID_PERIOD     2.5
#define BCAST_ID_SAVE             5.0
// Must be larger than the time difference between a node propagates a route request and gets the route reply back.
#define RREP_WAIT_TIME            1.0
#define RREPACK_WAIT_TIME         1.0
// If the link layer feedback is used to detect loss of link, DELETE_PERIOD must be at least ACTIVE_ROUTE_TIMEOUT.
#define DELETE_PERIOD    ACTIVE_ROUTE_TIMEOUT

#define USE_DYNAMIC_PATH_SHORTENING
#define USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
#define PLRR_SUPPRESSION_TIME     3
#define MAX_RECV_POWER_POLYNOMIAL_DEGREE 4

#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
#define USE_IRRESPONSIBLE_REBROADCAST
// #define USE_DESTINATION_AGGREGATION
#endif

#ifdef USE_DYNAMIC_PATH_SHORTENING
#undef USE_L2_UNICAST_DATA
#endif

/**
 * @brief Preemptive Ad hoc On-demand Multipath Bypass Routing protocol designed for VANET.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseWaveApplLayer
 * @see RouteRepairInterface
 *
 * T. Goff, N. A.-Ghazaleh, D. Phatak, and R. Kahvecioglu, "Preemptive Routing in Ad Hoc Networks," Journal of Parallel and Distributed Computing, Jun. 2002.
 * C. Sengul, and R. Kravets, "Bypass routing: An on-demand local recovery protocol for ad hoc networks," Ad Hoc Networks, vol. 4, pp. 380-397, 2006.
 */
class PAOMBR : public BaseWaveApplLayer, private RouteRepairInterface
{
public:
	/** @brief The message kinds PAOMBR uses. */
	enum PAOMBRMsgKinds {
		PURGE_ROUTING_TABLE_EVT = LAST_WAVE_APPL_MESSAGE_KIND,
		PURGE_BROADCAST_CACHE_EVT,
		RREP_TIMEOUT_EVT,
		RREPACK_TIMEOUT_EVT,
		PLRR_TIMEOUT_EVT,
		LET_TIMEOUT_EVT,
		SEND_DATA_EVT,
		SEND_BUF_DATA_EVT,
		LAST_PAOMBR_MESSAGE_KIND
	};

	/**
	 * PAOMBR route table entry has a new field for the advertised hop count. Besides a route list is used
	 * in PAOMBR to store additional information for each alternate path including: next hop, last hop,
	 * hop count, and expiration timeout. As already discussed, last hop information is useful in
	 * checking the disjointness of alternate paths.
	 */
	class PaombrPath
	{
	public:
		PaombrPath(LAddress::L3Type nh, LAddress::L3Type lh, uint8_t hc, simtime_t ea, simtime_t pet) : nextHop(nh), lastHop(lh), hopCount(hc), expireAt(ea), PET(pet) {}

		std::string info()
		{
			std::ostringstream oss;
			oss << '(' << nextHop << ',' << lastHop << ',' << (uint32_t)hopCount << ',' << expireAt.dbl() << ',' << PET.dbl() << ')';
			return oss.str();
		}

		LAddress::L3Type nextHop; ///< next hop address.
		LAddress::L3Type lastHop; ///< last hop address.
		uint8_t hopCount;         ///< hop count through this next hop.
		simtime_t expireAt;       ///< expiration time instant.
		simtime_t PET;            ///< path expiration time.
	};

	class PaombrRtEntry
	{
	public:
		PaombrRtEntry(PAOMBR *o, LAddress::L3Type d);
		~PaombrRtEntry();

		/** @name path operations. */
		///@{
		PaombrPath* pathInsert(LAddress::L3Type nextHop, LAddress::L3Type lastHop, uint8_t hopCount, simtime_t expireAt, simtime_t PET);
		PaombrPath* pathSelect(SendState *sendState=nullptr);
		PaombrPath* pathLookup(LAddress::L3Type nextHop);
		PaombrPath* pathLookupLastHop(LAddress::L3Type lastHop);
		PaombrPath* pathLookupMinHop();
		PaombrPath* disjointPathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		bool        disjointPathExists(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		void        pathDelete(LAddress::L3Type nextHop);
		void        pathPurge();
		std::string pathPrint();
		uint8_t     pathGetMinHopCount();
		uint8_t     pathGetMaxHopCount();
		simtime_t   pathGetMaxExpirationTime();
		simtime_t   pathGetMaxPET();
		bool revPathInsert(LAddress::L3Type source, LAddress::L3Type lastHop);
		LAddress::L3Type revPathLookup(LAddress::L3Type source);
		void revPathDelete(LAddress::L3Type source);
		std::string revPathPrint();
		///@}

	private:
		PAOMBR *owner; ///< usage: owner->cancelAndDelete(sendBufDataEvt) in destructor.

	public:
		bool error;     ///< need to broadcast RERR if true.
		bool connected; ///< is upper protocol connection based on this routing entry.
		simtime_t expireAt;   ///< expiration time instant of this routing entry.
		simtime_t reqTimeout; ///< when I can send another request.
		uint8_t reqCount;     ///< number of route requests.
		uint8_t reqLastTTL;   ///< last TTL value used.
		uint8_t flags;        ///< state of this table entry.
		uint8_t advertisedHops; ///< advertised hop count.
		uint8_t lastHopCount;   ///< last valid hop count.
		uint32_t seqno;         ///< sequence number of destination.
		uint32_t highestSeqnoHeard; ///< highest sequence number ever heard.
		uint32_t bufPktsNum;  ///< number of data packets toward this destination buffered in queue.
		LAddress::L3Type dest;      ///< used inside a function whose parameters contain PaombrRtEntry*.
		LAddress::L3Type brokenNb;  ///< usage: rrepTimeoutEvt->setContextPointer(&brokenNb).
		WaitForRREPMessage *rrepTimeoutEvt;    ///< self message event used to wait for RREP (RREP timeout).
		WaitForRREPMessage *rrepAckTimeoutEvt; ///< self message event used to wait for RREP-ACK (RREP-ACK timeout).

		std::list<PaombrPath> pathList; ///< store all disjoint paths to a destination.
		std::list<PaombrPath>::iterator itPL; ///< an iterator used to traverse container pathList.
		std::list<PaombrPath>::iterator itSP; ///< an iterator used to select a path in pathList when calling pathSelect().
		std::list<std::pair<LAddress::L3Type, LAddress::L3Type> > revPathList; ///< store sources that use this intermediate node and last hops on these paths.
		std::list<std::pair<LAddress::L3Type, LAddress::L3Type> >::iterator itRPL; ///< an iterator used to traverse container revPathList.
	};

	class PaombrBroadcastID
	{
	public:
		PaombrBroadcastID(LAddress::L3Type src, uint32_t bid) : source(src), rreqId(bid), count(0), expireAt(simTime()+BCAST_ID_SAVE) {}

		LAddress::L3Type source; ///< RREQ source.
		uint32_t rreqId;    ///< RREQ ID of the RREQ source.
		uint32_t count;     ///< how many times I have sent RREP for the same <RREQ src IP, RREQ ID> pair.
		simtime_t expireAt; ///< expiration time instant.
	};

public:
	/** @name constructors, destructor. */
	///@{
	PAOMBR() : BaseWaveApplLayer(), RouteRepairInterface() {}
	PAOMBR(unsigned stacksize) : BaseWaveApplLayer(stacksize), RouteRepairInterface() {}
	~PAOMBR() {}
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
	/** @brief call-back method of receiving PAOMBR RREQ message. */
	void onRREQ(RREQMessage *rreq);
	/** @brief call-back method of receiving PAOMBR RREP message. */
	void onRREP(RREPMessage *rrep);
	/** @brief call-back method of receiving PAOMBR RERR message. */
	void onRERR(RERRMessage *rerr);
#ifdef USE_DESTINATION_AGGREGATION
	/** @brief call-back method of receiving PAOMBR RREQp message. */
	void onRREQp(RREQpMessage *rreqp);
	/** @brief call-back method of receiving PAOMBR RREPp message. */
	void onRREPp(RREPpMessage *rrepp);
#endif
#ifdef USE_RECEIVER_REPORT
	/** @brief call-back method of receiving PAOMBR RR message. */
	void onRR(RRMessage *rr);
#endif
	/** @brief call-back method of receiving PAOMBR LR message. */
	void onLR(LRMessage *lr);
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataPkt);
	/** @brief call-back method of receiving data control message. */
	void onDataLost(DataMessage *lostDataPkt);
	///@}

	/** @name packet sending functions. */
	///@{
	/** @brief forward PAOMBR message. */
	LAddress::L3Type forward(PaombrRtEntry *entry, RoutingMessage *pkt, LAddress::L3Type dest=-1);
	/** @brief broadcast PAOMBR RREQ message. */
	void sendRREQ(LAddress::L3Type dest, uint8_t TTL=INFINITE_HOPS, LAddress::L3Type lastHop=-1);
	/** @brief send PAOMBR RREP message. */
	void sendRREP(RREQMessage *rq, uint8_t hopCount, uint32_t rpseq, simtime_t lifetime, LAddress::L3Type lastHop);
	/** @brief send PAOMBR RERR message. */
	void sendRERR(RERRMessage *rerr);
#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
	/** @brief broadcast PAOMBR RREQp message. */
	void sendRREQp(LAddress::L3Type neighbor);
#ifdef USE_DESTINATION_AGGREGATION
	/** @brief send PAOMBR RREPp message. */
	void sendRREPp(RREQpMessage *rreqp, RREPpMessage *rrepp);
#endif
#endif
	/** @brief send PAOMBR LR message. */
	void sendLR(PaombrRtEntry *entry, RREPMessage *rrep);
	///@}

	/** @brief continuously failed to receive beacon message from the neighbor. */
	void onNeighborLost(LAddress::L3Type neighbor) override;
	/** @brief repair route locally. */
	void localRepair(PaombrRtEntry *entry, DataMessage *dataPkt, LAddress::L3Type lastHop);
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
	void callRouting(const PlanEntry& entry) override;
	/** @brief List all the routing path maintained in the routing table. */
	void printRoutingTable() override;
	///@}

	/** @name routing table management functions. */
	///@{
	PaombrRtEntry* insertRoutingEntry(LAddress::L3Type dest);
	PaombrRtEntry* lookupRoutingEntry(LAddress::L3Type dest);
	void downRoutingEntry(PaombrRtEntry *entry);
	void purgeRoutingTable();
	///@}

	/** @name broadcast ID management functions. */
	///@{
	PaombrBroadcastID* insertBroadcastID(LAddress::L3Type src, uint32_t bid);
	PaombrBroadcastID* lookupBroadcastID(LAddress::L3Type src, uint32_t bid);
	void purgeBroadcastCache();
	///@}

	/** @brief calculate link expiration time based on mobility information. */
	simtime_t calcLET(const Coord& nbPos, const Coord& nbSpeed);
	/** @brief reschedule data packets in rQueue, transfer them to bQueue. */
	void transferBufPackets(LAddress::L3Type dest);

#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
	/** @brief check if the link is going to break according to received power. */
	bool recvPowerCritical(std::list<double>& recvPowers);

private:
	/** @brief The derived class to store neighbor's information collected by beacon message. */
	class PaombrNeighborInfo : public BaseWaveApplLayer::NeighborInfo
	{
	public:
		PaombrNeighborInfo(Coord& p, Coord& s, LAddress::L2Type ma, simtime_t ra, LAddress::L3Type _nb);

		LAddress::L3Type nb;  ///< usage: LETTimeoutEvt->setContextPointer(&nb).
		simtime_t LET; ///< predicted link expiration time.
		simtime_t PLRRTimeout; ///< when preemptive local route repair timeout.
		simtime_t PLRRSuppressUntil; ///< when can send out a new RREQp no matter whether a previous one is successful.
		cMessage *LETTimeoutEvt;  ///< self message event used to set the time to broadcast RREQp.
		cMessage *PLRRTimeoutEvt; ///< self message event used to wait for RREPp (RREPp timeout).
		std::list<double> recvPower_dBm; ///< sampled received power measured in dBm.
	};
#endif

private:
	uint32_t seqno;  ///< Sequence Number.
	uint32_t rreqID; ///< Each node maintains only one RREQ ID.

	cMessage *sendBufDataEvt; ///< self message event used to periodically send buffered data packet to the receiver.
	cMessage *purgeRoutingTableEvt;   ///< self message event used to periodically purge routing table.
	cMessage *purgeBroadcastCacheEvt; ///< self message event used to periodically purge broadcast cache.

	std::list<PaombrBroadcastID> broadcastCache; ///< store (RREQ source, RREQ ID) pair and related information.
	std::list<PaombrBroadcastID>::iterator itBC; ///< an iterator used to traverse container broadcastCache.
	std::map<LAddress::L3Type, PaombrRtEntry*> routingTable;   ///< a map from L3 address of the destination to its entry in the routing table.
	std::map<LAddress::L3Type, PaombrRtEntry*>::iterator itRT; ///< an iterator used to traverse container routingTable.

	static double recvPowerThres_dBm; ///< received power threshold below which the link is going to break.
};

#endif /* __PAOMBR_H__ */
