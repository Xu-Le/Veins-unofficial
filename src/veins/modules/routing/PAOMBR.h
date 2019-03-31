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

#define PAOMBR_MAX_PATHS    3

#ifdef USE_DYNAMIC_PATH_SHORTENING
#undef USE_L2_UNICAST_DATA
#define DPS_OPERATION_TIME    0.05
#endif

/**
 * @brief Preemptive Ad hoc On-demand Multipath Bypass Routing protocol designed for VANET.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseWaveApplLayer
 * @see RouteRepairInterface
 *
 * [1] T. Goff, N. Abu-Ghazaleh, D. Phatak, and R. Kahvecioglu, "Preemptive routing in ad hoc networks,"
 *     in Proc. ACM Annu. Int. Conf. Mobile Comput. Netw. (MobiCom), Rome, Italy, 2001, pp. 43-52.
 * [2] S. Crisostomo, S. Sargento, P. Brandao, and R. Prior, "Improving AODV with preemptive local route repair,"
 *     in IEEE International Workshop on Wireless Ad-Hoc Networks, Oulu, Finland, 2004.
 * [3] A. Gorrieri, and G. Ferrari, "Irresponsible AODV routing," Vehicular Communications, vol. 2, pp. 47-57, Feb. 2015.
 * [4] Rei-H. Cheng, Tung-K. Wu, and C. W. Yu, "A highly topology adaptable ad hoc routing protocol with complementary
 *     preemptive link breaking avoidance and path shortening mechanisms," Wireless Netw., vol. 16, pp. 1289-1311, Aug. 2010.
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
#ifdef USE_PREEMPTIVE_LOCAL_ROUTE_REPAIR
		PLRR_TIMEOUT_EVT,
		LET_TIMEOUT_EVT,
#endif
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

	class AssistedOrig
	{
	public:
		AssistedOrig(LAddress::L3Type o, LAddress::L3Type lh) : originator(o), lastHop(lh), minAltitude(INFINITE_HOPS), minSender(-1), maxSeqno(-1) {}

		LAddress::L3Type originator; ///< originator for whom I provide routing assistance.
		LAddress::L3Type lastHop;    ///< last hop address.
#ifdef USE_DYNAMIC_PATH_SHORTENING
		uint8_t minAltitude;         ///< minimum altitude among recently received data packets.
		LAddress::L3Type minSender;  ///< sender corresponding to minAltitude.
		int maxSeqno;                ///< maximum sequence number among recently received data packets.
#endif
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
		PaombrPath* pathLookupMaxPET();
		PaombrPath* disjointPathLookup(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		bool        disjointPathExists(LAddress::L3Type nextHop, LAddress::L3Type lastHop);
		void        pathDelete(LAddress::L3Type nextHop);
		void        pathPurge();
		std::string pathPrint();
		uint8_t     pathGetMinHopCount();
		uint8_t     pathGetMaxHopCount();
		simtime_t   pathGetMaxExpirationTime();
		simtime_t   pathGetMaxPET();
		bool aoInsert(LAddress::L3Type orig, LAddress::L3Type lastHop);
		AssistedOrig* aoLookup(LAddress::L3Type orig);
		LAddress::L3Type aoLookupLastHop(LAddress::L3Type orig);
		void aoDelete(LAddress::L3Type orig);
		std::string aoPrint();
#ifdef USE_DYNAMIC_PATH_SHORTENING
		void aoReset();
#endif
		std::string print();
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
		WaitForRREPMessage *rrepTimeoutEvt;   ///< self message event used to wait for RREP (RREP timeout).

		std::list<PaombrPath> pathList; ///< store all disjoint paths to a destination.
		std::list<PaombrPath>::iterator itPL; ///< an iterator used to traverse container pathList.
		std::list<PaombrPath>::iterator itSP; ///< an iterator used to select a path in pathList when calling pathSelect().
		std::list<AssistedOrig> ao; ///< store originators' information that use this intermediate node.
		std::list<AssistedOrig>::iterator itAO; ///< an iterator used to traverse container ao.
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
	/** @brief call-back method of receiving PAOMBR LR message. */
	void onLR(LRMessage *lr);
#ifdef USE_DYNAMIC_PATH_SHORTENING
	/** @brief call-back method of receiving PAOMBR DPSR message. */
	void onDPSR(DPSRMessage *dpsr);
#endif
#ifdef USE_RECEIVER_REPORT
	/** @brief call-back method of receiving PAOMBR RR message. */
	void onRR(RRMessage *rr);
#endif
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
#endif
	/** @brief send PAOMBR LR message. */
	void sendLR(PaombrRtEntry *entry, RREPMessage *rrep);
	/** @brief send PAOMBR DPSR message. */
	void sendDPSR(DataMessage *dataPkt, AssistedOrig *pao);
	///@}

	/** @brief continuously failed to receive beacon message from the neighbor. */
	void onNeighborLost(LAddress::L3Type neighbor) override;
	/** @brief repair route locally. */
	bool localRepair(PaombrRtEntry *entry, DataMessage *dataPkt, LAddress::L3Type lastHop);
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
