//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __CONTENTRSU_H__
#define __CONTENTRSU_H__

#include "veins/modules/rsu/BaseRSU.h"
#include "veins/modules/rsu/STAG.h"
#include "veins/modules/application/ContentStatisticCollector.h"

/**
 * @brief A concrete RSU class which provides content download service.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRSU
 */
class ContentRSU : public BaseRSU
{
public:
	/** @brief The message kinds content RSU uses. */
	enum ContentRSUMsgKinds {
		DISTRIBUTE_V_EVT = LAST_BASE_RSU_MESSAGE_KIND,
		DISTRIBUTE_R_EVT,
		DISTRIBUTE_C_EVT,
		SCHEME_SWITCH_EVT,
		SEGMENT_ADVANCE_EVT,
		PREFETCH_REQUEST_EVT,
		LOOK_FOR_CARRIER_EVT,
		LINK_BROKEN_EVT,
		LAST_CONTENT_RSU_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	ContentRSU() : BaseRSU() {}
	ContentRSU(unsigned stacksize) : BaseRSU(stacksize) {}
	virtual ~ContentRSU() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;
	/** @brief Handle wired incoming messages. */
	void handleWiredMsg(WiredMessage *wiredMsg) override;
	/** @brief Handle west/east RSU messages, param 'direction': west is 1, east is 2, north is 3, south is 4. */
	void handleRSUMsg(WiredMessage *wiredMsg, int direction) override;

	/** @brief wave short message decorate method. */
	void decorateWSM(WaveShortMessage *wsm) override;

	/** @brief call-back method of receiving beacon message. */
	void onBeacon(BeaconMessage *beaconMsg) override;
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg) override;
	/** @brief call-back method of receiving content message. */
	void onContent(ContentMessage *contentMsg) override;
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg) override;

	/** @name relay related methods. */
	///@{
	/** @brief calculate the proper data amount to prefetch from content server. */
	SimTime obtainCooperativeScheme();
	/** @brief predict links' bandwidth between nodes in a short time according to the mobility information of vehicles. */
	void predictLinkBandwidth();
	/** @brief predict vehicles' mobility in a short time according to the current mobility status of vehicles. */
	void _predictVehicleMobility(std::vector<std::vector<Coord> >& vehicleSpeed, std::vector<std::vector<Coord> >& vehiclePos);
	/** @brief determine file segment offsets scheme in each time slot. */
	void _determineSchemeOffsets(Segment *&schemeOffset, Segment *STAGOffset, Segment *lackOffset);
	/** @brief handle with transmission scheme switch preparation. */
	void _prepareSchemeSwitch();
	/** @brief send prefetch request to the content server. */
	void _sendPrefetchRequest(const LAddress::L3Type downloader, const int startOffset, const int endOffset);
	/** @brief broadcast transmission scheme to relays. */
	void _broadcastTransmissionScheme();
	///@}

	/** @name carrier related methods. */
	///@{
	/** @brief calculate which carrier it the best one to be selected for carry-and-forward. */
	bool selectCarrier(LAddress::L3Type coDownloader);
	/** @brief filling cooperative notification message, and then send it to neighbor RSU. */
	void _sendCooperativeNotification(const LAddress::L3Type downloader, ContentMessage *reportMsg);
	///@}

private:
	/** @brief The class to store downloader's information. */
	class DownloaderInfo
	{
	public:
		DownloaderInfo(int t, int c) : totalContentSize(t), cacheStartOffset(-1), cacheEndOffset(0), distributedOffset(0), distributedROffset(0), acknowledgedOffset(0),
				remainingDataAmount(0), consumingRate(c), prefetchDataAmount(0), notifiedLinkBreak(false), sentCoNotification(false),
				distributedAt(), acknowledgedAt(), _lackOffset(), lackOffset(&_lackOffset) {}

		int totalContentSize;
		int cacheStartOffset;
		int cacheEndOffset;
		int distributedOffset;
		int distributedROffset;
		int acknowledgedOffset;
		int remainingDataAmount;
		int consumingRate;
		int prefetchDataAmount;
		bool notifiedLinkBreak;
		bool sentCoNotification;
		SimTime distributedAt;
		SimTime acknowledgedAt;
		Segment _lackOffset; ///< internal variable, head node of segment list, thus the whole list can be cleared when its destructor automatically called.
		Segment *lackOffset; ///< external variable, use lackOffset = lackOffset->next to iterate the segment list.
	};
	/** @brief The class to store co-downloader's information. */
	class CoDownloaderInfo
	{
	public:
		CoDownloaderInfo(Coord& _pos, Coord& _speed) : carrier(-1), transmissionAmount(-1), tryingLookForTimes(0), pos(_pos), speed(_speed) { neighbors.reserve(32); }

		LAddress::L3Type carrier;
		int transmissionAmount;
		int tryingLookForTimes;
		SimTime transmitAt;
		SimTime lookForAt;
		Coord pos;
		Coord speed;
		std::vector<LAddress::L3Type> neighbors;
	};

	/** @name STAG related variables and containers. */
	///@{
	int nodeNum;
	int linkNum;
	int downloaderNum;
	int slotNum;  ///< prediction time slot number, the result of slotSpan*slotNum is prediction period.
	int slotSpan; ///< unit prediction time slot span measured in millisecond.
	std::vector<int> downloaderArray;
	std::map<int, int> downloaderTable;
	std::map<int, int> remainingTable;
	std::map<int, int> playTable;
	std::map<int, int> demandingAmountTable; ///< avoid prefetched offset exceeding the whole content size.
	std::list<LinkTuple> linkTuples;
	///@}

	/** @name relay related variables. */
	///@{
	int ackMsgNum; ///< record how many transmissions have finished to recalculate cooperative scheme again.
	int activeSlotNum; ///< store the number of slot that actually transmit data.
	int prefetchCompletedNum;  ///< the number of downloaders whose prefetch process has completed.
	int prefetchNecessaryNum;  ///< the number of downloaders who need to prefetch data.
	SimTime prevSlotStartTime; ///< store the time when transmission in previous slot is started.
	SimTime executeSTAGNextAt; ///< store the time when to execute the next time.
	bool distributionActive;   ///< whether is providing downloading service currently.
	///@}

	/** @name carrier related variables. */
	///@{
	bool noticeRelayEntering; ///< pay attention to the event that the neighbor of co-downloader enters RSU's communication range.
	///@}

	LAddress::L3Type brokenDownloader;  ///< the downloader who is disconnected from.

	/** @name performance consideration. */
	///@{
	int distributeVLinkBytesOnce; ///< how many bytes measured in link layer to distribute to vehicle once in transmission.
	int distributeRLinkBytesOnce; ///< how many bytes measured in link layer to distribute to other RSU once in transmission.
	int distributeVApplBytesOnce; ///< how many bytes measured in application layer to distribute to vehicle once in transmission.
	int distributeRApplBytesOnce; ///< how many bytes measured in application layer to distribute to other RSU once in transmission.
	SimTime distributeVPeriod;    ///< period to handle self message distributeVEvt.
	SimTime distributeRPeriod;    ///< period to handle self message distributeREvt.
	///@}

	SimTime schemeSwitchInterval; ///< interval from current time to the time schemeSwitchEvt happens(this interval is time-varying).
	SimTime lookForCarrierPeriod; ///< period to handle self message lookForCarrierEvt.
	SimTime wiredTxDuration;  ///< transmission delay of a wired packet.

	cMessage *distributeVEvt; ///< self message used to periodically distribute data to vehicle.
	cMessage *distributeREvt; ///< self message used to periodically distribute data to other RSU.
	cMessage *distributeCEvt; ///< self message used to periodically distribute data to carrier.
	cMessage *schemeSwitchEvt;    ///< self message used to handle with transmission scheme switch in different time slot.
	cMessage *segmentAdvanceEvt;  ///< self message used to handle with segment advance in same time slot.
	cMessage *prefetchRequestEvt; ///< self message used to handle with when to send the prefetch request.
	cMessage *lookForCarrierEvt;  ///< self message used to handle with looking for carrier periodically if there is no proper carrier before.
	cMessage *linkBrokenEvt;  ///< self message used to handle with communication link broken event.

	cQueue prefetchMsgQueue; ///< the queue of prefetch messages.

	DownloaderItems activeDownloaders; ///< downloaders who are contained in current transmission scheme.
	std::list<SchemeTuple> rsuSchemeList; ///< describe how this RSU transmit in each slot.
	std::list<SchemeTuple>::iterator itRSL; ///< an iterator used to iterate container rsuSchemeList.
	SchemeItems schemeItemList; ///< describe how each vehicle transmit in each slot.
	std::map<LAddress::L3Type, LAddress::L3Type> relays; ///< a map from a relay's identifier to its downloader's identifier.
	std::map<LAddress::L3Type, DownloaderInfo*> downloaders; ///< a map from a downloader's identifier to all its related info.
	std::map<LAddress::L3Type, DownloaderInfo*>::iterator itDL; ///< an iterator used to traverse container downloaders.
	std::map<LAddress::L3Type, CoDownloaderInfo*> coDownloaders; ///< a map from an co-downloader to its mobility and neighbor information.
	std::map<LAddress::L3Type, CoDownloaderInfo*>::iterator itCDL; ///< an iterator used to traverse container coDownloaders.
};

#endif /* __CONTENTRSU_H__ */
