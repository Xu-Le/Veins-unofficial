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

#ifndef __CONTENTCLIENT_H__
#define __CONTENTCLIENT_H__

#include "veins/modules/wave/BaseWaveApplLayer.h"

#define STRANGER      1
#define RELAY         2
#define CARRIER       3

#define CACHE_TIME_BEFORE_PLAY    3

/**
 * @brief File Content Client.
 *
 * @author Xu Le
 * @ingroup applLayer
 * @see BaseWaveApplLayer
 * @see ContentRSU
 * @see BaseStation
 * @see ContentServer
 */
class ContentClient : public BaseWaveApplLayer
{
public:
	/** @brief The message kinds routing layer uses. */
	enum ContentClientMsgKinds {
		RELAY_EVT = LAST_WAVE_APPL_MESSAGE_KIND,
		FORWARD_EVT,
		SCHEME_SWITCH_EVT,
		SEGMENT_ADVANCE_EVT,
		REPORT_STATUS_EVT,
		DATA_CONSUMPTION_EVT,
		REQUEST_TIMEOUT_EVT,
		INTERRUPT_TIMEOUT_EVT,
		LINK_BROKEN_EVT,
		LAST_CONTENT_CLIENT_MESSAGE_KIND
	};

	/** @name constructor, destructor. */
	///@{
	ContentClient() : BaseWaveApplLayer() {}
	ContentClient(unsigned stacksize) : BaseWaveApplLayer(stacksize) {}
	virtual ~ContentClient() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief Called every time a message arrives. */
	void handleSelfMsg(cMessage *msg) override;

	/** @brief Handle wireless cellular incoming messages. */
	void handleCellularMsg(CellularMessage *cellularMsg) override;

	/** @brief call-back method of receiving beacon message. */
	void onBeacon(BeaconMessage *beaconMsg) override;
	/** @brief call-back method of receiving routing message(aims to unicast protocols). */
	void onRouting(RoutingMessage *routingMsg) override;
	/** @brief call-back method of receiving warning message(aims to broadcast and geocast protocols). */
	void onWarning(WarningMessage *warningMsg) override;
	/** @brief call-back method of receiving content message(aims to download application). */
	void onContent(ContentMessage *contentMsg) override;
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg) override;

	/** @brief call a content request for certain size determined by contentPlanList. */
	void callContent(int size) override;

	/** @brief relay or carrier cache data segment for downloader. */
	void _cacheDataSegment(const int downloader, const int startOffset, const int endOffset);
	/** @brief delete cached data segment that has been acknowledged by downloader and reset pointer DownloaderInfo->cacheOffset. */
	void _adjustCachedSegment();
	/** @brief handle with transmission scheme switch preparation. */
	void _prepareSchemeSwitch();
	/** @brief correct planned transmission offset determined by RSU according to current cached segments. */
	void _correctPlannedOffset();
	/** @brief filling downloading status report message, and then send it to RSU. */
	void _reportDownloadingStatus(const int contentMsgCC, const LAddress::L3Type receiver);
	/** @brief cut off cellular connection, stop downloading data from cellular network. */
	void _closeCellularConnection();

private:
	/** @brief Data consuming rate of some common video quality kinds. */
	enum VideoQuailty {
		_QCIF = 30720, // 240kbps
		_CIF = 89600,  // 700kbps
		_VGA = 217600, // 1700kbps
		_480P = 230400,  // 1800kbps
		_720P = 448000,  // 3500kbps
		_1080P = 1088000 // 8500kbps
	};
	/** @brief The class to store other downloader's information. */
	class DownloaderInfo
	{
	public:
		DownloaderInfo(int t) : myRole(STRANGER), totalContentSize(t), distributedOffset(0), acknowledgedOffset(0),
				notifiedLinkBreak(false), _cacheOffset(), cacheOffset(&_cacheOffset) {}

		int myRole;
		int totalContentSize;
		int distributedOffset;
		int acknowledgedOffset;
		bool notifiedLinkBreak;
		SimTime distributedAt;
		Segment _cacheOffset; ///< internal variable, head node of segment list, thus the whole list can be cleared when its destructor automatically called.
		Segment *cacheOffset; ///< external variable, use cacheOffset = cacheOffset->next to iterate the segment list.
	};
	/** @brief The class to store self downloading information. */
	class DownloadingInfo
	{
	public:
		DownloadingInfo() : totalContentSize(-1), availableOffset(-1), consumedOffset(-1), consumingRate(-1), segmentNum(0) {}

		void reset();
		void eraseSegment();
		void insertSegment(int startOffset, int endOffset);
		void unionSegment();
		void lackSegment(Segment *lackOffsets);
		double getInterruptedTime();

		int totalContentSize;
		int availableOffset;
		int consumedOffset;
		int consumingRate;
		int segmentNum;
		SimTime requestAt;  ///< statistic, the time made the content request.
		SimTime completeAt; ///< statistic, the time completed the downloading process.
		SimTime consumingBeginAt; ///< statistic, the time consuming process began.
		SimTime consumingEndAt;   ///< statistic, the time consuming process ended.
		std::list<std::pair<int /* start offset */, int /* end offset */> > segments;
		std::list<std::pair<int, int> >::iterator itS; // static iterator used to read or find.
		std::list<std::pair<int, int> >::iterator itD; // dynamic iterator used to insert or erase.

	private:
		DownloadingInfo(const DownloadingInfo&);
		DownloadingInfo& operator=(const DownloadingInfo&);
	};

	BaseStation *baseStation;
	cGate *baseStationGate;

	int cellularHeaderLength; ///< length of the cellular packet header measured in bits.
	int cellularBitsRate; ///< data transmission rate measured in bps of cellular radio.

	bool startConsuming; ///< whether the consuming process has started.
	bool cellularDownloading; ///< whether is downloading from cellular network currently.
	bool waitingDistribution; ///< whether is waiting for RSU's downloading service.
	bool encounteredDownloader; ///< whether has encountered the downloader who is the carried data belongs to.
	LAddress::L3Type carriedDownloader; ///< the downloader who is the carried data belongs to.
	LAddress::L3Type brokenDownloader;  ///< the downloader who is disconnected from.
	int slotSpan; ///< unit prediction time slot span measured in millisecond.
	SimTime prevSlotStartTime; ///< store the time when transmission in previous slot is started.

	/** @name performance consideration. */
	///@{
	int relayLinkBytesOnce; ///< how many bytes measured in link layer to relay to downloader once in transmission.
	int relayApplBytesOnce; ///< how many bytes measured in application layer to relay to downloader once in transmission.
	SimTime relayPeriod;    ///< period to handle self message relayEvt.
	SimTime schemeSwitchInterval; ///< interval from current time to the time schemeSwitchEvt happens(this interval is time-varying).
	///@}

	SimTime requestTimeoutDuration;   ///< when the request timer elapsed this duration, timeout event happens.
	SimTime interruptTimeoutDuration; ///< when the interrupt timer elapsed this duration, timeout event happens.

	cMessage *relayEvt;            ///< self message used to handle with relay data packets to downloader.
	cMessage *forwardEvt;          ///< self message used to handle with forward data packets to downloader.
	cMessage *schemeSwitchEvt;     ///< self message used to handle with transmission scheme switch in different time slot.
	cMessage *segmentAdvanceEvt;   ///< self message used to handle with segment advance in same time slot.
	cMessage *reportStatusEvt;     ///< self message used to handle with report downloading status to RSU.
	cMessage *dataConsumptionEvt;  ///< self message used to handle with data consumption process after a slot elapsed.
	cMessage *requestTimeoutEvt;   ///< self message used to handle with content request timeout event(this happens when the vehicle is not in any RSU's communication range area).
	cMessage *interruptTimeoutEvt; ///< self message used to handle with download interrupt timeout event.
	cMessage *linkBrokenEvt;       ///< self message used to handle with communication link broken event.

	DownloadingInfo downloadingStatus; ///< store self downloading status information.
	std::list<SchemeTuple> schemeList; ///< describe how this vehicle transmit in each slot.
	std::list<SchemeTuple>::iterator itSL; ///< an iterator used to iterate container schemeList.
	std::map<LAddress::L3Type, DownloaderInfo*> downloaders; ///< a map from a downloader's identifier to all my related its info.
	std::map<LAddress::L3Type, DownloaderInfo*>::iterator itDL; ///< a iterator used to traverse container downloaders.
};

#endif /* __CONTENTCLIENT_H__ */
