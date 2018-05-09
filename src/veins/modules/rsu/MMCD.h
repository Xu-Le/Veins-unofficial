//
// Copyright (C) 2018 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __MMCD_H__
#define __MMCD_H__

#include "veins/modules/rsu/BaseRSU.h"
#include "veins/modules/application/ContentStatisticCollector.h"

/**
 * @brief A concrete RSU class which provides content download service.
 *
 * @author Xu Le
 * @ingroup applLayer
 * @see BaseRSU
 */
class MMCD: public BaseRSU
{
public:
	/** @brief The message kinds content RSU uses. */
	enum MMCDMsgKinds {
		DISTRIBUTE_EVT = LAST_BASE_RSU_MESSAGE_KIND,
		FETCH_REQUEST_EVT,
		SCHEDULE_EVT,
		LINK_BROKEN_EVT,
		LAST_MMCD_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	MMCD() : BaseRSU() {}
	MMCD(unsigned stacksize) : BaseRSU(stacksize) {}
	virtual ~MMCD() {}
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

	/** @brief call-back method of receiving beacon message. */
	void onBeacon(BeaconMessage *beaconMsg) override;
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg) override;
	/** @brief call-back method of receiving content message. */
	void onContent(ContentMessage *contentMsg) override;
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg) override;

	/** @brief perform MMCD flow scheduling. */
	void _flowScheduling();
	/** @brief fetch proper amount data from the content server for distributing in a short time. */
	void _sendFetchingRequest(const LAddress::L3Type downloader, const int curFetchStartOffset);
	/** @brief filling cooperative notification message, and then send it to neighbor RSU. */
	void _sendCooperativeNotification(const LAddress::L3Type downloader, ContentMessage *reportMsg);
	/** @brief send link break acknowledgment to the downloader. */
	void _sendLinkBreakAcknowledgment(const LAddress::L3Type downloader, const LAddress::L3Type receiver);

	/** @brief erase the downloader safely to avoid memory leak. */
	void __eraseDownloader(const LAddress::L3Type downloader);

private:
	/** @brief The class to store downloader's information. */
	class DownloaderInfo
	{
	public:
		explicit DownloaderInfo(int t, int c);

		bool fetchingActive;
		bool notifiedLinkBreak;
		bool sentCoNotification;
		bool noticeEntering;
		int totalContentSize;
		int cacheStartOffset;
		int cacheEndOffset;
		int distributedOffset;
		int curFetchEndOffset;
		int acknowledgedOffset;
		int remainingDataAmount;
		int consumingRate;
		Segment _lackOffset; ///< internal variable, head node of segment list, thus the whole list can be cleared when its destructor automatically called.
		Segment *lackOffset; ///< external variable, use lackOffset = lackOffset->next to iterate the segment list.
	};

	int noticeEnteringNum; ///< pay attention to the event that the co-downloader enters RSU's communication range.
	int activeDownloaderNum; ///< the number of downloaders who are downloading currently.
	LAddress::L3Type brokenDownloader; ///< the downloader who is disconnected from.
	LAddress::L3Type scheduledReceiver;   ///< the receiver who is scheduled in function _flowScheduling().
	LAddress::L3Type scheduledDownloader; ///< the downloader who is scheduled in function _flowScheduling().

	int distThreshold;
	/** @name performance consideration. */
	///@{
	int fetchApplBytesOnce; ///< how many bytes measured in application layer to fetch from content server once.
	int distributeLinkBytesOnce; ///< how many bytes measured in link layer to distribute to vehicle once in transmission.
	int distributeApplBytesOnce; ///< how many bytes measured in application layer to distribute to vehicle once in transmission.
	SimTime distributePeriod;    ///< period to handle self message distributeEvt.
	///@}
	SimTime wiredTxDuration;  ///< transmission delay of a wired packet.
	SimTime slotSpan;         ///< flow scheduling time slot span.
	SimTime slotEndAt;        ///< the ended time instant of current flow scheduling period.

	cMessage *distributeEvt;  ///< self message used to periodically distribute data to vehicle.
	cMessage *fetchRequestEvt; ///< self message used to handle with when to send the prefetch request.
	cMessage *scheduleEvt;    ///< self message used to handle with periodic flow scheduling event.
	cMessage *linkBrokenEvt;  ///< self message used to handle with communication link broken event.

	cQueue fetchMsgQueue; ///< the queue of fetching messages.

	std::map<LAddress::L3Type, DownloaderInfo*> downloaders; ///< a map from a downloader's identifier to all its related info.
	std::map<LAddress::L3Type, DownloaderInfo*>::iterator itDL; ///< an iterator used to traverse container downloaders.
	std::set<LAddress::L3Type> coDownloaders; ///< a set of co-downloaders.
	std::set<LAddress::L3Type>::iterator itCDL; ///< an iterator used to traverse container coDownloaders.
};

#endif /* __MMCD_H__ */
