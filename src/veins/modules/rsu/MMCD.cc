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

#include "veins/modules/rsu/MMCD.h"

Define_Module(MMCD);

void MMCD::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		noticeEnteringNum = 0;

		int _slotSpan = par("slotSpan").longValue();
		slotSpan = SimTime(_slotSpan, SIMTIME_MS);
		double vehSpeed = par("vehSpeed").longValue(); // max allowable vehicular speed measured in m/s
		vehSpeed /= 3.6;
		distThreshold = 250 - 2*vehSpeed;

		fetchApplBytesOnce = 500 * 1472; // fetchPacketsOnce == 500
		distributeLinkBytesOnce = 10 * (headerLength + dataLengthBits) / 8; // distributeLinkPacketsOnce == 20
		distributeApplBytesOnce = 10 * dataLengthBits / 8;
		wiredTxDuration = SimTime((wiredHeaderLength + 160) * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6

		distributeEvt = new cMessage("distribute evt", MMCDMsgKinds::DISTRIBUTE_EVT);
		distributeEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at vehicle
		scheduleEvt = new cMessage("schedult evt", MMCDMsgKinds::SCHEDULE_EVT);
		scheduleEvt->setSchedulingPriority(1);
		fetchRequestEvt = new cMessage("fetch request evt", MMCDMsgKinds::FETCH_REQUEST_EVT);
		fetchRequestEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at content server
		linkBrokenEvt = new cMessage("link broken evt", MMCDMsgKinds::LINK_BROKEN_EVT);
	}
}

void MMCD::finish()
{
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		delete itDL->second;
	downloaders.clear();

	cancelAndDelete(distributeEvt);
	cancelAndDelete(scheduleEvt);
	cancelAndDelete(fetchRequestEvt);
	cancelAndDelete(linkBrokenEvt);

	BaseRSU::finish();
}

void MMCD::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case MMCDMsgKinds::DISTRIBUTE_EVT:
	{
		if ((itDL = downloaders.find(scheduledDownloader)) == downloaders.end() || (itV = vehicles.find(scheduledReceiver)) == vehicles.end())
			return;

		DownloaderInfo *downloaderInfo = itDL->second; // alias
		int undistributedAmount = downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset; // alias
		if (!downloaderInfo->fetchingActive && undistributedAmount < 3*distributeApplBytesOnce) // need to fetch more data
		{
			downloaderInfo->curFetchEndOffset += fetchApplBytesOnce;
			if (downloaderInfo->curFetchEndOffset > downloaderInfo->totalContentSize)
				downloaderInfo->curFetchEndOffset = downloaderInfo->totalContentSize;
			if (downloaderInfo->curFetchEndOffset > downloaderInfo->cacheEndOffset)
			{
				EV << "undistributed data is not adequate, current fetching start offset: "
						<< downloaderInfo->cacheEndOffset << ", end offset: " << downloaderInfo->curFetchEndOffset << ".\n";
				_sendFetchingRequest(itDL->first, downloaderInfo->cacheEndOffset);
			}
		}

		if (undistributedAmount == 0)
		{
			if (downloaderInfo->distributedOffset < downloaderInfo->totalContentSize)
			{
				EV << "there is no data to send currently, check later.\n";
				scheduleAt(simTime() + SimTime(100, SIMTIME_MS), distributeEvt);
			}
			return;
		}

		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(scheduledReceiver);
		dataMsg->setDownloader(itDL->first);
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		// estimate transmission rate between self and receiver only taking relative distance into consideration
		Coord &recevierPos = itV->second->pos;
		Coord &receiverSpeed = itV->second->speed;
		int distance = RoutingUtils::_length(curPosition, recevierPos);
		ASSERT( distance < 250 );
		int estimatedRate = ContentUtils::rateTable[distance/5];
		if (scheduledReceiver != scheduledDownloader) // relay cannot send and receive simutaneously
		{
			estimatedRate *= 2;
			estimatedRate /= 3;
		}
		EV << "distance to receiver [" << scheduledReceiver << "] is " << distance << "m, estimated rate is " << 128*estimatedRate << " bytes per second.\n";
		if (distance >= distThreshold && !downloaderInfo->notifiedLinkBreak && (curPosition.x - recevierPos.x)*receiverSpeed.x < 0)
		{
			EV << "the communication link between downloader/relay and RSU will break soon, notify downloader to fetch its downloading status.\n";
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

			notifyMsg->setControlCode(scheduledReceiver == scheduledDownloader ? ContentMsgCC::LINK_BREAK_DIRECT : ContentMsgCC::LINK_BREAK_RR);
			notifyMsg->setReceiver(scheduledReceiver);
			notifyMsg->setDownloader(itDL->first);

			sendWSM(notifyMsg);

			downloaderInfo->notifiedLinkBreak = true;
		}

		if (undistributedAmount > distributeApplBytesOnce) // has enough data to filling a normal data packet
		{
			dataMsg->setBytesNum(distributeApplBytesOnce);
			// dataMsg->addBitLength(8 * distributeLinkBytesOnce);
			if (scheduledReceiver == scheduledDownloader)
				ContentStatisticCollector::globalDirectFlux += distributeApplBytesOnce;
			else
				ContentStatisticCollector::globalRelayFlux += distributeApplBytesOnce;
			distributePeriod = SimTime(distributeLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
			if (simTime() + distributePeriod < slotEndAt)
			{
				dataMsg->setIsLast(false);
				scheduleAt(simTime() + distributePeriod, distributeEvt);
				downloaderInfo->distributedOffset += distributeApplBytesOnce;
			}
			else
			{
				EV << "there is not enough time to transmit data in current slot, send a dummy last packet.\n";

				dataMsg->setIsLast(true);
				dataMsg->setBytesNum(0);
				sendDelayedDown(dataMsg, SimTime::ZERO);
				return;
			}
		}
		else // (undistributedAmount > 0) // has data but not enough to filling a normal data packet
		{
			dataMsg->setIsLast(true);
			EV << "send the half-filled data packet.\n";
			int totalLinkBytes = ContentUtils::calcLinkBytes(undistributedAmount, headerLength/8, dataLengthBits/8);
			dataMsg->setBytesNum(undistributedAmount);
			// dataMsg->addBitLength(8 * totalLinkBytes);
			if (scheduledReceiver == scheduledDownloader)
				ContentStatisticCollector::globalDirectFlux += undistributedAmount;
			else
				ContentStatisticCollector::globalRelayFlux += undistributedAmount;
			distributePeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
			if (simTime() + distributePeriod < slotEndAt)
				downloaderInfo->distributedOffset = downloaderInfo->cacheEndOffset;
			else
			{
				EV << "there is not enough time to transmit data in current slot, send a dummy last packet.\n";

				dataMsg->setBytesNum(0);
				sendDelayedDown(dataMsg, SimTime::ZERO);
				return;
			}
		}
		EV << "distribute vehicle period is " << distributePeriod.dbl() << "s.\n";
		EV << "downloader [" << itDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << ".\n";
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		sendDelayedDown(dataMsg, distributePeriod);
		break;
	}
	case MMCDMsgKinds::FETCH_REQUEST_EVT:
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(fetchMsgQueue.pop());
		sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
		EV << "send downloader [" << wiredMsg->getDownloader() << "]'s fetching request to content server.\n";
		if (fetchMsgQueue.getLength() > 0)
			scheduleAt(simTime() + wiredTxDuration, fetchRequestEvt);
		break;
	}
	case MMCDMsgKinds::SCHEDULE_EVT:
	{
		_flowScheduling();
		break;
	}
	case MMCDMsgKinds::LINK_BROKEN_EVT:
	{
		__eraseDownloader(brokenDownloader);
		break;
	}
	default:
		BaseRSU::handleSelfMsg(msg);
	}
}

void MMCD::handleWiredMsg(WiredMessage *wiredMsg)
{
	ASSERT( wiredMsg->getControlCode() == WiredMsgCC::NORMAL_DATA_PACKET || wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET );

	LAddress::L3Type downloader = wiredMsg->getDownloader();  // alias
	if ((itDL = downloaders.find(downloader)) == downloaders.end())
		return;

	DownloaderInfo *downloaderInfo = itDL->second;
	if (downloaderInfo->cacheStartOffset == -1)
		downloaderInfo->cacheStartOffset = wiredMsg->getCurOffset() - wiredMsg->getBytesNum();
	downloaderInfo->cacheEndOffset = wiredMsg->getCurOffset();
	ContentStatisticCollector::globalStorageAmount += wiredMsg->getBytesNum();
	EV << "downloader [" << downloader << "]'s cache end offset updates to " << downloaderInfo->cacheEndOffset << " bytes.\n";

	if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET) // it is the last data packet, indicating fetching progress has finished
	{
		EV << "its content prefetch progress has finished.\n";
		downloaderInfo->fetchingActive = false;
		if (!scheduleEvt->isScheduled())
			scheduleAt(simTime(), scheduleEvt);
	}
}

void MMCD::handleRSUMsg(WiredMessage *wiredMsg, int direction)
{
	ASSERT( wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_NOTIFICATION || wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT );

	LAddress::L3Type downloader = wiredMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	int outGate = direction == 1 ? westOut : eastOut;
	if (wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_NOTIFICATION)
	{
		EV << "vehicle [" << downloader << "] is a co-downloader, insert its info.\n";
		coDownloaders.insert(downloader);

		EV << "downloader [" << downloader << "] is a new downloader, insert its info.\n";
		downloaderInfo = new DownloaderInfo(wiredMsg->getContentSize(), wiredMsg->getBytesNum()); // reuse function name, actually is consuming rate
		downloaderInfo->noticeEntering = true;
		downloaderInfo->cacheEndOffset = downloaderInfo->cacheStartOffset = wiredMsg->getStartOffset();
		downloaderInfo->distributedOffset = wiredMsg->getStartOffset();
		downloaderInfo->acknowledgedOffset = wiredMsg->getStartOffset();
		downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
		++noticeEnteringNum;

		WiredMessage *coAckMsg = new WiredMessage("co-ack", WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT);
		coAckMsg->setControlCode(WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT);
		coAckMsg->setDownloader(downloader);
		coAckMsg->addBitLength(wiredHeaderLength);
		sendDelayed(coAckMsg, SimTime::ZERO, outGate);
	}
	else // (wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT)
	{
		EV << "receiving cooperative acknowledgment from neighbor RSU.\n";
		__eraseDownloader(downloader);
	}
}

void MMCD::onBeacon(BeaconMessage *beaconMsg)
{
	BaseRSU::onBeacon(beaconMsg);

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias

	if (noticeEnteringNum > 0)
	{
		for (itCDL = coDownloaders.begin(); itCDL != coDownloaders.end(); ++itCDL)
		{
			if (sender == *itCDL && downloaders[sender]->noticeEntering)
			{
				EV << "noticing vehicle [" << sender << "] is entering communication range.\n";

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *discoveryMsg = dynamic_cast<ContentMessage*>(wsm);

				discoveryMsg->setControlCode(ContentMsgCC::DOWNLOADER_DISCOVERY);
				discoveryMsg->setReceiver(sender);
				discoveryMsg->setDownloader(sender);

				sendWSM(discoveryMsg);

				downloaders[sender]->noticeEntering = false;
				--noticeEnteringNum;
				break;
			}
		}
	}
}

void MMCD::onRouting(RoutingMessage *routingMsg)
{
	EV << "MMCD don't react to routing messages since they don't help routings.\n";
}

void MMCD::onContent(ContentMessage *contentMsg)
{
	EV << "rsu[" << myAddr - RSU_ADDRESS_OFFSET << "]: " << "onContent!\n";

	LAddress::L3Type downloader = contentMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	switch (contentMsg->getControlCode())
	{
	case ContentMsgCC::CONTENT_REQUEST: // it is a request message from a downloader
	{
		ASSERT( downloaders.find(downloader) == downloaders.end() );

		EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
		downloaderInfo = new DownloaderInfo(contentMsg->getContentSize(), contentMsg->getConsumingRate());
		downloaderInfo->curFetchEndOffset = std::min(fetchApplBytesOnce, downloaderInfo->totalContentSize);
		downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));

		// response to the request vehicle
		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);

		responseMsg->setControlCode(ContentMsgCC::CONTENT_RESPONSE);
		responseMsg->setDownloader(downloader);

		sendWSM(responseMsg);

		EV << "current fetching start offset: 0, end offset: " << downloaderInfo->curFetchEndOffset << ".\n";
		_sendFetchingRequest(downloader, 0);
		break;
	}
	case ContentMsgCC::CONTENT_RESPONSE: // it is a response message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::ACKNOWLEDGEMENT:
	{
		if ((itDL = downloaders.find(downloader)) != downloaders.end())
		{
			downloaderInfo = itDL->second;
			LAddress::L3Type sender = contentMsg->getSenderAddress();  // alias
			if (vehicles.find(downloader) == vehicles.end() || (sender != downloader && downloaderInfo->sentCoNotification))
			{
				EV << "downloader [" << downloader << "] has driven away RSU's communication area, erase its info.\n";
				__eraseDownloader(downloader);
			}
			else
			{
				cancelEvent(linkBrokenEvt);

				EV << "downloader [" << downloader << "]'s acknowledged offset updates to " << contentMsg->getReceivedOffset() << ".\n";
				downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
				downloaderInfo->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
				downloaderInfo->_lackOffset = contentMsg->getLackOffset();
				downloaderInfo->lackOffset = &downloaderInfo->_lackOffset;
			}
		}
		break;
	}
	case ContentMsgCC::DOWNLOADING_COMPLETED: // it is a downloading completed notification message from vehicle
	{
		EV << "    downloader [" << downloader << "]'s downloading process has completed, erase its info.\n";
		// notify content server to release corresponding resources
		WiredMessage *wiredMsg = new WiredMessage("wired");
		wiredMsg->setControlCode(WiredMsgCC::COMPLETE_DOWNLOADING);
		wiredMsg->setDownloader(downloader);
		wiredMsg->addBitLength(wiredHeaderLength);
		sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
		// release ourself resources corresponding to this downloader
		__eraseDownloader(downloader);
		coDownloaders.erase(downloader);
		break;
	}
	case ContentMsgCC::LINK_BREAK_DIRECT: // it is a link break notification - the communication link between downloader and RSU will break soon
	{
		_sendCooperativeNotification(downloader, contentMsg);
		break;
	}
	case ContentMsgCC::LINK_BREAK_DR: // it is a link break notification - the communication link between downloader and relay will break soon
	{
		if (contentMsg->getConsumingRate() > 0)
			_sendCooperativeNotification(downloader, contentMsg);
		else
			EV << "it is a link break notification sent by relay, downloader has not response it, ignore it.\n";
		break;
	}
	case ContentMsgCC::LINK_BREAK_RR: // it is a link break notification - the communication link between relay and RSU will break soon
	{
		if (contentMsg->getConsumingRate() > 0)
		{
			if (contentMsg->getSenderAddress() != downloader)
				_sendCooperativeNotification(downloader, contentMsg);
			else
				EV << "it is a direct response from downloader, relay will rebroadcast it, just wait.\n";
		}
		else
			EV << "it is a link break notification rebroadcast by relay, downloader has not response it, ignore it.\n";
		break;
	}
	case ContentMsgCC::LINK_BROKEN_DR:
	{
		EV << "it is a link broken notification sent by relay, indicating distribution process become inactive.\n";
		__eraseDownloader(downloader);
		break;
	}
	case ContentMsgCC::DOWNLOADER_DISCOVERY: // it is a downloader discovery message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::DISCOVERY_RESPONSE: // it is a response to relay discovery message from relay vehicle
	{
		coDownloaders.erase(downloader);

		if (contentMsg->getReceivedOffset() == contentMsg->getContentSize())
		{
			EV << "    downloader [" << downloader << "]'s downloading process has completed, erase its info.\n";
			__eraseDownloader(downloader);
			return;
		}

		downloaderInfo = downloaders[downloader];
		downloaderInfo->cacheEndOffset = downloaderInfo->cacheStartOffset = contentMsg->getReceivedOffset();
		downloaderInfo->distributedOffset = contentMsg->getReceivedOffset();
		downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
		downloaderInfo->_lackOffset = contentMsg->getLackOffset();
		downloaderInfo->lackOffset = &downloaderInfo->_lackOffset;
		downloaderInfo->curFetchEndOffset = downloaderInfo->distributedOffset + fetchApplBytesOnce;
		if (downloaderInfo->curFetchEndOffset > downloaderInfo->totalContentSize)
			downloaderInfo->curFetchEndOffset = downloaderInfo->totalContentSize;

		EV << "current fetching start offset: " << downloaderInfo->distributedOffset << ", end offset: " << downloaderInfo->curFetchEndOffset << ".\n";
		_sendFetchingRequest(downloader, downloaderInfo->distributedOffset);
		break;
	}
	default:
		EV_WARN << "Unknown control code " << contentMsg->getControlCode() << ", ignore it." << std::endl;
	}
}

void MMCD::onData(DataMessage* dataMsg)
{
	EV << "rsu[" << myAddr - RSU_ADDRESS_OFFSET << "]: " << "onData!\n";
}

void MMCD::_flowScheduling()
{
	scheduledDownloader = -1;
	double minDeadline = 1000000.0;
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		double deadline = itDL->second->remainingDataAmount / itDL->second->consumingRate;
		if (minDeadline > deadline && vehicles.find(itDL->first) != vehicles.end())
		{
			minDeadline = deadline;
			scheduledDownloader = itDL->first;
		}
	}
	if (scheduledDownloader == -1)
	{
		EV << "no downloader is scheduled.\n";
		ASSERT(!distributeEvt->isScheduled());
		return;
	}
	scheduledReceiver = scheduledDownloader;
	VehicleInfo *info = vehicles[scheduledDownloader];
	double minDistance = RoutingUtils::_length(info->pos, curPosition);
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		if (itV->first == scheduledDownloader || downloaders.find(itV->first) != downloaders.end())
			continue;
		double distR2V = RoutingUtils::_length(curPosition, itV->second->pos);
		double distD2V = RoutingUtils::_length(info->pos, itV->second->pos);
		// ensure cooperator has the same direction as the downloader
		if (minDistance > distR2V && distD2V < 150.0 && itV->second->speed.x * info->speed.x > 0)
		{
			minDistance = distR2V;
			scheduledReceiver = itV->first;
		}
	}
	EV << "scheduled receiver: " << scheduledReceiver << ", scheduled downloader: " << scheduledDownloader << std::endl;
	scheduleAt(simTime(), distributeEvt);
	slotEndAt = simTime() + slotSpan;
	scheduleAt(slotEndAt, scheduleEvt);
	if (scheduledReceiver != scheduledDownloader) // deliver assistance command to the scheduled receiver
	{
		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

		notifyMsg->setControlCode(ContentMsgCC::SCHEME_DISTRIBUTION);
		notifyMsg->setReceiver(scheduledReceiver);
		notifyMsg->setDownloader(scheduledDownloader);
		notifyMsg->setContentSize(downloaders[scheduledDownloader]->totalContentSize);
		notifyMsg->setConsumingRate(downloaders[scheduledDownloader]->consumingRate);

		sendWSM(notifyMsg);
	}
}

void MMCD::_sendFetchingRequest(const LAddress::L3Type downloader, const int curFetchStartOffset)
{
	DownloaderInfo *downloaderInfo = downloaders[downloader];

	WiredMessage *wiredMsg = new WiredMessage("fetch-request", WiredMsgCC::START_TRANSMISSION);
	wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
	wiredMsg->setDownloader(downloader);
	wiredMsg->setContentSize(downloaderInfo->totalContentSize);
	wiredMsg->setStartOffset(curFetchStartOffset);
	wiredMsg->setEndOffset(downloaderInfo->curFetchEndOffset);
	wiredMsg->addBitLength(wiredHeaderLength + 160); // 5*sizeof(int) * 8
	fetchMsgQueue.insert(wiredMsg);
	if (!fetchRequestEvt->isScheduled())
		scheduleAt(simTime(), fetchRequestEvt);

	downloaderInfo->fetchingActive = true;
}

void MMCD::_sendCooperativeNotification(const LAddress::L3Type downloader, ContentMessage *reportMsg)
{
	_sendLinkBreakAcknowledgment(downloader, reportMsg->getSenderAddress());

	if ((itDL = downloaders.find(downloader)) == downloaders.end())
		return;
	DownloaderInfo *downloaderInfo = itDL->second;
	if (downloaderInfo->sentCoNotification)
	{
		EV << "had sent cooperative notification to neighbor RSU, do nothing.\n";
		return;
	}

	EV << "getting current downloading status of the downloader, notify neighbor RSU.\n";
	WiredMessage *coNotifyMsg = new WiredMessage("co-notification", WiredMsgCC::COOPERATIVE_NOTIFICATION);
	coNotifyMsg->setControlCode(WiredMsgCC::COOPERATIVE_NOTIFICATION);
	coNotifyMsg->setDownloader(downloader);
	coNotifyMsg->setContentSize(downloaderInfo->totalContentSize);
	coNotifyMsg->setStartOffset(reportMsg->getReceivedOffset());
	coNotifyMsg->setEndOffset(downloaderInfo->cacheEndOffset);
	coNotifyMsg->setBytesNum(reportMsg->getConsumingRate()); // reuse function name
	coNotifyMsg->setPosition(reportMsg->getPosition());
	coNotifyMsg->setSpeed(reportMsg->getSpeed());
	coNotifyMsg->addBitLength(wiredHeaderLength);
	sendDelayed(coNotifyMsg, SimTime::ZERO, reportMsg->getSpeed().x < 0 ? westOut : eastOut);

	downloaderInfo->sentCoNotification = true;
}

void MMCD::_sendLinkBreakAcknowledgment(const LAddress::L3Type downloader, const LAddress::L3Type receiver)
{
	WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
	ASSERT( wsm != nullptr );
	ContentMessage *ackMsg = dynamic_cast<ContentMessage*>(wsm);

	ackMsg->setControlCode(ContentMsgCC::LINK_BREAK_ACK);
	ackMsg->setReceiver(receiver);
	ackMsg->setDownloader(downloader);

	sendWSM(ackMsg);
	EV << "sending link break acknowledgment to the downloader.\n";
}

void MMCD::__eraseDownloader(const LAddress::L3Type downloader)
{
	if ((itDL = downloaders.find(downloader)) != downloaders.end())
	{
		delete itDL->second;
		downloaders.erase(itDL);
		if (noticeEnteringNum > 0 && (itCDL = coDownloaders.find(downloader)) != coDownloaders.end())
		{
			coDownloaders.erase(itCDL);
			--noticeEnteringNum;
		}
	}
}

/////////////////////////    internal class implementations    /////////////////////////
MMCD::DownloaderInfo::DownloaderInfo(int t, int c) : fetchingActive(false), notifiedLinkBreak(false), sentCoNotification(false), noticeEntering(false),
		totalContentSize(t), cacheStartOffset(-1), cacheEndOffset(0), distributedOffset(0), curFetchEndOffset(0), acknowledgedOffset(0),
		remainingDataAmount(0), consumingRate(c), _lackOffset(), lackOffset(&_lackOffset)
{
	_lackOffset.begin = 0;
	_lackOffset.end = totalContentSize;
}
