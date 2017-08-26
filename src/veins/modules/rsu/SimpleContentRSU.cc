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

#include "veins/modules/rsu/SimpleContentRSU.h"

Define_Module(SimpleContentRSU);

void SimpleContentRSU::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		noticeEnteringNum = 0;
		activeDownloaderNum = 0;

		fetchApplBytesOnce = 500 * 1472; // fetchPacketsOnce == 500
		distributeLinkBytesOnce = 20 * (headerLength + dataLengthBits) / 8; // distributeLinkPacketsOnce == 20
		distributeApplBytesOnce = 20 * dataLengthBits / 8;
		wiredTxDuration = SimTime((wiredHeaderLength + 160) * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6

		fetchRequestEvt = new cMessage("fetch request evt", SimpleContentRSUMsgKinds::FETCH_REQUEST_EVT);
		fetchRequestEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at content server
		linkBrokenEvt = new cMessage("link broken evt", SimpleContentRSUMsgKinds::LINK_BROKEN_EVT);
	}
}

void SimpleContentRSU::finish()
{
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		cancelAndDelete(itDL->second->distributeEvt);
		delete itDL->second;
	}
	downloaders.clear();

	cancelAndDelete(fetchRequestEvt);
	cancelAndDelete(linkBrokenEvt);

	BaseRSU::finish();
}

void SimpleContentRSU::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case SimpleContentRSUMsgKinds::DISTRIBUTE_EVT:
	{
		for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		{
			DownloaderInfo *downloaderInfo = itDL->second; // alias
			if (downloaderInfo->distributedAt == simTime()) // self message aims to this downloader
			{
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
					EV << "there is no data to send currently, check later.\n";
					scheduleAt(downloaderInfo->distributedAt, downloaderInfo->distributeEvt);
					return;
				}

				WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
				ASSERT( wsm != nullptr );
				DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

				dataMsg->setIsLast(false);
				dataMsg->setReceiver(itDL->first);
				dataMsg->setDownloader(itDL->first);
				// estimate transmission rate between self and receiver only taking relative distance into consideration
				Coord &recevierPos = vehicles[itDL->first]->pos;
				Coord &receiverSpeed = vehicles[itDL->first]->speed;
				int distance = RoutingUtils::_length(curPosition, recevierPos);
				ASSERT( distance < 250 );
				int estimatedRate = ContentUtils::rateTable[distance/5];
				EV << "estimated rate between myself and receiver [" << itDL->first << "] is " << 128*estimatedRate << " bytes per second.\n";
				if (distance >= 200 && !downloaderInfo->notifiedLinkBreak && (curPosition.x - recevierPos.x)*receiverSpeed.x < 0)
				{
					EV << "the communication link between downloader and RSU will break soon, notify downloader to fetch its downloading status.\n";
					WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
					ASSERT( wsm != nullptr );
					ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

					notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_DIRECT);
					notifyMsg->setReceiver(itDL->first);
					notifyMsg->setDownloader(itDL->first);

					sendWSM(notifyMsg);

					downloaderInfo->notifiedLinkBreak = true;
				}

				if (undistributedAmount > distributeApplBytesOnce) // has enough data to filling a normal data packet
				{
					dataMsg->setBytesNum(distributeApplBytesOnce);
					// dataMsg->addBitLength(8 * distributeLinkBytesOnce);
					ContentStatisticCollector::globalDirectFlux += distributeApplBytesOnce;
					downloaderInfo->distributedOffset += distributeApplBytesOnce;
					distributePeriod = activeDownloaderNum*SimTime(distributeLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
					downloaderInfo->distributedAt = simTime() + distributePeriod;
					if (distance < 200 || (curPosition.x - recevierPos.x)*receiverSpeed.x > 0)
						scheduleAt(downloaderInfo->distributedAt, downloaderInfo->distributeEvt);
					else
						--activeDownloaderNum;
				}
				else // (undistributedAmount > 0) // has data but not enough to filling a normal data packet
				{
					EV << "send the half-filled data packet.\n";
					int totalLinkBytes = ContentUtils::calcLinkBytes(undistributedAmount, headerLength/8, dataLengthBits/8);
					dataMsg->setBytesNum(undistributedAmount);
					// dataMsg->addBitLength(8 * totalLinkBytes);
					ContentStatisticCollector::globalDirectFlux += undistributedAmount;
					downloaderInfo->distributedOffset = downloaderInfo->cacheEndOffset;
					distributePeriod = activeDownloaderNum*SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
					--activeDownloaderNum;
				}
				EV << "distribute vehicle period is " << distributePeriod.dbl() << "s.\n";
				EV << "downloader [" << itDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << ".\n";
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayedDown(dataMsg, distributePeriod);
				break;
			}
		}
		break;
	}
	case SimpleContentRSUMsgKinds::FETCH_REQUEST_EVT:
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(fetchMsgQueue.pop());
		sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
		EV << "send downloader [" << wiredMsg->getDownloader() << "]'s fetching request to content server.\n";
		if (fetchMsgQueue.getLength() > 0)
			scheduleAt(simTime() + wiredTxDuration, fetchRequestEvt);
		break;
	}
	case SimpleContentRSUMsgKinds::LINK_BROKEN_EVT:
	{
		__eraseDownloader(brokenDownloader);
		break;
	}
	default:
		BaseRSU::handleSelfMsg(msg);
	}
}

void SimpleContentRSU::handleWiredMsg(WiredMessage *wiredMsg)
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

	if (!downloaderInfo->distributeEvt->isScheduled())
	{
		downloaderInfo->distributedAt = simTime();
		scheduleAt(downloaderInfo->distributedAt, downloaderInfo->distributeEvt);
		++activeDownloaderNum;
	}

	if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET) // it is the last data packet, indicating fetching progress has finished
	{
		EV << "its content prefetch progress has finished.\n";
		downloaderInfo->fetchingActive = false;
	}
}

void SimpleContentRSU::handleRSUMsg(WiredMessage *wiredMsg, int direction)
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
		downloaderInfo = new DownloaderInfo(wiredMsg->getContentSize());
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

void SimpleContentRSU::onBeacon(BeaconMessage *beaconMsg)
{
	BaseRSU::onBeacon(beaconMsg);

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias

	if (noticeEnteringNum > 0)
	{
		for (itCDL = coDownloaders.begin(); itCDL != coDownloaders.end(); ++itCDL)
		{
			if (sender == *itCDL && downloaders[sender]->noticeEntering && RoutingUtils::_length(curPosition, beaconMsg->getSenderPos()) < 200)
			{
				EV << "noticing vehicle [" << sender << "] is entering communication range.\n";

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *discoveryMsg = dynamic_cast<ContentMessage*>(wsm);

				discoveryMsg->setControlCode(ContentMsgCC::DOWNLOADER_DISCOVERY);
				discoveryMsg->setReceiver(sender);
				discoveryMsg->setDownloader(sender);

				sendWSM(discoveryMsg);

				--noticeEnteringNum;
				downloaders[sender]->noticeEntering = false;
				break;
			}
		}
	}
}

void SimpleContentRSU::onRouting(RoutingMessage *routingMsg)
{
	EV << "SimpleContentRSUs don't react to routing messages since they don't help routings.\n";
}

void SimpleContentRSU::onContent(ContentMessage *contentMsg)
{
	EV << "rsu[" << myAddr - RSU_ADDRESS_OFFSET << "]: " << "onContent!\n";

	LAddress::L3Type downloader = contentMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	switch (contentMsg->getControlCode())
	{
	case ContentMsgCC::CONTENT_REQUEST: // it is a request message from a downloader
	{
		if ( downloaders.find(downloader) != downloaders.end() ) // update old record
		{
			EV << "    downloader [" << downloader << "] is an old downloader, update its info.\n";
			downloaderInfo = downloaders[downloader];
			downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
		}
		else
		{
			// insert new record
			EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
			downloaderInfo = new DownloaderInfo(contentMsg->getContentSize());
			downloaderInfo->curFetchEndOffset = fetchApplBytesOnce;
			if (downloaderInfo->curFetchEndOffset > downloaderInfo->totalContentSize)
				downloaderInfo->curFetchEndOffset = downloaderInfo->totalContentSize;
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
		}
		break;
	}
	case ContentMsgCC::CONTENT_RESPONSE: // it is a response message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
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
	case ContentMsgCC::DOWNLOADER_DISCOVERY: // it is a downloader discovery message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::DISCOVERY_RESPONSE: // it is a response to relay discovery message from relay vehicle
	{
		if (contentMsg->getNeighbors().empty() == false) // received response is yes, otherwise still paying attention to entering vehicles
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
		}
		break;
	}
	default:
		EV_WARN << "Unknown control code " << contentMsg->getControlCode() << ", ignore it." << std::endl;
	}
}

void SimpleContentRSU::onData(DataMessage* dataMsg)
{
	EV << "rsu[" << myAddr - RSU_ADDRESS_OFFSET << "]: " << "onData!\n";
}

void SimpleContentRSU::_sendFetchingRequest(const LAddress::L3Type downloader, const int curFetchStartOffset)
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

void SimpleContentRSU::_sendCooperativeNotification(const LAddress::L3Type downloader, ContentMessage *reportMsg)
{
	DownloaderInfo *downloaderInfo = downloaders[downloader];
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
	coNotifyMsg->setPosition(reportMsg->getPosition());
	coNotifyMsg->setSpeed(reportMsg->getSpeed());
	coNotifyMsg->addBitLength(wiredHeaderLength);
	sendDelayed(coNotifyMsg, SimTime::ZERO, reportMsg->getSpeed().x < 0 ? westOut : eastOut);

	downloaderInfo->sentCoNotification = true;
}

void SimpleContentRSU::__eraseDownloader(const LAddress::L3Type downloader)
{
	if ((itDL = downloaders.find(downloader)) != downloaders.end())
	{
		cancelAndDelete(itDL->second->distributeEvt);
		delete itDL->second;
		downloaders.erase(itDL);
	}
}

/////////////////////////    internal class implementations    /////////////////////////
SimpleContentRSU::DownloaderInfo::DownloaderInfo(int t) : fetchingActive(false), notifiedLinkBreak(false), sentCoNotification(false), noticeEntering(false),
		totalContentSize(t), cacheStartOffset(-1), cacheEndOffset(0), distributedOffset(0), curFetchEndOffset(0), acknowledgedOffset(0),
		distributedAt(), _lackOffset(), lackOffset(&_lackOffset)
{
	distributeEvt = new cMessage("distribute evt", SimpleContentRSUMsgKinds::DISTRIBUTE_EVT);
	distributeEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at vehicle
	_lackOffset.begin = 0;
	_lackOffset.end = totalContentSize;
}
