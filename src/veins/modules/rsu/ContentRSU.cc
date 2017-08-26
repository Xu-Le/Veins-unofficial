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

#include "veins/modules/rsu/ContentRSU.h"
#if !DEBUG_STAG
#include <fstream>
#endif

Define_Module(ContentRSU);

void ContentRSU::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		nodeNum = linkNum = downloaderNum = 0;
		slotNum = par("slotNum").longValue();
		slotSpan = par("slotSpan").longValue();

		executeSTAGNextAt = SimTime::ZERO;
		noticeEnteringNum = 0;
		unitOffsetPerSecond = 3000000/8; // equals to cellular rate

		distributeVLinkBytesOnce = 10 * (headerLength + dataLengthBits) / 8; // distributeVLinkPacketsOnce == 10
		distributeVApplBytesOnce = 10 * dataLengthBits / 8;
		distributeVPeriod = SimTime::ZERO;
		lookForCarrierPeriod = SimTime(3, SIMTIME_S);
		wiredTxDuration = SimTime((wiredHeaderLength + 160) * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6

		distributeVEvt = new cMessage("distribute v evt", ContentRSUMsgKinds::DISTRIBUTE_V_EVT);
		distributeCEvt = new cMessage("distribute c evt", ContentRSUMsgKinds::DISTRIBUTE_C_EVT);
		distributeVEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at vehicle
		distributeCEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at carrier
		schemeSwitchEvt = new cMessage("scheme switch evt", ContentRSUMsgKinds::SCHEME_SWITCH_EVT);
		segmentAdvanceEvt = new cMessage("segment advance evt", ContentRSUMsgKinds::SEGMENT_ADVANCE_EVT);
		prefetchRequestEvt = new cMessage("prefetch request evt", ContentRSUMsgKinds::PREFETCH_REQUEST_EVT);
		prefetchRequestEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at content server
		lookForCarrierEvt = new cMessage("look for carrier evt", ContentRSUMsgKinds::LOOK_FOR_CARRIER_EVT);
		linkBrokenEvt = new cMessage("link broken evt", ContentRSUMsgKinds::LINK_BROKEN_EVT);
	}
}

void ContentRSU::finish()
{
	relays.clear();
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		delete itDL->second;
	downloaders.clear();

	cancelAndDelete(distributeVEvt);
	cancelAndDelete(distributeCEvt);
	cancelAndDelete(schemeSwitchEvt);
	cancelAndDelete(segmentAdvanceEvt);
	cancelAndDelete(prefetchRequestEvt);
	cancelAndDelete(lookForCarrierEvt);
	cancelAndDelete(linkBrokenEvt);

	BaseRSU::finish();
}

void ContentRSU::handleSelfMsg(cMessage *msg)
{
	bool isSegmentAdvanceEvt = true;
	switch (msg->getKind())
	{
	case ContentRSUMsgKinds::SCHEME_SWITCH_EVT:
	{
		if (downloaders.find(itRSL->downloader) == downloaders.end())
		{
			EV << "communication link with this downloader has broken, skip current planned scheme.\n";
			rsuSchemeList.clear();
			return;
		}

		EV << "switched to the scheme in slot " << itRSL->slot << ", distributed offset starts at " << itRSL->offset->begin << " bytes.\n";
		downloaders[itRSL->downloader]->distributedOffset = itRSL->offset->begin;
		prevSlotStartTime = simTime();
		isSegmentAdvanceEvt = false;
	}
	case ContentRSUMsgKinds::SEGMENT_ADVANCE_EVT:
	{
		if ((itDL = downloaders.find(itRSL->downloader)) == downloaders.end())
		{
			rsuSchemeList.clear();
			return;
		}

		if (isSegmentAdvanceEvt)
		{
			EV << "switched to advance the next segment, distributed offset starts at " << itRSL->offset->begin << " bytes.\n";
			downloaders[itRSL->downloader]->distributedOffset = itRSL->offset->begin;
		}
	}
	case ContentRSUMsgKinds::DISTRIBUTE_V_EVT:
	{
		if ((itDL = downloaders.find(itRSL->downloader)) == downloaders.end() || (itV = vehicles.find(itRSL->receiver)) == vehicles.end())
		{
			rsuSchemeList.clear();
			return;
		}

		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(itRSL->receiver);
		dataMsg->setDownloader(itRSL->downloader);
		DownloaderInfo *downloaderInfo = downloaders[itRSL->downloader];
		// estimate transmission rate between self and receiver only taking relative distance into consideration
		Coord &receiverPos = itV->second->pos;
		Coord &receiverSpeed = itV->second->speed;
		int distance = RoutingUtils::_length(curPosition, receiverPos);
		ASSERT( distance < 250 );
		if (distance >= 225 && (curPosition.x - receiverPos.x)*receiverSpeed.x < 0)
		{
			EV << "distance " << distance << " is larger than 225, erase this downloader's info.\n";
			delete dataMsg;

			__eraseDownloader(itRSL->downloader);

			_prepareSchemeSwitch();
			return;
		}
		int estimatedRate = ContentUtils::rateTable[distance/5];
		EV << "estimated rate between myself and receiver [" << itRSL->receiver << "] is " << 128*estimatedRate << " bytes per second.\n";
		if (itRSL->offset->end - itRSL->offset->begin > distributeVApplBytesOnce)
		{
			dataMsg->setIsLast(false);
			dataMsg->setBytesNum(distributeVApplBytesOnce);
			// dataMsg->addBitLength(8*distributeVLinkBytesOnce);
			if (itRSL->receiver == itRSL->downloader)
				ContentStatisticCollector::globalDirectFlux += distributeVApplBytesOnce;
			else
				ContentStatisticCollector::globalRelayFlux += distributeVApplBytesOnce;
			distributeVPeriod = SimTime(distributeVLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
			EV << "distribute vehicle period is " << distributeVPeriod.dbl() << "s.\n";
			if (simTime() + distributeVPeriod - prevSlotStartTime < SimTime(slotSpan, SIMTIME_MS))
			{
				scheduleAt(simTime() + distributeVPeriod, distributeVEvt);
				itRSL->offset->begin += distributeVApplBytesOnce;
				downloaderInfo->distributedOffset += distributeVApplBytesOnce;
			}
			else
			{
				EV << "there is not enough time to transmit data in current slot, send a dummy last packet.\n";

				dataMsg->setIsLast(true);
				dataMsg->setBytesNum(0);
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayedDown(dataMsg, SimTime::ZERO);
				if (itRSL->receiver == itRSL->downloader)
				{
					brokenDownloader = itRSL->downloader;
					scheduleAt(simTime() + SimTime(20, SIMTIME_MS), linkBrokenEvt);
				}

				_prepareSchemeSwitch();
				return;
			}
		}
		else
		{
			dataMsg->setIsLast(itRSL->offset->next == NULL);
			int lastPktAmount = itRSL->offset->end - itRSL->offset->begin; // alias
			int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, headerLength/8, dataLengthBits/8);
			dataMsg->setBytesNum(lastPktAmount);
			// dataMsg->addBitLength(8*totalLinkBytes);
			if (itRSL->receiver == itRSL->downloader)
				ContentStatisticCollector::globalDirectFlux += lastPktAmount;
			else
				ContentStatisticCollector::globalRelayFlux += lastPktAmount;
			distributeVPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
			EV << "distribute vehicle period is " << distributeVPeriod.dbl() << "s.\n";
			if (simTime() + distributeVPeriod - prevSlotStartTime < SimTime(slotSpan, SIMTIME_MS))
				downloaderInfo->distributedOffset += lastPktAmount;
			else
			{
				EV << "there is not enough time to transmit data in current slot, send a dummy last packet.\n";

				dataMsg->setIsLast(true);
				dataMsg->setBytesNum(0);
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayedDown(dataMsg, SimTime::ZERO);
				if (itRSL->receiver == itRSL->downloader)
				{
					brokenDownloader = itRSL->downloader;
					scheduleAt(simTime() + SimTime(20, SIMTIME_MS), linkBrokenEvt);
				}

				_prepareSchemeSwitch();
				return;
			}

			if (itRSL->offset->next == NULL)
				_prepareSchemeSwitch();
			else // this rarely happens
			{
				itRSL->offset = itRSL->offset->next; // prepare to transmit the next segment
				EV << "prepare to advance to the next segment.\n";
				scheduleAt(simTime() + distributeVPeriod, segmentAdvanceEvt);
			}
		}
		EV << "downloader [" << itRSL->downloader << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		sendDelayedDown(dataMsg, distributeVPeriod);
		break;
	}
	case ContentRSUMsgKinds::DISTRIBUTE_C_EVT:
	{
		for (itCDL = coDownloaders.begin(); itCDL != coDownloaders.end(); ++itCDL)
		{
			if (itCDL->second->transmitAt == simTime()) // self message aims to this co-downloader
			{
				DownloaderInfo *downloaderInfo = downloaders[itCDL->first]; // alias
				CoDownloaderInfo *coDownloaderInfo = itCDL->second; // alias

				WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
				ASSERT( wsm != nullptr );
				DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

				dataMsg->setReceiver(coDownloaderInfo->carrier);
				dataMsg->setDownloader(itCDL->first);
				// estimate transmission rate between self and receiver only taking relative distance into consideration
				int distance = RoutingUtils::_length(curPosition, vehicles[coDownloaderInfo->carrier]->pos);
				ASSERT( distance < 250 );
				int estimatedRate = ContentUtils::rateTable[distance/5];
				EV << "estimated rate between myself and carrier [" << coDownloaderInfo->carrier << "] is " << 128*estimatedRate << " bytes per second.\n";
				SimTime distributeCPeriod(SimTime::ZERO);
				if (downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset > distributeVApplBytesOnce)
				{
					dataMsg->setIsLast(distance >= 200);
					dataMsg->setBytesNum(distributeVApplBytesOnce);
					// dataMsg->addBitLength(8*distributeVLinkBytesOnce);
					distributeCPeriod = SimTime(distributeVLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
					if (distance < 200)
						scheduleAt(simTime() + distributeCPeriod, distributeCEvt);
					else
					{
						if (!downloaderInfo->noticeEntering)
							++noticeEnteringNum;
						downloaderInfo->noticeEntering = true;
					}
					downloaderInfo->distributedOffset += distributeVApplBytesOnce;
					coDownloaderInfo->transmitAt = simTime() + distributeCPeriod;
				}
				else
				{
					dataMsg->setIsLast(true);
					int lastPktAmount = downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset; // alias
					int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, headerLength/8, dataLengthBits/8);
					dataMsg->setBytesNum(lastPktAmount);
					// dataMsg->addBitLength(8*totalLinkBytes);
					distributeCPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
					if (!downloaderInfo->noticeEntering)
						++noticeEnteringNum;
					downloaderInfo->noticeEntering = true;
					downloaderInfo->distributedOffset += lastPktAmount;
				}
				EV << "distribute carrier period is " << distributeCPeriod.dbl() << "s.\n";
				EV << "downloader [" << itCDL->first << "]'s distributed(carrier) offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayedDown(dataMsg, distributeCPeriod);
				break;
			}
		}
		break;
	}
	case ContentRSUMsgKinds::PREFETCH_REQUEST_EVT:
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(prefetchMsgQueue.pop());
		sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
		EV << "send downloader [" << wiredMsg->getDownloader() << "]'s prefetching request to content server.\n";
		if (prefetchMsgQueue.getLength() > 0)
			scheduleAt(simTime() + wiredTxDuration, prefetchRequestEvt);
		break;
	}
	case ContentRSUMsgKinds::LOOK_FOR_CARRIER_EVT:
	{
		for (itCDL = coDownloaders.begin(); itCDL != coDownloaders.end(); ++itCDL)
		{
			if (itCDL->second->lookForAt == simTime()) // self message aims to this co-downloader
			{
				EV << "wait time elapsed, try to choose a carrier again.\n";

				bool hasCarrier = false;
				if (!distributeCEvt->isScheduled())
					hasCarrier = selectCarrier(itCDL->first);
				if (!hasCarrier && ++itCDL->second->tryingLookForTimes < 3)
				{
					EV << "there is no proper carrier, wait a certain time and check later.\n";
					itCDL->second->lookForAt = simTime() + lookForCarrierPeriod;
					scheduleAt(itCDL->second->lookForAt, lookForCarrierEvt);
				}
				break;
			}
		}
		break;
	}
	case ContentRSUMsgKinds::LINK_BROKEN_EVT:
	{
		__eraseDownloader(brokenDownloader);
		break;
	}
	default:
		BaseRSU::handleSelfMsg(msg);
	}
}

void ContentRSU::handleWiredMsg(WiredMessage *wiredMsg)
{
	ASSERT( wiredMsg->getControlCode() == WiredMsgCC::NORMAL_DATA_PACKET || wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET );

	LAddress::L3Type downloader = wiredMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = downloaders[downloader];
	if (downloaderInfo->cacheStartOffset == -1)
		downloaderInfo->cacheStartOffset = wiredMsg->getCurOffset() - wiredMsg->getBytesNum();
	downloaderInfo->cacheEndOffset = wiredMsg->getCurOffset();
	ContentStatisticCollector::globalStorageAmount += wiredMsg->getBytesNum();
	EV << "downloader [" << downloader << "]'s cache end offset updates to " << downloaderInfo->cacheEndOffset << " bytes.\n";

	if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET) // it is the last data packet, indicating prefetch progress has finished
	{
		if (coDownloaders.find(downloader) == coDownloaders.end())
		{
			EV << "its content prefetch progress has finished, check if all downloaders' prefetch process has completed.\n";

			if (++prefetchCompletedNum < prefetchNecessaryNum)
			{
				EV << "there exists downloaders whose prefetch process has not completed.\n";
				return;
			}
			else
				EV << "all downloaders' prefetch process has completed, now begin to distribute transmission scheme to relays.\n";

			_broadcastTransmissionScheme();
		}
		else
		{
			EV << "its content prefetch progress has finished, now begin to transmit data to the carrier.\n";

			coDownloaders[downloader]->transmitAt = simTime();
			scheduleAt(simTime(), distributeCEvt);
		}
	}
}

void ContentRSU::handleRSUMsg(WiredMessage *wiredMsg, int direction)
{
	ASSERT( wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_NOTIFICATION || wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT );

	LAddress::L3Type downloader = wiredMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	int outGate = direction == 1 ? westOut : eastOut;
	if (wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_NOTIFICATION)
	{
		EV << "vehicle [" << downloader << "] is a co-downloader, insert its info.\n";
		CoDownloaderInfo *coDownloaderInfo = new CoDownloaderInfo(wiredMsg->getPosition(), wiredMsg->getSpeed());
		NeighborItems &neighborItems = wiredMsg->getNeighbors();
		for (NeighborItems::iterator itNI = neighborItems.begin(); itNI != neighborItems.end(); ++itNI)
		{
			EV << "vehicle [" << *itNI << "] is a neighbor of co-downloader, append it to co-downloader's neighbor vector.\n";
			coDownloaderInfo->neighbors.push_back(*itNI);
		}
		std::sort(coDownloaderInfo->neighbors.begin(), coDownloaderInfo->neighbors.end());
		coDownloaders.insert(std::pair<LAddress::L3Type, CoDownloaderInfo*>(downloader, coDownloaderInfo));

		EV << "downloader [" << downloader << "] is a co-downloader, insert its info.\n";
		downloaderInfo = new DownloaderInfo(wiredMsg->getContentSize(), wiredMsg->getBytesNum()); // reuse function name, actually is consuming rate
		downloaderInfo->cacheEndOffset = downloaderInfo->cacheStartOffset = wiredMsg->getStartOffset();
		downloaderInfo->distributedOffset = wiredMsg->getStartOffset();
		downloaderInfo->acknowledgedOffset = wiredMsg->getStartOffset();
		downloaderInfo->remainingDataAmount = downloaderInfo->acknowledgedOffset - wiredMsg->getCurOffset(); // reuse function name, actually is consumed offset
		downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));

		WiredMessage *coAckMsg = new WiredMessage("co-ack", WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT);
		coAckMsg->setControlCode(WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT);
		coAckMsg->setDownloader(downloader);
		coAckMsg->setStartOffset(wiredMsg->getStartOffset());
		coAckMsg->setEndOffset(wiredMsg->getEndOffset());
		coAckMsg->addBitLength(wiredHeaderLength);
		sendDelayed(coAckMsg, SimTime::ZERO, outGate);

		EV << "now try to choose a carrier.\n";
		bool hasCarrier = false;
		if (!distributeCEvt->isScheduled())
			hasCarrier = selectCarrier(downloader);
		if (!hasCarrier)
		{
			EV << "there is no proper carrier, wait a certain time and check later.\n";
			++noticeEnteringNum;
			downloaderInfo->noticeEntering = true;
			++coDownloaderInfo->tryingLookForTimes;
			coDownloaderInfo->lookForAt = simTime() + lookForCarrierPeriod;
			scheduleAt(coDownloaderInfo->lookForAt, lookForCarrierEvt);
		}
	}
	else // (wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT)
	{
		EV << "receiving cooperative acknowledgment from neighbor RSU.\n";
		// __eraseDownloader(downloader);
	}
}

void ContentRSU::onBeacon(BeaconMessage *beaconMsg)
{
	BaseRSU::onBeacon(beaconMsg);

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias

	if (!downloaders.empty() && (itDL = downloaders.find(sender)) != downloaders.end())
	{
		int distD = RoutingUtils::_length(curPosition, beaconMsg->getSenderPos());
		if (itDL->second->notifiedLinkBreak == false && distD >= 200 && (curPosition.x - beaconMsg->getSenderPos().x)*beaconMsg->getSenderSpeed().x < 0)
		{
			EV << "the communication link between downloader and RSU will break soon, notify downloader to fetch its downloading status.\n";
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

			notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_DIRECT);
			notifyMsg->setReceiver(sender);
			notifyMsg->setDownloader(sender);

			sendWSM(notifyMsg);

			itDL->second->notifiedLinkBreak = true;
		}
	}
	if (!relays.empty() && relays.find(sender) != relays.end())
	{
		LAddress::L3Type downloader = relays[sender];
		if ((itDL = downloaders.find(downloader)) != downloaders.end())
		{
			int distR = RoutingUtils::_length(curPosition, beaconMsg->getSenderPos());
			int distD = 0;
			if ((itV = vehicles.find(downloader)) != vehicles.end())
				distD = RoutingUtils::_length(curPosition, itV->second->pos);
			if (itV == vehicles.end() || distD >= 200)
			{
				if (itDL->second->notifiedLinkBreak == false && distR >= 200 && (curPosition.x - beaconMsg->getSenderPos().x)*beaconMsg->getSenderSpeed().x < 0)
				{
					EV << "the communication link between relay and RSU will break soon, notify downloader to fetch its downloading status.\n";
					WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
					ASSERT( wsm != nullptr );
					ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

					notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_RR);
					notifyMsg->setReceiver(sender);
					notifyMsg->setDownloader(downloader);

					sendWSM(notifyMsg);

					itDL->second->notifiedLinkBreak = true;
				}
				else if (itDL->second->notifiedLinkBreak == true && distR >= 225)
				{
					EV << "downloader [" << downloader << "] has driven away RSU's communication area, erase its info.\n";
					delete itDL->second;
					downloaders.erase(itDL);
				}
			}
		}
	}

	if (noticeEnteringNum > 0)
	{
		for (itCDL = coDownloaders.begin(); itCDL != coDownloaders.end(); ++itCDL)
		{
			if (sender == itCDL->first && downloaders[sender]->noticeEntering)
			{
				EV << "noticing vehicle [" << sender << "] is entering communication range.\n";

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *discoveryMsg = dynamic_cast<ContentMessage*>(wsm);

				discoveryMsg->setControlCode(ContentMsgCC::DOWNLOADER_DISCOVERY);
				discoveryMsg->setReceiver(sender);
				discoveryMsg->setDownloader(itCDL->first);

				sendWSM(discoveryMsg);

				downloaders[sender]->noticeEntering = false;
				--noticeEnteringNum;
				break;
			}
#if 0
			std::vector<LAddress::L3Type> &potentialRelays = itCDL->second->neighbors;
			if (ContentUtils::vectorSearch(potentialRelays, sender))
			{
				EV << "noticing vehicle [" << sender << "] is entering communication range.\n";

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *discoveryMsg = dynamic_cast<ContentMessage*>(wsm);

				discoveryMsg->setControlCode(ContentMsgCC::RELAY_DISCOVERY);
				discoveryMsg->setReceiver(sender);
				discoveryMsg->setDownloader(itCDL->first);

				sendWSM(discoveryMsg);
				break;
			}
#endif
		}
	}
}

void ContentRSU::onRouting(RoutingMessage *routingMsg)
{
	EV << "ContentRSUs don't react to routing messages since they don't help routings.\n";
}

void ContentRSU::onContent(ContentMessage *contentMsg)
{
	EV << "rsu[" << myAddr - RSU_ADDRESS_OFFSET << "]: " << "onContent!\n";

	LAddress::L3Type sender = contentMsg->getSenderAddress();  // alias
	LAddress::L3Type downloader = contentMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	switch (contentMsg->getControlCode())
	{
	case ContentMsgCC::CONTENT_REQUEST: // it is a request message from a downloader
	{
		EV << "downloaders:";
		for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
			EV << ' ' << itDL->first;
		EV << "\n";

		if ( downloaders.find(downloader) != downloaders.end() ) // update old record
		{
			EV << "    downloader [" << downloader << "] is an old downloader, update its info.\n";
			downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
			downloaders[downloader]->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
		}
		else
		{
			bool noDownloader = downloaders.empty();
			if (downloaders.empty()) // insert new record
			{
				EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
				downloaderInfo = new DownloaderInfo(contentMsg->getContentSize(), contentMsg->getConsumingRate());
				downloaderInfo->lackOffset->begin = 0;
				downloaderInfo->lackOffset->end = contentMsg->getContentSize();
				downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
			}
			else
				EV << "    downloader [" << downloader << "] is a hanging downloader, don't insert its info.\n";

			// response to the request vehicle
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);

			responseMsg->setControlCode(ContentMsgCC::CONTENT_RESPONSE);
			responseMsg->setDownloader(downloader);
			if (!noDownloader)
				responseMsg->setReportAt(executeSTAGNextAt - SimTime(500, SIMTIME_MS));
			sendWSM(responseMsg);
			if (!noDownloader)
				return;

			if (vehicles.find(downloader) == vehicles.end()) // insert downloader's mobility information into container vehicles
			{
				EV << "    vehicle [" << downloader << "] is a new vehicle, insert its info.\n";
				VehicleInfo *vehicleInfo = new VehicleInfo(contentMsg->getPosition(), contentMsg->getSpeed(), simTime());
				vehicles.insert(std::pair<LAddress::L3Type, VehicleInfo*>(downloader, vehicleInfo));
			}
			// check if there exists any downloader has driven away RSU's communication area
			for (itDL = downloaders.begin(); itDL != downloaders.end();)
			{
				if (vehicles.find(itDL->first) == vehicles.end() && coDownloaders.find(itDL->first) == coDownloaders.end())
				{
					EV << "downloader [" << itDL->first << "] has driven away RSU's communication area, erase its info.\n";
					delete itDL->second;
					downloaders.erase(itDL++);
				}
				else
					++itDL;
			}

			// use STAG approach to obtain the optimal cooperative transmission scheme
			SimTime calculatingTime = obtainCooperativeScheme();
			ackMsgNum = 0;

			// send prefetch data request to content server
			_sendPrefetchRequest(downloader, 0, downloaderInfo->prefetchDataAmount);
			prefetchCompletedNum = 0;
			prefetchNecessaryNum = 1;
			std::pair<LAddress::L3Type, Coord> downloaderItem(downloader, vehicles[downloader]->speed);
			activeDownloaders.push_back(downloaderItem);
			scheduleAt(simTime() + calculatingTime, prefetchRequestEvt);
		}
		break;
	}
	case ContentMsgCC::CONTENT_RESPONSE: // it is a response message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::SCHEME_DISTRIBUTION: // it is a transmission scheme message distributed by RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::ACKNOWLEDGEMENT:
	{
		if ((itDL = downloaders.find(downloader)) != downloaders.end())
		{
			downloaderInfo = itDL->second;
			if (simTime() - downloaderInfo->acknowledgedAt < SimTime(10, SIMTIME_MS))
			{
				EV << "it is a duplicate acknowledgment message rebroadcast by relay, do nothing.\n";
				return;
			}

			if (vehicles.find(downloader) == vehicles.end() || (sender != downloader && downloaderInfo->sentCoNotification)
				|| (sender == downloader && (curPosition.x - vehicles[downloader]->pos.x)*vehicles[downloader]->speed.x < 0 && RoutingUtils::_length(curPosition, vehicles[downloader]->pos) >= 225))
			{
				EV << "downloader [" << downloader << "] has driven away RSU's communication area, erase its info.\n";
				delete itDL->second;
				downloaders.erase(itDL);
			}
			else
			{
				cancelEvent(linkBrokenEvt);

				EV << "downloader [" << downloader << "]'s acknowledged offset updates to " << contentMsg->getReceivedOffset() << ".\n";
				downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
				downloaderInfo->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
				downloaderInfo->acknowledgedAt = simTime();
				downloaderInfo->_lackOffset = contentMsg->getLackOffset();
				downloaderInfo->lackOffset = &downloaderInfo->_lackOffset;
			}
			EV << "ackMsgNum: " << ackMsgNum+1 << ", activeSlotNum: " << activeSlotNum << ".\n";
			if (++ackMsgNum == activeSlotNum && downloaders.size() - coDownloaders.size() > 0)
			{
				// use STAG approach to obtain the optimal cooperative transmission scheme again
				SimTime calculatingTime = obtainCooperativeScheme(); // activeSlotNum reset inside
				ackMsgNum = 0;

				prefetchNecessaryNum = 0;
				for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
				{
					if ((itV = vehicles.find(itDL->first)) == vehicles.end())
						continue;
					std::pair<LAddress::L3Type, Coord> downloaderItem(itDL->first, itV->second->speed);
					activeDownloaders.push_back(downloaderItem);
					if (itDL->second->cacheEndOffset < itDL->second->totalContentSize && itDL->second->prefetchDataAmount > 0)
					{
						++prefetchNecessaryNum;
						_sendPrefetchRequest(itDL->first, itDL->second->cacheEndOffset, itDL->second->cacheEndOffset + itDL->second->prefetchDataAmount);
					}
				}
				if (prefetchNecessaryNum > 0)
				{
					prefetchCompletedNum = 0;
					scheduleAt(simTime() + calculatingTime, prefetchRequestEvt);
				}
				else // no need to prefetch data
					_broadcastTransmissionScheme();
			}
		}
		break;
	}
	case ContentMsgCC::CARRIER_SELECTION: // it is a carrier chosen notification message from RSU
	{
		EV << "receiving carry response from carrier, now check whether cached data is adequate.\n";

		downloaderInfo = downloaders[downloader];
		CoDownloaderInfo *coDownloaderInfo = coDownloaders[downloader];
		ASSERT( downloaderInfo->distributedOffset == downloaderInfo->cacheStartOffset );
		// distribution offset correction
		int remainingSeconds = downloaderInfo->remainingDataAmount/downloaderInfo->consumingRate - 3*coDownloaderInfo->tryingLookForTimes;
		EV << "downloader [" << downloader << "]'s consuming process remains " << remainingSeconds << "s.\n";
		downloaderInfo->distributedOffset += (coDownloaderInfo->encounterSeconds - remainingSeconds - 4) * unitOffsetPerSecond;
		if (downloaderInfo->distributedOffset < downloaderInfo->cacheStartOffset) // in the best situation, downloader doesn't need cellular network
			downloaderInfo->distributedOffset = downloaderInfo->cacheStartOffset;
		EV << "cache start offset is " << downloaderInfo->cacheStartOffset << ", distributed offset corrects to " << downloaderInfo->distributedOffset << ".\n";
		// check whether cached data is adequate for transmitting to carrier
		// cached segment       [_____________________]-----------------| EOF
		// distribute segment   |----------->|____________|
		if (downloaderInfo->distributedOffset + coDownloaderInfo->transmissionAmount > downloaderInfo->cacheEndOffset)
		{
			// cached segment       [_____________________]-----------------| EOF
			// distribute segment   |----------------------------------------->|____________|
			if (downloaderInfo->distributedOffset >= downloaderInfo->totalContentSize)
			{
				EV << "carrier is useless in this situation, thus don't distribute data to carrier.\n";
				if (!downloaderInfo->noticeEntering)
					++noticeEnteringNum;
				downloaderInfo->noticeEntering = true;
				return;
			}

			EV << "cached data is not adequate, prefetch the lacking portion from content server.\n";

			// cached segment       [_____________________]-----------------| EOF
			// distribute segment   |------------------------------>|____________|
			if (coDownloaderInfo->transmissionAmount > downloaderInfo->totalContentSize - downloaderInfo->distributedOffset)
				coDownloaderInfo->transmissionAmount = downloaderInfo->totalContentSize - downloaderInfo->distributedOffset;

			_sendPrefetchRequest(downloader, std::max(downloaderInfo->cacheEndOffset, downloaderInfo->distributedOffset), downloaderInfo->distributedOffset + coDownloaderInfo->transmissionAmount);
			prefetchCompletedNum = 0;
			prefetchNecessaryNum = 1;
			std::pair<LAddress::L3Type, Coord> downloaderItem(downloader, coDownloaderInfo->speed);
			activeDownloaders.push_back(downloaderItem);
			scheduleAt(simTime(), prefetchRequestEvt);
		}
		else
		{
			EV << "cached data is adequate, now begin to transmit data to the carrier.\n";

			coDownloaders[downloader]->transmitAt = simTime();
			scheduleAt(simTime(), distributeCEvt);
		}
		break;
	}
	case ContentMsgCC::CARRIER_ENCOUNTER:
	{
		EV_WARN << "RSU should not receive a message sent by carrier when it encounter downloader.\n";
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
		ackMsgNum = 0;
		activeSlotNum = 0;
		break;
	}
	case ContentMsgCC::LINK_BREAK_DIRECT: // it is a link break notification - the communication link between downloader and RSU will break soon
	{
		_sendCooperativeNotification(downloader, contentMsg);
		break;
	}
	case ContentMsgCC::LINK_BREAK_DR: // it is a link break notification - the communication link between downloader and relay will break soon
	{
		if (contentMsg->getReceivedOffset() > 0)
			_sendCooperativeNotification(downloader, contentMsg);
		else
			EV << "it is a link break notification sent by relay, downloader has not response it, ignore it.\n";
		break;
	}
	case ContentMsgCC::LINK_BREAK_RR: // it is a link break notification - the communication link between relay and RSU will break soon
	{
		if (contentMsg->getReceivedOffset() > 0)
			_sendCooperativeNotification(downloader, contentMsg);
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
	case ContentMsgCC::RELAY_DISCOVERY: // it is a relay discovery message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::DISCOVERY_RESPONSE: // it is a response to relay discovery message from relay vehicle
	{
		if (contentMsg->getNeighbors().empty()) // received response is no, still paying attention to entering vehicles
			ContentUtils::vectorRemove(coDownloaders[downloader]->neighbors, sender);
		else // received response is yes, now start to execute STAG algorithm
		{
			itCDL = coDownloaders.find(downloader);
			itCDL->second->neighbors.clear();
			delete itCDL->second;
			coDownloaders.erase(itCDL);

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
			downloaderInfo->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
			downloaderInfo->consumingRate = contentMsg->getConsumingRate();
			downloaderInfo->_lackOffset = contentMsg->getLackOffset();
			downloaderInfo->lackOffset = &downloaderInfo->_lackOffset;
#if 0
			if (vehicles.find(downloader) == vehicles.end()) // insert downloader's mobility information into container vehicles
			{
				EV << "    vehicle [" << downloader << "] is a new vehicle, insert its info.\n";
				VehicleInfo *vehicleInfo = new VehicleInfo(contentMsg->getPosition(), contentMsg->getSpeed(), simTime());
				vehicles.insert(std::pair<LAddress::L3Type, VehicleInfo*>(downloader, vehicleInfo));
			}
#endif
			if (downloaders.size() - coDownloaders.size() > 1)
			{
				EV << "RSU is distributing data to other downloaders currently, don't execute STAG algorithm.\n";
				return;
			}

			// use STAG approach to obtain the optimal cooperative transmission scheme
			SimTime calculatingTime = obtainCooperativeScheme();
			ackMsgNum = 0;

			if (downloaderInfo->cacheEndOffset < downloaderInfo->totalContentSize && downloaderInfo->prefetchDataAmount > 0)
			{
				_sendPrefetchRequest(downloader, downloaderInfo->cacheEndOffset, downloaderInfo->cacheEndOffset + downloaderInfo->prefetchDataAmount);
				prefetchCompletedNum = 0;
				prefetchNecessaryNum = 1;
				std::pair<LAddress::L3Type, Coord> downloaderItem(downloader, vehicles[downloader]->speed);
				activeDownloaders.push_back(downloaderItem);
				scheduleAt(simTime() + calculatingTime, prefetchRequestEvt);
			}
			else // no need to prefetch data
				_broadcastTransmissionScheme();
		}
		break;
	}
	case ContentMsgCC::STATUS_QUERY:
	{
		EV << "it is a downloading status query/report message, ignore it.\n";
		break;
	}
	case ContentMsgCC::STATUS_REPORT:
	{
		EV << "it is a downloading status report message from hanging downloader [" << downloader << "], insert its info.\n";
		downloaderInfo = new DownloaderInfo(contentMsg->getContentSize(), contentMsg->getConsumingRate());

		EV << "its available offset is " << contentMsg->getReceivedOffset() << ", its consumed offset is " << contentMsg->getConsumedOffset() << ".\n";
		downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
		downloaderInfo->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
		downloaderInfo->_lackOffset = contentMsg->getLackOffset();
		downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
		break;
	}
	case ContentMsgCC::REBROADCAST_BEACON:
	{
		EV << "it is a rebroadcast beacon message of downloader [" << downloader << "], updates its mobility info if necessary.\n";

		VehicleInfo *vehicleInfo = nullptr;
		if ((itV = vehicles.find(downloader)) != vehicles.end())
			vehicleInfo = itV->second;
		if (vehicleInfo != nullptr && simTime() - vehicleInfo->receivedAt > SimTime(100, SIMTIME_MS)) // it is not a duplicate beacon message
		{
			vehicleInfo->prevPos = vehicleInfo->pos;
			vehicleInfo->prevSpeed = vehicleInfo->speed;
			vehicleInfo->pos = contentMsg->getPosition();
			vehicleInfo->speed = contentMsg->getSpeed();
			vehicleInfo->receivedAt = simTime();
		}
		break;
	}
	default:
		EV_WARN << "Unknown control code " << contentMsg->getControlCode() << ", ignore it." << std::endl;
	}
}

void ContentRSU::onData(DataMessage* dataMsg)
{
	EV << "rsu[" << myAddr - RSU_ADDRESS_OFFSET << "]: " << "onData!\n";

	// findHost()->getDisplayString().updateWith("r=16,green");
	// annotations->scheduleErase(1, annotations->drawLine(dataMsg->getSenderPos(), curPosition, "blue"));
}

SimTime ContentRSU::obtainCooperativeScheme()
{
	Timer algorithmTimer("Algorithm Time Cost: ");

	predictLinkBandwidth();

	STAG graph(nodeNum, linkNum, downloaderNum, slotNum, downloaderArray, downloaderTable, remainingTable, playTable, demandingAmountTable);

	graph.construct(linkTuples);
	graph.undirectify(linkTuples);

	graph.maximumFlow();

	// maintain node ID map from node IDs used by STAG to real vehicle IDs
	std::vector<int> nodeIdMap(vehicles.size()+1, 0);
	size_t k = 0;
	int nodeId = 1;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		nodeIdMap[nodeId++] = itV->first;
	// get prefetching data amount list from STAG
	for (int d = 0; d < downloaderNum; ++d)
	{
		DownloaderInfo *downloaderInfo = downloaders[nodeIdMap[downloaderArray[d]]]; // alias
		downloaderInfo->prefetchDataAmount = graph.prefetchAmountTable[downloaderArray[d]];
		// sum up length of all lack segments that have been cached, for there is no need to prefetch these segments
		Segment *offsets = downloaderInfo->lackOffset; // alias
		int cachedLackingAmount = 0;
		while (offsets != nullptr && offsets->begin < downloaderInfo->cacheEndOffset)
		{
			cachedLackingAmount += std::min(offsets->end, downloaderInfo->cacheEndOffset) - offsets->begin;
			offsets = offsets->next;
		}
		EV << "downloader [" << nodeIdMap[downloaderArray[d]] << "]'s cached end offset is " << downloaderInfo->cacheEndOffset << ", cached lacking amount is " << cachedLackingAmount << ".\n";
		downloaderInfo->prefetchDataAmount -= cachedLackingAmount; // note: downloaderInfo->prefetchDataAmount might become 0 after -=
	}
	// get RSU self scheme list from STAG
	for (k = 0; k < graph.fluxSchemeList[0].size(); ++k)
	{
		STAG::FluxScheme &fluxScheme = graph.fluxSchemeList[0][k]; // alias
		rsuSchemeList.push_back(SchemeTuple(fluxScheme.slot, nodeIdMap[fluxScheme.dest], nodeIdMap[fluxScheme.downloader], fluxScheme.flow));
		// calculate offsets of distributed data in each slot
		_determineSchemeOffsets(rsuSchemeList.back().offset, &fluxScheme.segment, downloaders[nodeIdMap[fluxScheme.downloader]]->lackOffset);
#if INFO_STAG
		EV << "slot: " << rsuSchemeList.back().slot << ", receiver: " << rsuSchemeList.back().receiver << ", downloader: " << rsuSchemeList.back().downloader << ", offsets: ";
		rsuSchemeList.back().offset->print();
#endif
	}
	int lastActiveSlot = 0;
	activeSlotNum = 0;
	relays.clear();
	for (itRSL = rsuSchemeList.begin(); itRSL != rsuSchemeList.end(); ++itRSL)
	{
		if (itRSL->receiver == itRSL->downloader) // direct downloading
		{
			lastActiveSlot = itRSL->slot;
			++activeSlotNum;
		}
		else
			relays.insert(std::pair<LAddress::L3Type, LAddress::L3Type>(itRSL->receiver, itRSL->downloader));
	}
	// get vehicles' scheme item list from STAG
	for (nodeId = 1; nodeId < nodeNum; ++nodeId)
	{
		if (graph.fluxSchemeList[nodeId].empty() == false)
		{
			schemeItemList.insert(std::pair<LAddress::L3Type, std::list<SchemeTuple> >(nodeIdMap[nodeId], std::list<SchemeTuple>()));
			for (k = 0; k < graph.fluxSchemeList[nodeId].size(); ++k)
			{
				STAG::FluxScheme &fluxScheme = graph.fluxSchemeList[nodeId][k]; // alias
				std::list<SchemeTuple> &schemeItem = schemeItemList[nodeIdMap[nodeId]]; // alias
				schemeItem.push_back(SchemeTuple(fluxScheme.slot, nodeIdMap[fluxScheme.dest], nodeIdMap[fluxScheme.downloader], fluxScheme.flow));
				// calculate offsets of distributed data in each slot
				_determineSchemeOffsets(schemeItem.back().offset, &fluxScheme.segment, downloaders[nodeIdMap[fluxScheme.downloader]]->lackOffset);
#if INFO_STAG
				EV << "slot: " << schemeItem.back().slot << ", receiver: " << schemeItem.back().receiver << ", downloader: " << schemeItem.back().downloader << ", offsets: ";
				schemeItem.back().offset->print();
#endif
			}
			if (lastActiveSlot < graph.fluxSchemeList[nodeId].back().slot)
				lastActiveSlot = graph.fluxSchemeList[nodeId].back().slot;
			activeSlotNum += static_cast<int>(graph.fluxSchemeList[nodeId].size());
		}
	}
	executeSTAGNextAt = simTime() + SimTime(lastActiveSlot*slotSpan, SIMTIME_MS);
	EV << "active transmission slot number is " << activeSlotNum << ", the last active slot is " << lastActiveSlot << ".\n";

	graph.destruct();
	linkTuples.clear();
	downloaderArray.clear();
	downloaderTable.clear();
	remainingTable.clear();
	playTable.clear();
	demandingAmountTable.clear();
#if __linux__
	return SimTime(algorithmTimer.elapsed(), SIMTIME_US);
#else
	return SimTime(algorithmTimer.elapsed(), SIMTIME_MS);
#endif
}

void ContentRSU::predictLinkBandwidth()
{
	nodeNum = static_cast<int>(vehicles.size()) + 1;
	linkNum = nodeNum * (nodeNum - 1) / 2;
	downloaderNum = 0;

	std::vector<int> nodeIdMap(vehicles.size()+1, 0); // maintain node ID map from node IDs used by STAG to real vehicle IDs
	std::map<int, int> nodeIdRevMap; // maintain node ID map from real vehicle IDs to node IDs used by STAG
	int nodeId = 1, j = 0;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		nodeIdMap[nodeId] = itV->first;
		nodeIdRevMap.insert(std::pair<int, int>(itV->first, nodeId++));
	}

	std::vector<int> distPadding(nodeNum, 0.0);
	std::vector<Coord> slotPadding(slotNum+2, Coord::ZERO); // the last element stores the previous mobility information
	std::vector<std::vector<int> > nodeDist;
	std::vector<std::vector<Coord> > nodeSpeed;
	std::vector<std::vector<Coord> > nodePos;
	for (nodeId = 0; nodeId < nodeNum; ++nodeId)
	{
		nodeDist.push_back(distPadding);
		nodeSpeed.push_back(slotPadding);
		nodePos.push_back(slotPadding);
	}
	for (j = 0; j <= slotNum; ++j)
	{
		nodeSpeed[0][j] = Coord::ZERO; // RSU has no speed since it cannot move
		nodePos[0][j] = curPosition;   // node 0 is RSU
	}
	for (nodeId = 1; nodeId < nodeNum; ++nodeId) // current position of vehicles
	{
		VehicleInfo *vehicleInfo = vehicles[nodeIdMap[nodeId]];
		nodeSpeed[nodeId][0] = vehicleInfo->speed;
		nodePos[nodeId][0] = vehicleInfo->pos;
		nodeSpeed[nodeId][slotNum+1] = vehicleInfo->prevSpeed;
		nodePos[nodeId][slotNum+1] = vehicleInfo->prevPos;
	}
	_predictVehicleMobility(nodeSpeed, nodePos);

	int srcNode = 0, dstNode = 0;
	for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
	{
		for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
		{
			linkTuples.push_back(LinkTuple(srcNode, dstNode, slotNum));
			memset(linkTuples.back().bandwidth, 0, slotNum*sizeof(int));
			nodeDist[srcNode][dstNode] = RoutingUtils::_length(nodePos[srcNode][0], nodePos[dstNode][0]);
		}
	}

	std::list<LinkTuple>::iterator itLT = linkTuples.begin();
	for (j = 0; j < slotNum; ++j)
	{
		itLT = linkTuples.begin();
		for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
		{
			for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
			{
				int curDist = nodeDist[srcNode][dstNode];
				int nextDist = RoutingUtils::_length(nodePos[srcNode][j+1], nodePos[dstNode][j+1]);
				nodeDist[srcNode][dstNode] = nextDist;
				if (curDist < 250 || nextDist < 250)
				{
					int minIdx = curDist/5, maxIdx = nextDist/5;
					if (minIdx > maxIdx)
						std::swap(minIdx, maxIdx);
					int maxIdx_ = std::min(49, maxIdx); // avoid reading memory outside of the bounds of ContentUtils::rateTable[]
					for (int i = minIdx; i <= maxIdx_; ++i)
					{
						if (i < 25)
							itLT->bandwidth[j] += 115*ContentUtils::rateTable[i]; // due to prediction precision consideration
						else if (i < 30 && i >= 25)
							itLT->bandwidth[j] += 108*ContentUtils::rateTable[i]; // due to prediction precision consideration
						else if (i < 35 && i >= 30)
							itLT->bandwidth[j] += 102*ContentUtils::rateTable[i]; // due to prediction precision consideration
						else
							itLT->bandwidth[j] += 96*ContentUtils::rateTable[i]; // due to prediction precision consideration
					}
					itLT->bandwidth[j] /= maxIdx - minIdx + 1;
				}
				else
					itLT->bandwidth[j] = 0;
				++itLT;
			}
		}
	}

	int downloaderIdx = 0;
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		if (coDownloaders.find(itDL->first) != coDownloaders.end() || nodeIdRevMap.find(itDL->first) == nodeIdRevMap.end())
			continue;
		int downloaderID = nodeIdRevMap[itDL->first]; // downloader's corresponding nodeId used by STAG
		downloaderArray.push_back(downloaderID);
		downloaderTable.insert(std::pair<int, int>(downloaderID, downloaderIdx));
		remainingTable.insert(std::pair<int, int>(downloaderID, itDL->second->remainingDataAmount));
		playTable.insert(std::pair<int, int>(downloaderID, itDL->second->consumingRate));
		Segment *offsets = itDL->second->lackOffset; // alias
		int demandingAmount = 0;
		while (offsets != nullptr)
		{
			demandingAmount += offsets->end - offsets->begin;
			offsets = offsets->next;
		}
		demandingAmountTable.insert(std::pair<int, int>(downloaderID, demandingAmount));
		++downloaderNum;
		++downloaderIdx;
	}
	if (downloaderNum > 1)
		srcNode = 0;

#if !DEBUG_STAG
	std::ofstream fout("STAG_input.txt", std::ios_base::out | std::ios_base::trunc);
	if ( !fout.is_open() )
	{
		error("cannot open file STAG_input.txt!");
	}
	else
	{
		fout << nodeNum << " " << linkNum << " " << downloaderNum << " " << slotNum << "\n\n";
		for (itLT = linkTuples.begin(); itLT != linkTuples.end(); ++itLT)
		{
			fout << itLT->src << " " << itLT->dst;
			for (int j = 0; j < slotNum; ++j)
				fout << " " << itLT->bandwidth[j];
			fout << "\n";
		}
		fout << std::endl;

		for (nodeId = 0; nodeId < nodeNum; ++nodeId)
		{
			fout << nodeId;
			for (int d = 0; d < downloaderIdx; ++d)
			{
				if (nodeId == downloaderArray[d])
					fout << " " << downloaderArray[d] << " " << remainingTable[nodeId];
				else if (nodeId > 0)
					fout << " " << downloaderArray[d] << " 0";
				else // RSU self
					fout << " " << downloaderArray[d] << " 99999999";
			}
			fout << "\n";
		}
		fout << std::endl;

		for (std::map<int, int>::iterator itPT = playTable.begin(); itPT != playTable.end(); ++itPT)
			fout << itPT->first << " " << itPT->second << "\n";
	}
	fout.close();
#endif

#if 1
	EV << nodeNum << " " << linkNum << " " << downloaderNum << " " << slotNum << "\n\n";
	for (itLT = linkTuples.begin(); itLT != linkTuples.end(); ++itLT)
	{
		EV << itLT->src << " " << itLT->dst << " |";
		for (int j = 0; j < slotNum; ++j)
			EV << " " << itLT->bandwidth[j];
		EV << "\n";
	}
	EV << std::endl;

	for (std::map<int, int>::iterator itPT = playTable.begin(); itPT != playTable.end(); ++itPT)
		EV << itPT->first << " " << itPT->second << "\n";
#endif
}

void ContentRSU::_predictVehicleMobility(std::vector<std::vector<Coord> >& vehicleSpeed, std::vector<std::vector<Coord> >& vehiclePos)
{
	int nodeId = 1;
	size_t k = 0;
	double safeGap = 50.0, velocity = 0.0, acceleration = 0.0;
	std::vector<int> frontVehicle(nodeNum, -1);
	std::vector<double> vehicleGap(nodeNum, 1000.0);
	std::vector<Coord> vehicleAcceleration(nodeNum, Coord::ZERO);
	// fill proper value into frontVehicle
	{
		std::vector<std::pair<double, int> > sortPositiveAxis;
		std::vector<std::pair<double, int> > sortNegativeAxis;
		for (nodeId = 1; nodeId < nodeNum; ++nodeId)
		{
			if (vehicleSpeed[nodeId][0].x > 0)
				sortPositiveAxis.push_back(std::pair<double, int>(vehiclePos[nodeId][0].x, nodeId));
			else
				sortNegativeAxis.push_back(std::pair<double, int>(vehiclePos[nodeId][0].x, nodeId));
		}
		std::sort(sortPositiveAxis.begin(), sortPositiveAxis.end(), std::greater<std::pair<double,int> >());
		std::sort(sortNegativeAxis.begin(), sortNegativeAxis.end());
		if (!sortPositiveAxis.empty())
		{
			frontVehicle[sortPositiveAxis[0].second] = -1;
			for (k = 1; k < sortPositiveAxis.size(); ++k)
				frontVehicle[sortPositiveAxis[k].second] = sortPositiveAxis[k-1].second;
		}
		if (!sortNegativeAxis.empty())
		{
			frontVehicle[sortNegativeAxis[0].second] = -1;
			for (k = 1; k < sortNegativeAxis.size(); ++k)
				frontVehicle[sortNegativeAxis[k].second] = sortNegativeAxis[k-1].second;
		}
	}
#if 0
	EV << "front vehicle:";
	for (nodeId = 1; nodeId < nodeNum; ++nodeId)
		EV << ' ' << frontVehicle[nodeId];
	EV << "\n";
#endif

	for (int j = 1; j <= slotNum; ++j)
	{
		for (nodeId = 1; nodeId < nodeNum; ++nodeId)
		{
			if (frontVehicle[nodeId] != -1)
				vehicleGap[nodeId] = RoutingUtils::_length(vehiclePos[nodeId][j-1], vehiclePos[frontVehicle[nodeId]][j-1]);
			if (vehiclePos[nodeId][slotNum+1].squareLength() > Epsilon) // there exists its previous mobility information, otherwise assume its acceleration is zero
			{
				velocity = vehicleSpeed[nodeId][j-1].length();
				if (j == 1)
					acceleration = (velocity*velocity - vehicleSpeed[nodeId][slotNum+1].squareLength())/2/RoutingUtils::_length(vehiclePos[nodeId][0], vehiclePos[nodeId][slotNum+1]);
				else
					acceleration = (velocity*velocity - vehicleSpeed[nodeId][j-2].squareLength())/2/RoutingUtils::_length(vehiclePos[nodeId][j-1], vehiclePos[nodeId][j-2]);
				vehicleAcceleration[nodeId].x = acceleration/velocity * vehicleSpeed[nodeId][j-1].x;
				vehicleAcceleration[nodeId].y = acceleration/velocity * vehicleSpeed[nodeId][j-1].y;
			}

			// adopt free driving or safety braking model to calculate vehicle speed at the next time according to vehicle gap
			if (vehicleGap[nodeId] > safeGap)
			{
				Coord randAcceleration = vehicleAcceleration[nodeId]*(4*dblrand()-2);
				if ((acceleration = randAcceleration.length()) > 2.6)
					randAcceleration *= 2.6/acceleration;
				vehicleSpeed[nodeId][j] = vehicleSpeed[nodeId][j-1] + randAcceleration;
				velocity = vehicleSpeed[nodeId][j].length();
				if (velocity < 16.7)
					vehicleSpeed[nodeId][j] *= 16.7/velocity;
				// else if (velocity > 25.0)
				//	vehicleSpeed[nodeId][j] *= 25.0/velocity;
			}
			else
			{
				if (vehicleAcceleration[nodeId].x * vehicleSpeed[nodeId][j-1].x > 0)
					vehicleAcceleration[nodeId] *= -1;
				vehicleSpeed[nodeId][j] = vehicleSpeed[nodeId][j-1] + vehicleAcceleration[nodeId]*(dblrand()+1)/2;
			}

			vehiclePos[nodeId][j] = vehiclePos[nodeId][j-1] + vehicleSpeed[nodeId][j-1];
		}
	}

#if 0
	for (nodeId = 1; nodeId < nodeNum; ++nodeId)
	{
		EV << "node [" << nodeId << "]'s speed:";
		for (int j = 0; j <= slotNum+1; ++j)
			EV << " (" << vehicleSpeed[nodeId][j].x << ',' << vehicleSpeed[nodeId][j].y << ')';
		EV << "\nnode [" << nodeId << "]'s pos:";
		for (int j = 0; j <= slotNum+1; ++j)
			EV << " (" << vehiclePos[nodeId][j].x << ',' << vehiclePos[nodeId][j].y << ')';
		EV << "\n";
	}
#endif
}

void ContentRSU::_determineSchemeOffsets(Segment *&schemeOffset, Segment *STAGOffset, Segment *lackOffset)
{
	Segment *storeSchemeOffset = schemeOffset; // store the head pointer of segment list
	int passedOffset = 0;
	while (true)
	{
		STAGOffset->begin -= passedOffset;
		STAGOffset->end -= passedOffset;
		while (STAGOffset->begin + lackOffset->begin >= lackOffset->end)
		{
			passedOffset += lackOffset->end - lackOffset->begin;
			STAGOffset->begin -= lackOffset->end - lackOffset->begin;
			STAGOffset->end -= lackOffset->end - lackOffset->begin;
			lackOffset = lackOffset->next;
		}
		schemeOffset->begin = STAGOffset->begin + lackOffset->begin;
		while (STAGOffset->end + lackOffset->begin > lackOffset->end)
		{
			schemeOffset->end = lackOffset->end;
			schemeOffset->next = new Segment;
			schemeOffset = schemeOffset->next;
			passedOffset += lackOffset->end - lackOffset->begin;
			STAGOffset->end -= lackOffset->end - lackOffset->begin;
			lackOffset = lackOffset->next;
			schemeOffset->begin = lackOffset->begin;
		}
		schemeOffset->end = STAGOffset->end + lackOffset->begin;
		STAGOffset = STAGOffset->next;
		if (STAGOffset != nullptr)
		{
			schemeOffset->next = new Segment;
			schemeOffset = schemeOffset->next;
		}
		else
			break;
	}
	schemeOffset = storeSchemeOffset;
}

void ContentRSU::_prepareSchemeSwitch()
{
	int prevSlot = itRSL->slot;
	if (++itRSL != rsuSchemeList.end()) // prepare to schedule self message schemeSwitchEvt
	{
		ASSERT( schemeSwitchEvt->isScheduled() == false );
		SimTime dispatchedSlotInterval(slotSpan*(itRSL->slot - prevSlot), SIMTIME_MS);
		schemeSwitchInterval = dispatchedSlotInterval - (simTime() - prevSlotStartTime);
		EV << "prepare to switch to the scheme in slot " << itRSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
		scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
	}
	else
		rsuSchemeList.clear();
}

void ContentRSU::_sendPrefetchRequest(const LAddress::L3Type downloader, const int startOffset, const int endOffset)
{
	WiredMessage *wiredMsg = new WiredMessage("prefetch", WiredMsgCC::START_TRANSMISSION);
	wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
	wiredMsg->setDownloader(downloader);
	wiredMsg->setContentSize(downloaders[downloader]->totalContentSize);
	wiredMsg->setStartOffset(startOffset);
	wiredMsg->setEndOffset(endOffset);
	EV << "prefetching " << endOffset-startOffset << " bytes from content server for it.\n";
	wiredMsg->addBitLength(wiredHeaderLength + 160); // 5*sizeof(int) * 8
	prefetchMsgQueue.insert(wiredMsg);
}

void ContentRSU::_broadcastTransmissionScheme()
{
	if (!schemeItemList.empty()) // it is necessary to distribute transmission scheme items to vehicles
	{
		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *distributionMsg = dynamic_cast<ContentMessage*>(wsm);

		distributionMsg->setControlCode(ContentMsgCC::SCHEME_DISTRIBUTION);
		distributionMsg->setDownloaders(activeDownloaders);
		distributionMsg->setScheme(schemeItemList);

		sendWSM(distributionMsg);

		// clear scheme item list
		for (SchemeItems::iterator itSI = schemeItemList.begin(); itSI != schemeItemList.end(); ++itSI)
			itSI->second.clear();
		schemeItemList.clear();
	}

	if (!rsuSchemeList.empty()) // handle with self transmission scheme
	{
		itRSL = rsuSchemeList.begin();
		ASSERT( schemeSwitchEvt->isScheduled() == false );
		schemeSwitchInterval = SimTime(slotSpan*(itRSL->slot-1), SIMTIME_MS);
		EV << "prepare to switch to the scheme in slot " << itRSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
		scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
	}

	activeDownloaders.clear();
}

bool ContentRSU::selectCarrier(LAddress::L3Type coDownloader)
{
	CoDownloaderInfo *coDownloaderInfo = coDownloaders[coDownloader];

	int minDistR2C = -1;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		if (coDownloaderInfo->speed.x * itV->second->speed.x > 0) // same direction with co-downloader
			continue;
		int distR2C = RoutingUtils::_length(curPosition, itV->second->pos);
		if (distR2C > 50) // it can carry too little data or it can carry too much data
			continue;
		// calculate total data amount it can carry before driving out
		int carryAmount = 0;
		int nextDistR2C = 0;
		Coord currentPos = itV->second->pos, nextPos = currentPos + itV->second->speed;
		while ((nextDistR2C = RoutingUtils::_length(curPosition, nextPos)) < 200)
		{
			int minIdx = distR2C/5, maxIdx = nextDistR2C/5, oneSecondAmount = 0;
			if (minIdx > maxIdx)
				std::swap(minIdx, maxIdx);
			for (int i = minIdx; i <= maxIdx; ++i)
				oneSecondAmount += 128*ContentUtils::rateTable[i];
			oneSecondAmount /= maxIdx - minIdx + 1;
			carryAmount += oneSecondAmount;
			currentPos = nextPos;
			nextPos += itV->second->speed;
			distR2C = nextDistR2C;
		}
		// calculate the time from now on to the time it encounters the co-downloader
		double distD2C = RoutingUtils::_length(coDownloaderInfo->pos, itV->second->pos);
		double relativeSpeed = RoutingUtils::_length(coDownloaderInfo->speed, itV->second->speed);
		double encounterTime = (distD2C - BaseConnectionManager::maxInterferenceDistance) / relativeSpeed;

		EV << "vehicle [" << itV->first << "] can carry " << carryAmount << " bytes, it will encounter the co-downloader after " << encounterTime << "s.\n";

		if (minDistR2C < distR2C)
		{
			minDistR2C = distR2C;
			coDownloaderInfo->carrier = itV->first;
			coDownloaderInfo->encounterSeconds = encounterTime;
			coDownloaderInfo->transmissionAmount = carryAmount;
		}
	}
	if (minDistR2C != -1)
	{
		EV << "find a carrier " << coDownloaderInfo->carrier << ", send carry request to it.\n";

		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *selectMsg = dynamic_cast<ContentMessage*>(wsm);

		selectMsg->setControlCode(ContentMsgCC::CARRIER_SELECTION);
		selectMsg->setReceiver(coDownloaders[coDownloader]->carrier);
		selectMsg->setDownloader(coDownloader);
		selectMsg->setContentSize(downloaders[coDownloader]->totalContentSize);

		sendWSM(selectMsg);
	}
	return minDistR2C != -1;
}

void ContentRSU::_sendCooperativeNotification(const LAddress::L3Type downloader, ContentMessage *reportMsg)
{
	if ((itDL = downloaders.find(downloader)) == downloaders.end())
		return;
	DownloaderInfo *downloaderInfo = itDL->second;
	if (downloaderInfo->sentCoNotification)
	{
		EV << "had sent cooperative notification to neighbor RSU, do nothing.\n";
		return;
	}

	EV << "getting current downloading status and neighbor information of the downloader, notify neighbor RSU.\n";
	WiredMessage *coNotifyMsg = new WiredMessage("co-notification", WiredMsgCC::COOPERATIVE_NOTIFICATION);
	coNotifyMsg->setControlCode(WiredMsgCC::COOPERATIVE_NOTIFICATION);
	coNotifyMsg->setDownloader(downloader);
	coNotifyMsg->setContentSize(downloaderInfo->totalContentSize);
	coNotifyMsg->setCurOffset(reportMsg->getConsumedOffset());
	coNotifyMsg->setStartOffset(reportMsg->getReceivedOffset());
	coNotifyMsg->setEndOffset(downloaderInfo->cacheEndOffset);
	coNotifyMsg->setBytesNum(reportMsg->getConsumingRate()); // reuse function name
	coNotifyMsg->setPosition(reportMsg->getPosition());
	coNotifyMsg->setSpeed(reportMsg->getSpeed());
	coNotifyMsg->setNeighbors(reportMsg->getNeighbors());
	coNotifyMsg->addBitLength(wiredHeaderLength);
	sendDelayed(coNotifyMsg, SimTime::ZERO, reportMsg->getSpeed().x < 0 ? westOut : eastOut);

	downloaderInfo->sentCoNotification = true;
}

void ContentRSU::__eraseDownloader(const LAddress::L3Type downloader)
{
	if ((itDL = downloaders.find(downloader)) != downloaders.end())
	{
		delete itDL->second;
		downloaders.erase(itDL);
	}
}

