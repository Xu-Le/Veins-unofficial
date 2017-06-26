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

		noticeRelayEntering = false;

		distributeVLinkBytesOnce = 20 * (headerLength + dataLengthBits) / 8; // distributeVLinkPacketsOnce == 20
		distributeRLinkBytesOnce = 1000 * 1500; // MTU is 1500 bytes, distributeRLinkPacketsOnce == 1000
		distributeVApplBytesOnce = 20 * dataLengthBits / 8;
		distributeRApplBytesOnce = 1000 * 1472; // UDP/IP header length is 28 bytes
		distributeVPeriod = SimTime::ZERO;
		distributeRPeriod = SimTime(8 * distributeRLinkBytesOnce * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6

		distributeVEvt = new cMessage("distribute v evt", ContentRSUMsgKinds::DISTRIBUTE_V_EVT);
		distributeREvt = new cMessage("distribute r evt", ContentRSUMsgKinds::DISTRIBUTE_R_EVT);
		distributeCEvt = new cMessage("distribute c evt", ContentRSUMsgKinds::DISTRIBUTE_C_EVT);
		distributeVEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at vehicle
		distributeREvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at another RSU
		distributeCEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at carrier
		schemeSwitchEvt = new cMessage("scheme switch evt", ContentRSUMsgKinds::SCHEME_SWITCH_EVT);
		segmentAdvanceEvt = new cMessage("segment advance evt", ContentRSUMsgKinds::SEGMENT_ADVANCE_EVT);
	}
}

void ContentRSU::finish()
{
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		delete itDL->second;
	}
	downloaders.clear();

	cancelAndDelete(distributeVEvt);
	cancelAndDelete(distributeREvt);
	cancelAndDelete(distributeCEvt);
	cancelAndDelete(schemeSwitchEvt);
	cancelAndDelete(segmentAdvanceEvt);

	BaseRSU::finish();
}

void ContentRSU::handleSelfMsg(cMessage *msg)
{
	bool isSegmentAdvanceEvt = true;
	switch (msg->getKind())
	{
	case ContentRSUMsgKinds::SCHEME_SWITCH_EVT:
	{
		EV << "switched to the scheme in slot " << itRSL->slot << ", distributed offset starts at " << itRSL->offset->begin << " bytes.\n";
		downloaders[itRSL->downloader]->distributedOffset = itRSL->offset->begin;
		prevSlotStartTime = simTime();
		isSegmentAdvanceEvt = false;
	}
	case ContentRSUMsgKinds::SEGMENT_ADVANCE_EVT:
	{
		if (isSegmentAdvanceEvt)
		{
			EV << "switched to advance the next segment, distributed offset starts at " << itRSL->offset->begin << " bytes.\n";
			downloaders[itRSL->downloader]->distributedOffset = itRSL->offset->begin;
		}
	}
	case ContentRSUMsgKinds::DISTRIBUTE_V_EVT:
	{
		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(itRSL->receiver);
		dataMsg->setDownloader(itRSL->downloader);
		DownloaderInfo *downloaderInfo = downloaders[itRSL->downloader];
		// estimate transmission rate between self and receiver only taking relative distance into consideration
		int distance = RoutingUtils::_length(curPosition, vehicles[itRSL->receiver]->pos);
		ASSERT( distance < 250 );
		int estimatedRate = ContentUtils::rateTable[distance/5];
		EV << "estimated rate between myself and receiver [" << itRSL->receiver << "] is " << 128*estimatedRate << " bytes per second.\n";
		if (distance >= 200)
		{
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

			if (itRSL->receiver == itRSL->downloader)
			{
				EV << "the communication link between downloader and RSU will break soon, notify downloader to fetch its downloading status.\n";
				notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_DIRECT);
			}
			else
			{
				EV << "the communication link between relay and RSU will break soon, notify downloader to fetch its downloading status.\n";
				notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_RR);
			}
			notifyMsg->setReceiver(itRSL->receiver);
			notifyMsg->setDownloader(itRSL->downloader);

			sendWSM(notifyMsg);
		}
		if (itRSL->offset->end - itRSL->offset->begin > distributeVApplBytesOnce)
		{
			dataMsg->setIsLast(false);
			dataMsg->setBytesNum(distributeVApplBytesOnce);
			dataMsg->addBitLength(headerLength); // 8*distributeVLinkBytesOnce
			distributeVPeriod = SimTime(distributeVLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
			distributeVPeriod /= 2; // TODO: remove it in the future
			EV << "distribute vehicle period is " << distributeVPeriod.dbl() << "s.\n";
			if (simTime() + distributeVPeriod - prevSlotStartTime < SimTime(slotSpan, SIMTIME_MS))
			{
				scheduleAt(simTime() + distributeVPeriod, distributeVEvt);
				itRSL->offset->begin += distributeVApplBytesOnce;
				downloaderInfo->distributedOffset += distributeVApplBytesOnce;
			}
			else
			{
				EV << "there is not enough time to transmit data in current slot, cancel the transmission.\n";

				_prepareSchemeSwitch();

				delete dataMsg;
				dataMsg = nullptr;
				return;
			}
		}
		else
		{
			dataMsg->setIsLast(itRSL->offset->next == NULL);
			int lastPktAmount = itRSL->offset->end - itRSL->offset->begin; // alias
			int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, headerLength/8, dataLengthBits/8);
			dataMsg->setBytesNum(lastPktAmount);
			dataMsg->addBitLength(headerLength); // 8*totalLinkBytes
			distributeVPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
			distributeVPeriod /= 2; // TODO: remove it in the future
			EV << "distribute vehicle period is " << distributeVPeriod.dbl() << "s.\n";
			if (simTime() + distributeVPeriod - prevSlotStartTime < SimTime(slotSpan, SIMTIME_MS))
				downloaderInfo->distributedOffset += lastPktAmount;
			else
			{
				EV << "there is not enough time to transmit data in current slot, cancel the transmission.\n";

				_prepareSchemeSwitch();

				delete dataMsg;
				dataMsg = nullptr;
				return;
			}

			if (itRSL->offset->next == NULL)
				_prepareSchemeSwitch();
			else // this rarely happens
			{
				itRSL->offset = itRSL->offset->next; // prepare to transmit the next segment
				EV << "prepare to advance to the next segment.\n";
				scheduleAt(simTime() + SimTime(1, SIMTIME_US), segmentAdvanceEvt);
			}
		}
		EV << "downloader [" << itRSL->downloader << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		sendDelayedDown(dataMsg, distributeVPeriod);
		break;
	}
	case ContentRSUMsgKinds::DISTRIBUTE_R_EVT:
	{
		for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		{
			DownloaderInfo *downloaderInfo = itDL->second; // alias
			if (downloaderInfo->distributedAt == simTime() - distributeRPeriod) // self message aims to this downloader
			{
				WiredMessage *dataMsg = new WiredMessage("data");
				dataMsg->setDownloader(itDL->first);
				if (downloaderInfo->cacheEndOffset - downloaderInfo->distributedROffset > distributeRApplBytesOnce)
				{
					dataMsg->setControlCode(WiredMsgCC::NORMAL_DATA_PACKET);
					dataMsg->setBytesNum(distributeRApplBytesOnce);
					dataMsg->addBitLength(8*distributeRLinkBytesOnce);
					scheduleAt(simTime() + distributeRPeriod, distributeREvt);
					downloaderInfo->distributedROffset += distributeRApplBytesOnce;
					downloaderInfo->distributedAt = simTime();
				}
				else
				{
					int lastPktAmount = downloaderInfo->cacheEndOffset - downloaderInfo->distributedROffset; // alias
					dataMsg->setControlCode(WiredMsgCC::LAST_DATA_PACKET);
					int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, 28, 1472);
					dataMsg->setBytesNum(lastPktAmount);
					dataMsg->addBitLength(8*totalLinkBytes);
					downloaderInfo->distributedROffset = downloaderInfo->cacheEndOffset;
				}
				EV << "downloader [" << itDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedROffset << " bytes.\n";
				dataMsg->setCurOffset(downloaderInfo->distributedROffset);
				int *outGate = static_cast<int*>(distributeREvt->getContextPointer());
				sendDelayed(dataMsg, SimTime::ZERO, *outGate);
				break;
			}
		}
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
					dataMsg->setIsLast(false);
					dataMsg->setBytesNum(distributeVApplBytesOnce);
					dataMsg->addBitLength(headerLength); // 8*distributeVLinkBytesOnce
					distributeCPeriod = SimTime(distributeVLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
					scheduleAt(simTime() + distributeCPeriod, distributeCEvt);
					downloaderInfo->distributedOffset += distributeVApplBytesOnce;
					coDownloaderInfo->transmitAt = simTime() + distributeCPeriod;
				}
				else
				{
					dataMsg->setIsLast(true);
					int lastPktAmount = downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset; // alias
					int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, headerLength/8, dataLengthBits/8);
					dataMsg->setBytesNum(lastPktAmount);
					dataMsg->addBitLength(headerLength); // 8*totalLinkBytes
					distributeCPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
					downloaderInfo->distributedOffset += lastPktAmount;
				}
				EV << "distribute carrier period is " << distributeCPeriod.dbl() << "s.\n";
				EV << "downloader [" << itCDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayedDown(dataMsg, distributeCPeriod);
				break;
			}
		}
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
	DownloaderInfo *downloaderInfo = downloaders[wiredMsg->getDownloader()];
	if (downloaderInfo->cacheStartOffset == -1)
		downloaderInfo->cacheStartOffset = wiredMsg->getCurOffset() - wiredMsg->getBytesNum();
	downloaderInfo->cacheEndOffset = wiredMsg->getCurOffset();
	EV << "downloader [" << downloader << "]'s cache end offset updates to " << downloaderInfo->cacheEndOffset << " bytes.\n";

	if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET) // it is the last data packet, indicating prefetch progress has finished
	{
		if (coDownloaders.find(downloader) == coDownloaders.end())
		{
			EV << "its content prefetch progress has finished, now begin to distribute transmission scheme to relays.\n";

			if (!schemeItemList.empty()) // it is necessary to distribute transmission scheme items to vehicles
			{
				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *distributionMsg = dynamic_cast<ContentMessage*>(wsm);

				distributionMsg->setControlCode(ContentMsgCC::SCHEME_DISTRIBUTION);
				distributionMsg->setDownloader(downloader);
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
				schemeSwitchInterval = SimTime(slotSpan*(itRSL->slot-1), SIMTIME_MS);
				EV << "prepare to switch to the scheme in slot " << itRSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
				scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
			}
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
		downloaderInfo = new DownloaderInfo(wiredMsg->getContentSize(), 0); // consuming rate is set when relay discovery event happens
		downloaderInfo->totalContentSize = wiredMsg->getContentSize();
		downloaderInfo->distributedOffset = wiredMsg->getStartOffset();
		downloaderInfo->acknowledgedOffset = wiredMsg->getStartOffset();
		downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));

		WiredMessage *coAckMsg = new WiredMessage("co-ack");
		coAckMsg->setControlCode(WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT);
		coAckMsg->setDownloader(downloader);
		coAckMsg->setStartOffset(wiredMsg->getStartOffset());
		coAckMsg->setEndOffset(wiredMsg->getEndOffset());
		coAckMsg->addBitLength(wiredHeaderLength);
		sendDelayed(coAckMsg, SimTime::ZERO, outGate);
	}
	else if (wiredMsg->getControlCode() == WiredMsgCC::COOPERATIVE_ACKNOWLEDGEMENT)
	{
		downloaderInfo = downloaders[downloader];

		downloaderInfo->distributedROffset = wiredMsg->getStartOffset();
		WiredMessage *dataMsg = new WiredMessage("co-data");
		dataMsg->setDownloader(downloader);
		if (wiredMsg->getEndOffset() - wiredMsg->getStartOffset() > distributeRApplBytesOnce)
		{
			dataMsg->setControlCode(WiredMsgCC::NORMAL_DATA_PACKET);
			dataMsg->setBytesNum(distributeRApplBytesOnce);
			dataMsg->addBitLength(8*distributeRLinkBytesOnce);
			distributeREvt->setContextPointer(&westOut);
			scheduleAt(simTime() + distributeRPeriod, distributeREvt);
			downloaderInfo->distributedROffset += distributeRApplBytesOnce;
			downloaderInfo->distributedAt = simTime();
		}
		else
		{
			int lastPktAmount = wiredMsg->getEndOffset() - wiredMsg->getStartOffset(); // alias
			dataMsg->setControlCode(WiredMsgCC::LAST_DATA_PACKET);
			int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, 28, 1472);
			dataMsg->setBytesNum(lastPktAmount);
			dataMsg->addBitLength(8*totalLinkBytes);
			downloaderInfo->distributedROffset = wiredMsg->getEndOffset();
		}
		EV << "downloader [" << downloader << "]'s distributed offset updates to " << downloaderInfo->distributedROffset << " bytes.\n";
		dataMsg->setCurOffset(downloaderInfo->distributedROffset);
		sendDelayed(dataMsg, SimTime::ZERO, outGate);
	}
	else // receiving data packet
	{
		downloaderInfo = downloaders[downloader];

		if (downloaderInfo->cacheStartOffset == -1)
			downloaderInfo->cacheStartOffset = wiredMsg->getCurOffset() - wiredMsg->getBytesNum();
		downloaderInfo->cacheEndOffset = wiredMsg->getCurOffset();
		EV << "downloader [" << downloader << "]'s cache end offset updates to " << downloaderInfo->cacheEndOffset << " bytes.\n";

		if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET)
		{
			EV << "its co-data receiving progress has finished, now try to choose a carrier.\n";

			bool hasCarrier = selectCarrier(downloader);
			if (hasCarrier)
			{
				EV << "find a carrier " << coDownloaders[downloader]->carrier << ", send carry request to it.\n";

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *selectMsg = dynamic_cast<ContentMessage*>(wsm);

				selectMsg->setControlCode(ContentMsgCC::CARRIER_SELECTION);
				selectMsg->setReceiver(coDownloaders[downloader]->carrier);
				selectMsg->setDownloader(wiredMsg->getDownloader());
				selectMsg->setContentSize(downloaderInfo->totalContentSize);

				sendWSM(selectMsg);
			}
		}
	}
}

void ContentRSU::decorateWSM(WaveShortMessage *wsm)
{
	BaseRSU::decorateWSM(wsm);
}

void ContentRSU::onBeacon(BeaconMessage *beaconMsg)
{
	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias
	// bool isOld = vehicles.find(sender) != vehicles.end();

	BaseRSU::onBeacon(beaconMsg);

	if (noticeRelayEntering)
	{
		for (itCDL = coDownloaders.begin(); itCDL != coDownloaders.end(); ++itCDL)
		{
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
			}
		}
	}
}

void ContentRSU::onRouting(RoutingMessage *routingMsg)
{
	EV << "ContentRSUs don't react to content messages since they don't help routings.\n";
}

void ContentRSU::onContent(ContentMessage *contentMsg)
{
	EV << "rsu[" << myAddr - RSU_ADDRESS_OFFSET << "]: " << "onContent!\n";

	// LAddress::L3Type sender = contentMsg->getSenderAddress(); // alias
	LAddress::L3Type downloader = contentMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	switch (contentMsg->getControlCode())
	{
	case ContentMsgCC::CONTENT_REQUEST: // it is a request message from a downloader
	{
		if ( downloaders.find(downloader) != downloaders.end() ) // update old record
		{
			EV << "    downloader [" << downloader << "] is an old downloader, update its info.\n";
			downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
			downloaders[downloader]->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
		}
		else
		{
			// insert new record
			EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
			downloaderInfo = new DownloaderInfo(contentMsg->getContentSize(), contentMsg->getConsumingRate());
			downloaderInfo->lackOffset->begin = 0;
			downloaderInfo->lackOffset->end = contentMsg->getContentSize();
			downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));

			// response to the request vehicle
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);

			responseMsg->setControlCode(ContentMsgCC::CONTENT_RESPONSE);
			responseMsg->setDownloader(downloader);
			sendWSM(responseMsg);
			++ContentStatisticCollector::globalParticipatingRSUNum;

			// use STAG approach to obtain the optimal cooperative transmission scheme
			SimTime calculatingTime = obtainCooperativeScheme();
			ackMsgNum = 0;

			// send prefetch data request to content server
			WiredMessage *wiredMsg = new WiredMessage("prefetch");
			wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
			wiredMsg->setDownloader(downloader);
			wiredMsg->setContentSize(contentMsg->getContentSize());
			wiredMsg->setStartOffset(0);
			wiredMsg->setEndOffset(downloaderInfo->prefetchDataAmount);
			EV << "prefetching " << downloaderInfo->prefetchDataAmount << " bytes from content server for it.\n";
			wiredMsg->addBitLength(wiredHeaderLength + 160); // 5*sizeof(int) * 8
			sendDelayed(wiredMsg, calculatingTime, wiredOut);
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
		if ( downloaders.find(downloader) != downloaders.end() )
		{
			EV << "downloader [" << downloader << "]'s acknowledged offset updates to " << contentMsg->getReceivedOffset() << std::endl;
			downloaderInfo = downloaders[downloader];
			downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
			downloaderInfo->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
			if (++ackMsgNum == activeSlotNum)
			{
				downloaderInfo->lackOffset->assign(&contentMsg->getLackOffset());

				// use STAG approach to obtain the optimal cooperative transmission scheme again
				SimTime calculatingTime = obtainCooperativeScheme(); // activeSlotNum reset inside
				ackMsgNum = 0;

				// send prefetch data request to content server
				WiredMessage *wiredMsg = new WiredMessage("prefetch");
				wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
				wiredMsg->setDownloader(downloader);
				wiredMsg->setContentSize(downloaderInfo->totalContentSize);
				wiredMsg->setStartOffset(downloaderInfo->cacheEndOffset);
				wiredMsg->setEndOffset(downloaderInfo->cacheEndOffset + downloaderInfo->prefetchDataAmount);
				EV << "prefetching " << downloaderInfo->prefetchDataAmount << " bytes from content server for it.\n";
				wiredMsg->addBitLength(wiredHeaderLength + 160); // 5*sizeof(int) * 8
				sendDelayed(wiredMsg, calculatingTime, wiredOut);
			}
		}
		break;
	}
	case ContentMsgCC::CARRIER_SELECTION: // it is a carrier chosen notification message from RSU
	{
		EV << "receiving carry response from carrier, now check whether cached data is adequate.\n";
		downloaderInfo = downloaders[downloader];
		CoDownloaderInfo *coDownloaderInfo = coDownloaders[downloader];
		// check whether cached data is adequate for transmitting to carrier
		if (coDownloaderInfo->transmissionAmount > downloaderInfo->cacheEndOffset - downloaderInfo->cacheStartOffset)
		{
			EV << "cached data is not adequate, prefetch lack portion from content server.\n";

			if (coDownloaderInfo->transmissionAmount > downloaderInfo->totalContentSize - downloaderInfo->cacheStartOffset)
				coDownloaderInfo->transmissionAmount = downloaderInfo->totalContentSize - downloaderInfo->cacheStartOffset;
			WiredMessage *wiredMsg = new WiredMessage("wired");
			wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
			wiredMsg->setDownloader(downloader);
			wiredMsg->setStartOffset(downloaderInfo->cacheEndOffset);
			wiredMsg->setEndOffset(downloaderInfo->cacheEndOffset + coDownloaderInfo->transmissionAmount);
			wiredMsg->addBitLength(wiredHeaderLength);
			sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
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
		downloaders.erase(downloader);
		ackMsgNum = 0;
		activeSlotNum = 0;
	    break;
	}
	case ContentMsgCC::LINK_BREAK_DIRECT: // it is a link break notification - the communication link between downloader and RSU will break soon
	{
		EV << "getting current downloading status and neighbor information of the downloader, notify neighbor RSU.\n";

		_sendCooperativeNotification(downloader, contentMsg);
		break;
	}
	case ContentMsgCC::LINK_BREAK_DR: // it is a link break notification - the communication link between downloader and relay will break soon
	{
		if (contentMsg->getReceivedOffset() > 0)
		{
			EV << "getting current downloading status and neighbor information of the downloader, notify neighbor RSU.\n";

			_sendCooperativeNotification(downloader, contentMsg);
		}
		else
			EV << "it is a link break notification sent by relay, downloader has not response it, ignore it.\n";
		break;
	}
	case ContentMsgCC::LINK_BREAK_RR: // it is a link break notification - the communication link between relay and RSU will break soon
	{
		if (contentMsg->getReceivedOffset() > 0)
		{
			EV << "getting current downloading status and neighbor information of the downloader, notify neighbor RSU.\n";

			_sendCooperativeNotification(downloader, contentMsg);
		}
		else
			EV << "it is a link break notification rebroadcast by relay, downloader has not response it, ignore it.\n";
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
			ContentUtils::vectorRemove(coDownloaders[downloader]->neighbors, contentMsg->getSenderAddress());
		else // received response is yes, now start to execute STAG algorithm
		{
			noticeRelayEntering = false;

			coDownloaders[downloader]->neighbors.clear();
			downloaderInfo = downloaders[downloader];
			downloaderInfo->distributedOffset = contentMsg->getReceivedOffset();
			downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
			downloaderInfo->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
			downloaderInfo->consumingRate = contentMsg->getConsumingRate();

			// insert downloader's mobility information into container vehicles
			EV << "    vehicle [" << downloader << "] is a new vehicle, insert its info.\n";
			VehicleInfo *vehicleInfo = new VehicleInfo(contentMsg->getPosition(), contentMsg->getSpeed(), simTime());
			vehicles.insert(std::pair<LAddress::L3Type, VehicleInfo*>(downloader, vehicleInfo));

			// use STAG approach to obtain the optimal cooperative transmission scheme
			SimTime calculatingTime = obtainCooperativeScheme();
			ackMsgNum = 0;

			// send prefetch data request to content server
			WiredMessage *wiredMsg = new WiredMessage("prefetch");
			wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
			wiredMsg->setDownloader(downloader);
			wiredMsg->setContentSize(downloaderInfo->totalContentSize);
			wiredMsg->setStartOffset(downloaderInfo->distributedOffset);
			wiredMsg->setEndOffset(downloaderInfo->prefetchDataAmount);
			EV << "prefetching " << downloaderInfo->prefetchDataAmount << " bytes from content server for it.\n";
			wiredMsg->addBitLength(wiredHeaderLength + 160); // 5*sizeof(int) * 8
			sendDelayed(wiredMsg, calculatingTime, wiredOut);
		}
		break;
	}
	case ContentMsgCC::STATUS_QUERY:
	{
		EV << "it is a downloading status query/report message, ignore it.\n";
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
	activeSlotNum = 0;
	for (itRSL = rsuSchemeList.begin(); itRSL != rsuSchemeList.end(); ++itRSL)
		if (itRSL->receiver == itRSL->downloader) // direct downloading
			++activeSlotNum;
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
			activeSlotNum += static_cast<int>(graph.fluxSchemeList[nodeId].size());
		}
	}
	EV << "active transmission slot number is " << activeSlotNum << std::endl;

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
	downloaderNum = static_cast<int>(downloaders.size());

	std::vector<int> nodeIdMap(vehicles.size()+1, 0); // maintain node ID map from node IDs used by STAG to real vehicle IDs
	std::map<int, int> nodeIdRevMap; // maintain node ID map from real vehicle IDs to node IDs used by STAG
	int nodeId = 1, j = 0;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		nodeIdMap[nodeId] = itV->first;
		nodeIdRevMap.insert(std::pair<int, int>(itV->first, nodeId++));
	}

	std::vector<int> distPadding(nodeNum, 0.0);
	std::vector<Coord> slotPadding(slotNum+1, Coord::ZERO);
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
		nodeSpeed[nodeId][0] = vehicles[nodeIdMap[nodeId]]->speed;
		nodePos[nodeId][0] = vehicles[nodeIdMap[nodeId]]->pos;
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
				int nextDist = RoutingUtils::_length(nodePos[srcNode][j], nodePos[dstNode][j]);
				nodeDist[srcNode][dstNode] = nextDist;
				if (curDist < 250 || nextDist < 250)
				{
					int minIdx = curDist/5, maxIdx = nextDist/5;
					if (minIdx > maxIdx)
						std::swap(minIdx, maxIdx);
					int maxIdx_ = std::min(49, maxIdx); // avoid reading memory outside of the bounds of ContentUtils::rateTable[]
					for (int i = minIdx; i <= maxIdx_; ++i)
						itLT->bandwidth[j] += 128*ContentUtils::rateTable[i];
					itLT->bandwidth[j] /= maxIdx - minIdx + 1;
				}
				else
					itLT->bandwidth[j] = 0;
				++itLT;
			}
		}
	}

	int downloaderIdx = 0;
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL, ++downloaderIdx)
	{
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
	}

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
		fout << std::endl;
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
	for (int j = 1; j < slotNum; ++j)
		for (int nodeId = 1; nodeId < nodeNum; ++nodeId) // update vehicles position, assuming they have uniform motion
			vehiclePos[nodeId][j] = vehiclePos[nodeId][j-1] + vehicleSpeed[nodeId][0];
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
		SimTime dispatchedSlotInterval(slotSpan*(itRSL->slot - prevSlot), SIMTIME_MS);
		schemeSwitchInterval = dispatchedSlotInterval - (simTime() - prevSlotStartTime);
		EV << "prepare to switch to the scheme in slot " << itRSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
		scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
	}
	else
		rsuSchemeList.clear();
}

bool ContentRSU::selectCarrier(LAddress::L3Type coDownloader)
{
	CoDownloaderInfo *coDownloaderInfo = coDownloaders[coDownloader];

	double minEncounterTime = 3601.0;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		if (coDownloaderInfo->speed.x * itV->second->speed.x > 0)
			continue;
		int distR2C = RoutingUtils::_length(curPosition, itV->second->pos);
		if (distR2C > 150 && (curPosition.x - itV->second->pos.x)*itV->second->speed.x < 0)
			continue;
		double distD2C = RoutingUtils::_length(coDownloaderInfo->pos, itV->second->pos);
		double relativeSpeed = RoutingUtils::_length(coDownloaderInfo->speed, itV->second->speed);
		double encounterTime = (distD2C - 2*BaseConnectionManager::maxInterferenceDistance) / relativeSpeed;
		if (minEncounterTime > encounterTime)
		{
			minEncounterTime = encounterTime;
			coDownloaderInfo->carrier = itV->first;
		}
	}
	return minEncounterTime < 3600.0;
}

void ContentRSU::_sendCooperativeNotification(int downloader, ContentMessage *reportMsg)
{
	DownloaderInfo *downloaderInfo = downloaders[downloader];
	downloaderInfo->acknowledgedOffset = reportMsg->getReceivedOffset();

	WiredMessage *coNotifyMsg = new WiredMessage("co-notification");
	coNotifyMsg->setControlCode(WiredMsgCC::COOPERATIVE_NOTIFICATION);
	coNotifyMsg->setDownloader(downloader);
	coNotifyMsg->setContentSize(downloaderInfo->totalContentSize);
	coNotifyMsg->setStartOffset(downloaderInfo->acknowledgedOffset);
	coNotifyMsg->setEndOffset(downloaderInfo->cacheEndOffset);
	coNotifyMsg->setPosition(reportMsg->getPosition());
	coNotifyMsg->setSpeed(reportMsg->getSpeed());
	coNotifyMsg->setNeighbors(reportMsg->getNeighbors());
	coNotifyMsg->addBitLength(wiredHeaderLength);
	sendDelayed(coNotifyMsg, SimTime::ZERO, reportMsg->getPosition().x < 0 ? westOut : eastOut);
}

