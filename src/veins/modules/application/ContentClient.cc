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

#include "veins/modules/application/ContentClient.h"
#define CLIENT_DEBUG
#ifdef CLIENT_DEBUG
#include <fstream>
#endif

Define_Module(ContentClient);

void ContentClient::initialize(int stage)
{
	BaseWaveApplLayer::initialize(stage);

	if (stage == 0)
	{
		carrierDownloading = false;
		encounteredDownloader = false;
		carriedDownloader = -1;
		brokenDownloader = -1;
		rebroadcastDownloader = -1;

		slotSpan = par("slotSpan").longValue();
		prevSlotStartTime = SimTime::ZERO;
		relayLinkBytesOnce = 10 * (headerLength + dataLengthBits) / 8; // relayLinkPacketsOnce == 10
		relayApplBytesOnce = 10 * dataLengthBits / 8;
		relayPeriod = SimTime::ZERO;

		requestTimeoutDuration = SimTime(100, SIMTIME_MS);
		requestTimeoutPeriod = SimTime(1, SIMTIME_S);

		relayEvt = new cMessage("relay evt", ContentClientMsgKinds::RELAY_EVT);
		forwardEvt = new cMessage("forward evt", ContentClientMsgKinds::FORWARD_EVT);
		relayEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at downloader
		forwardEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at downloader
		schemeSwitchEvt = new cMessage("scheme switch evt", ContentClientMsgKinds::SCHEME_SWITCH_EVT);
		segmentAdvanceEvt = new cMessage("segment advance evt", ContentClientMsgKinds::SEGMENT_ADVANCE_EVT);
		reportStatusEvt = new cMessage("report status evt", ContentClientMsgKinds::REPORT_STATUS_EVT);
		requestTimeoutEvt = new cMessage("request timeout evt", ContentClientMsgKinds::REQUEST_TIMEOUT_EVT);
		linkBrokenEvt = new cMessage("link broken evt", ContentClientMsgKinds::LINK_BROKEN_EVT);
		rebroadcastBeaconEvt = new cMessage("rebroadcast beacon evt", ContentClientMsgKinds::REBROADCAST_BEACON_EVT);
	}
}

void ContentClient::finish()
{
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		delete itDL->second;
	downloaders.clear();

	cancelAndDelete(relayEvt);
	cancelAndDelete(forwardEvt);
	cancelAndDelete(schemeSwitchEvt);
	cancelAndDelete(segmentAdvanceEvt);
	cancelAndDelete(reportStatusEvt);
	cancelAndDelete(requestTimeoutEvt);
	cancelAndDelete(linkBrokenEvt);
	cancelAndDelete(rebroadcastBeaconEvt);

	BaseWaveApplLayer::finish();
}

void ContentClient::handleSelfMsg(cMessage* msg)
{
	bool isSegmentAdvanceEvt = true;
	switch (msg->getKind())
	{
	case ContentClientMsgKinds::SCHEME_SWITCH_EVT:
	{
		if (downloaders.find(itSL->downloader) == downloaders.end())
		{
			EV << "communication link with this downloader has broken, skip current planned scheme.\n";
			schemeList.clear();
			return;
		}

		EV << "switched to the scheme in slot " << itSL->slot << ", check planned transmission offsets.\n";
		_correctPlannedOffset();

		EV << "after checking, distributed offset starts at " << itSL->offset->begin << " bytes.\n";
		downloaders[itSL->downloader]->distributedOffset = itSL->offset->begin;
		prevSlotStartTime = simTime();
		isSegmentAdvanceEvt = false;
	}
	case ContentClientMsgKinds::SEGMENT_ADVANCE_EVT:
	{
		if (isSegmentAdvanceEvt)
		{
			EV << "switched to advance the next segment, distributed offset starts at " << itSL->offset->begin << " bytes.\n";
			downloaders[itSL->downloader]->distributedOffset = itSL->offset->begin;
		}
	}
	case ContentClientMsgKinds::RELAY_EVT:
	{
		if (itSL->offset->end == itSL->offset->begin)
		{
			EV << "there is no data cached for this planned segment.\n";
			if (itSL->offset->next == NULL)
				_prepareSchemeSwitch();
			else
			{
				itSL->offset = itSL->offset->next; // prepare to transmit the next segment
				EV << "prepare to advance to the next segment.\n";
				scheduleAt(simTime(), segmentAdvanceEvt);
			}
			return;
		}
		if ((itN = neighbors.find(itSL->receiver)) == neighbors.end())
		{
			EV << "receiver [" << itSL->receiver << "] is disconnected.\n";
			_prepareSchemeSwitch();
			return;
		}

		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(itSL->receiver);
		dataMsg->setDownloader(itSL->downloader);
		DownloaderInfo *downloaderInfo = downloaders[itSL->downloader];
		// estimate transmission rate between self and downloader only taking relative distance into consideration
		Coord &receiverPos = itN->second->pos;
		Coord &receiverSpeed = itN->second->speed;
		int distance = RoutingUtils::_length(curPosition, receiverPos);
		if (distance >= 250)
		{
			EV << "distance " << distance << " is larger than 250, give up sending the data and notify link broken event.\n";
			delete dataMsg;

			brokenDownloader = itSL->downloader;
			scheduleAt(simTime(), linkBrokenEvt);

			_prepareSchemeSwitch();
			return;
		}
		int estimatedRate = ContentUtils::rateTable[distance/5];
		EV << "estimated rate between myself and receiver [" << itSL->receiver << "] is " << 128*estimatedRate << " bytes per second.\n";
		if (distance > 200 && !downloaderInfo->notifiedLinkBreak && (curPosition.x - receiverPos.x)*receiverSpeed.x < 0)
		{
			EV << "the communication link between downloader and relay will break soon, notify downloader to report its downloading status to RSU.\n";
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

			notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_DR);
			notifyMsg->setReceiver(itSL->receiver);
			notifyMsg->setDownloader(itSL->downloader);

			sendWSM(notifyMsg);

			downloaderInfo->notifiedLinkBreak = true;
		}
		if (itSL->offset->end - itSL->offset->begin > relayApplBytesOnce)
		{
			dataMsg->setIsLast(false);
			dataMsg->setBytesNum(relayApplBytesOnce);
			// dataMsg->addBitLength(8*relayLinkBytesOnce);
			ContentStatisticCollector::globalRelayFlux += relayApplBytesOnce;
			relayPeriod = SimTime(relayLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
			EV << "relay period is " << relayPeriod.dbl() << "s.\n";
			if (simTime() + relayPeriod - prevSlotStartTime < SimTime(slotSpan, SIMTIME_MS))
			{
				scheduleAt(simTime() + relayPeriod, relayEvt);
				itSL->offset->begin += relayApplBytesOnce;
				downloaderInfo->distributedOffset += relayApplBytesOnce;
			}
			else
			{
				EV << "there is not enough time to transmit data in current slot, send a dummy last packet.\n";

				dataMsg->setIsLast(true);
				dataMsg->setBytesNum(0);
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayedDown(dataMsg, SimTime::ZERO);
				brokenDownloader = itSL->downloader;
				scheduleAt(simTime() + SimTime(20, SIMTIME_MS), linkBrokenEvt);

				_prepareSchemeSwitch();
				return;
			}
		}
		else
		{
			dataMsg->setIsLast(itSL->offset->next == NULL);
			int lastPktAmount = itSL->offset->end - itSL->offset->begin; // alias
			int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, headerLength/8, dataLengthBits/8);
			dataMsg->setBytesNum(lastPktAmount);
			// dataMsg->addBitLength(8*totalLinkBytes);
			ContentStatisticCollector::globalRelayFlux += lastPktAmount;
			relayPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
			EV << "relay period is " << relayPeriod.dbl() << "s.\n";
			if (simTime() + relayPeriod - prevSlotStartTime < SimTime(slotSpan, SIMTIME_MS))
				downloaderInfo->distributedOffset += lastPktAmount;
			else
			{
				EV << "there is not enough time to transmit data in current slot, send a dummy last packet.\n";

				dataMsg->setIsLast(true);
				dataMsg->setBytesNum(0);
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayedDown(dataMsg, SimTime::ZERO);
				brokenDownloader = itSL->downloader;
				scheduleAt(simTime() + SimTime(20, SIMTIME_MS), linkBrokenEvt);

				_prepareSchemeSwitch();
				return;
			}

			if (itSL->offset->next == NULL)
				_prepareSchemeSwitch();
			else // this rarely happens
			{
				itSL->offset = itSL->offset->next; // prepare to transmit the next segment
				EV << "prepare to advance to the next segment.\n";
				scheduleAt(simTime() + relayPeriod, segmentAdvanceEvt);
			}
		}
		EV << "downloader [" << itSL->downloader << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		sendDelayedDown(dataMsg, relayPeriod);
		break;
	}
	case ContentClientMsgKinds::FORWARD_EVT:
	{
		if ((itDL = downloaders.find(carriedDownloader)) == downloaders.end())
			return;

		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(carriedDownloader);
		dataMsg->setDownloader(carriedDownloader);
		DownloaderInfo *downloaderInfo = itDL->second;
		// estimate transmission rate between self and downloader only taking relative distance into consideration
		int distance = RoutingUtils::_length(curPosition, neighbors[carriedDownloader]->pos);
		ASSERT( distance < 250 );
		int estimatedRate = ContentUtils::rateTable[distance/5];
		EV << "estimated rate between myself and receiver [" << carriedDownloader << "] is " << 128*estimatedRate << " bytes per second.\n";
		SimTime distributeCDPeriod;
		if (downloaderInfo->cacheOffset->end - downloaderInfo->distributedOffset > relayApplBytesOnce)
		{
			bool endTransmission = distance >= 200 && (curPosition.x -  neighbors[carriedDownloader]->pos.x)*curSpeed.x > 0;
			dataMsg->setIsLast(endTransmission); // due to their opposite direction moving, it is 200 not 225
			dataMsg->setBytesNum(relayApplBytesOnce);
			// dataMsg->addBitLength(8*relayLinkBytesOnce);
			ContentStatisticCollector::globalCarryFlux += relayApplBytesOnce;
			distributeCDPeriod = SimTime(relayLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
			if (!endTransmission)
				scheduleAt(simTime() + distributeCDPeriod, forwardEvt);
			downloaderInfo->distributedOffset += relayApplBytesOnce;
		}
		else
		{
			dataMsg->setIsLast(true);
			int lastPktAmount = downloaderInfo->cacheOffset->end - downloaderInfo->distributedOffset; // alias
			int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, headerLength/8, dataLengthBits/8);
			dataMsg->setBytesNum(lastPktAmount);
			// dataMsg->addBitLength(8*totalLinkBytes);
			ContentStatisticCollector::globalCarryFlux += lastPktAmount;
			distributeCDPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
			downloaderInfo->distributedOffset += lastPktAmount;
		}
		EV << "distribute carried downloader period is " << distributeCDPeriod.dbl() << "s.\n";
		EV << "downloader [" << carriedDownloader << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		if (dataMsg->getIsLast())
		{
			carriedDownloader = -1;
			encounteredDownloader = false;
			sendDelayedDown(dataMsg, individualOffset);
		}
		else
			sendDelayedDown(dataMsg, distributeCDPeriod);
		break;
	}
	case ContentClientMsgKinds::REPORT_STATUS_EVT:
	{
		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *reportMsg = dynamic_cast<ContentMessage*>(wsm);

		reportMsg->setControlCode(ContentMsgCC::STATUS_REPORT);
		reportMsg->setDownloader(myAddr);
		reportMsg->setContentSize(downloadingStatus.totalContentSize);
		reportMsg->setReceivedOffset(downloadingStatus.availableOffset);
		EV << "available offsets: ";
		downloadingStatus.lackSegment(&reportMsg->getLackOffset());

		sendWSM(reportMsg);
		break;
	}
	case ContentClientMsgKinds::REQUEST_TIMEOUT_EVT:
	{
		sendContentRequest();

		scheduleAt(simTime() + requestTimeoutPeriod, requestTimeoutEvt);
		EV << "content request is timeout, reset request timeout timer, its period is " << requestTimeoutPeriod.dbl() << "s.\n";
		break;
	}
	case ContentClientMsgKinds::LINK_BROKEN_EVT:
	{
		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

		notifyMsg->setControlCode(ContentMsgCC::LINK_BROKEN_DR);
		notifyMsg->setDownloader(brokenDownloader);

		sendWSM(notifyMsg);

		if ((itDL = downloaders.find(brokenDownloader)) != downloaders.end())
		{
			delete itDL->second;
			downloaders.erase(itDL);
		}
		break;
	}
	case ContentClientMsgKinds::REBROADCAST_BEACON_EVT:
	{
		rebroadcastDownloader = -1;
		break;
	}
	default:
		BaseWaveApplLayer::handleSelfMsg(msg);
	}
}

void ContentClient::handleCellularMsg(CellularMessage *cellularMsg)
{
	throw cRuntimeError("ContentClient::handleCellularMsg() should not be called.");
}

void ContentClient::onBeacon(BeaconMessage *beaconMsg)
{
	BaseWaveApplLayer::onBeacon(beaconMsg);

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias

	/*
	if (!downloaders.empty() && (itDL = downloaders.find(sender)) != downloaders.end() && itDL->second->notifiedLinkBreak == false)
	{
		if (itDL->second->myRole == RELAY && RoutingUtils::_length(curPosition, beaconMsg->getSenderPos()) > 200.0)
		{
			EV << "the communication link between downloader and relay will break soon, notify downloader to report its downloading status to RSU.\n";
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

			notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_DR);
			notifyMsg->setReceiver(sender);
			notifyMsg->setDownloader(sender);

			sendWSM(notifyMsg);

			itDL->second->notifiedLinkBreak = true;
		}
	}
	*/

	if (sender == carriedDownloader && encounteredDownloader == false) // carrier come across downloader
	{
		encounteredDownloader = true;

		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *encounterMsg = dynamic_cast<ContentMessage*>(wsm);

		encounterMsg->setControlCode(ContentMsgCC::CARRIER_ENCOUNTER);
		encounterMsg->setReceiver(carriedDownloader);
		encounterMsg->setDownloader(carriedDownloader);
		encounterMsg->setContentSize(downloaders[carriedDownloader]->_cacheOffset.begin);  // reuse function name
		encounterMsg->setReceivedOffset(downloaders[carriedDownloader]->_cacheOffset.end); // reuse function name

		sendWSM(encounterMsg);
	}

	if (sender == rebroadcastDownloader)
	{
		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *rebroadcastMsg = dynamic_cast<ContentMessage*>(wsm);

		rebroadcastMsg->setControlCode(ContentMsgCC::REBROADCAST_BEACON);
		rebroadcastMsg->setDownloader(sender);
		rebroadcastMsg->setPosition(beaconMsg->getSenderPos());
		rebroadcastMsg->setSpeed(beaconMsg->getSenderSpeed());

		sendWSM(rebroadcastMsg);
	}
}

void ContentClient::onRouting(RoutingMessage *routingMsg)
{
	EV << "ContentClient doesn't handle routing message since it is a content download application.\n";
	delete routingMsg;
	routingMsg = nullptr;
}

void ContentClient::onWarning(WarningMessage *warningMsg)
{
	EV << "ContentClient doesn't handle warning message since it is a content download application.\n";
	delete warningMsg;
	warningMsg = nullptr;
}

void ContentClient::onContent(ContentMessage *contentMsg)
{
	EV << "node[" << myAddr << "]: " << "onContent!\n";

	LAddress::L3Type downloader = contentMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	switch (contentMsg->getControlCode())
	{
	case ContentMsgCC::CONTENT_REQUEST: // it is a request message from another downloader
	{
		EV << "it is a request message from another downloader, ignore it.\n";
		break;
	}
	case ContentMsgCC::CONTENT_RESPONSE: // it is a response message from RSU
	{
		if (contentMsg->getDownloader() == myAddr)
		{
			if (contentMsg->getReportAt().isZero())
				EV << "it is a response message aims to me, cancel request timeout event.\n";
			else
			{
				EV << "RSU is distributing data to another downloader, report my downloading status at " << contentMsg->getReportAt().dbl() << "s.\n";
				scheduleAt(contentMsg->getReportAt(), reportStatusEvt);
			}
			cancelEvent(requestTimeoutEvt);
		}
		else
			EV << "it is a response message aims to another downloader, ignore it.\n";
		break;
	}
	case ContentMsgCC::SCHEME_DISTRIBUTION:
	{
		EV << "it is a scheme distribution message, check the downloaders it contains.\n";
		bool acceptSchemeItems = false;
		DownloaderItems downloaderList = contentMsg->getDownloaders();
		for (DownloaderItems::iterator itDLI = downloaderList.begin(); itDLI != downloaderList.end(); ++itDLI)
		{
			if (itDLI->first != myAddr)
			{
				acceptSchemeItems = true;
				if ((itDL = downloaders.find(itDLI->first)) != downloaders.end()) // update old record
				{
					EV << "    downloader [" << itDLI->first << "] is an old downloader, update its info.\n";
					itDL->second->myRole = (itDLI->second.x * curSpeed.x > 0 && neighbors.find(itDLI->first) == neighbors.end()) ? STRANGER : RELAY;
				}
				else // insert new record
				{
					EV << "    downloader [" << itDLI->first << "] is a new downloader, insert its info.\n";
					downloaderInfo = new DownloaderInfo(contentMsg->getContentSize());
					downloaderInfo->myRole = (itDLI->second.x * curSpeed.x > 0 && neighbors.find(itDLI->first) == neighbors.end()) ? STRANGER : RELAY;
					downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(itDLI->first, downloaderInfo));
				}
			}
		}
		if (acceptSchemeItems)
		{
			SchemeItems &schemeItems = contentMsg->getScheme();
			if (schemeItems.find(myAddr) != schemeItems.end()) // there exists a scheme for me
			{
				schemeList = schemeItems[myAddr];
				itSL = schemeList.begin();
				ASSERT( schemeSwitchEvt->isScheduled() == false );

				_adjustCachedSegment();

				schemeSwitchInterval = SimTime(slotSpan*(itSL->slot-1), SIMTIME_MS);
				EV << "prepare to switch to the scheme in slot " << itSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
				scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
			}
		}
		break;
	}
	case ContentMsgCC::ACKNOWLEDGEMENT:
	{
		if ( downloaders.find(downloader) != downloaders.end() )
		{
			cancelEvent(linkBrokenEvt);
			EV << "downloader [" << downloader << "]'s acknowledged offset updates to " << contentMsg->getReceivedOffset() << std::endl;
			downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
			// help to rebroadcast acknowledge message, for downloader might has driven away RSU's communication area
			if (simTime() - prevSlotStartTime < SimTime(slotSpan + 100, SIMTIME_MS))
			{
				EV << "I am the relay in the last slot, thus help to rebroadcast acknowledge message.\n";
				ContentMessage *dupAckMsg = new ContentMessage(*contentMsg);
				dupAckMsg->setSenderAddress(myAddr);
				sendWSM(dupAckMsg);
			}
		}
		break;
	}
	case ContentMsgCC::CARRIER_SELECTION:
	{
		if (contentMsg->getReceiver() == myAddr)
		{
			EV << "it is a carry request message aims to me, response it.\n";

			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *selectMsg = dynamic_cast<ContentMessage*>(wsm);

			selectMsg->setControlCode(ContentMsgCC::CARRIER_SELECTION);
			selectMsg->setReceiver(contentMsg->getSenderAddress());
			selectMsg->setDownloader(downloader);

			sendWSM(selectMsg);

			EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
			downloaderInfo = new DownloaderInfo(contentMsg->getContentSize());
			downloaderInfo->myRole = CARRIER;
			downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
		}
		else
			EV << "it is a carry message aims to another node, ignore it.\n";
		break;
	}
	case ContentMsgCC::CARRIER_ENCOUNTER:
	{
		if (downloader == myAddr)
		{
			EV << "I'm the downloader who encounter my carrier, report downloading status to it.\n";

			Segment cachedOffset;
			cachedOffset.begin = contentMsg->getContentSize();  // reuse function name
			cachedOffset.end = contentMsg->getReceivedOffset(); // reuse function name
			if (downloadingStatus.availableOffset < cachedOffset.end)
				carrierDownloading = true;

			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *reportMsg = dynamic_cast<ContentMessage*>(wsm);

			reportMsg->setControlCode(ContentMsgCC::CARRIER_ENCOUNTER);
			reportMsg->setReceiver(contentMsg->getSenderAddress());
			reportMsg->setDownloader(myAddr);
			reportMsg->setReceivedOffset(downloadingStatus.availableOffset);
			EV << "available offsets: ";
			downloadingStatus.lackSegment(&reportMsg->getLackOffset());
			EV << "lacking offsets: ";
			reportMsg->getLackOffset().print();

			sendWSM(reportMsg);
		}
		else if (contentMsg->getReceiver() == myAddr)
		{
			downloaderInfo = downloaders[downloader];

			EV << "receiving report message from carried downloader, its available offset is " << contentMsg->getReceivedOffset() << ".\n";
			EV << "whereas cached offset is ";
			downloaderInfo->_cacheOffset.print();
			Segment *lackOffsets = NULL;
#ifdef CLIENT_DEBUG
			std::ofstream fout("carried_offset.txt", std::ios_base::out | std::ios_base::app);
			if ( !fout.is_open() )
				throw cRuntimeError("cannot open file carried_offset.txt!");
			else
			{
				lackOffsets = &contentMsg->getLackOffset();
				fout << "At " << simTime().dbl() << ": C " << myAddr << " D " << downloader
						<< "\t[" << downloaderInfo->_cacheOffset.begin << ',' << downloaderInfo->_cacheOffset.end << ")\tL";
				while (lackOffsets != nullptr)
				{
					fout << " [" << lackOffsets->begin << ',' << lackOffsets->end << ')';
					lackOffsets = lackOffsets->next;
				}
				fout << "\n";
			}
			fout.close();
#endif
			if (contentMsg->getReceivedOffset() < downloaderInfo->cacheOffset->end)
			{
				lackOffsets = &contentMsg->getLackOffset();
				while (lackOffsets != nullptr)
				{
					if (lackOffsets->begin >= downloaderInfo->_cacheOffset.begin)
					{
						downloaderInfo->distributedOffset = lackOffsets->begin;
						break;
					}
					lackOffsets = lackOffsets->next;
				}
				if (lackOffsets == nullptr)
					downloaderInfo->distributedOffset = downloaderInfo->_cacheOffset.begin;
				scheduleAt(simTime(), forwardEvt);
			}
			else
			{
				EV << "carried downloader has received the whole file segment cached by me, do nothing.\n";

				carriedDownloader = -1;
				encounteredDownloader = false;
				delete downloaderInfo;
				downloaders.erase(downloader);
			}
		}
		else
			EV << "I'm not downloader who aimed by this carrier, ignore it.\n";
		break;
	}
	case ContentMsgCC::DOWNLOADING_COMPLETED:
	{
		EV << "    downloader [" << downloader << "]'s downloading process has completed, erase its info.\n";
		if ((itDL = downloaders.find(downloader)) != downloaders.end())
		{
			delete itDL->second;
			downloaders.erase(itDL);
			if (downloader == carriedDownloader && encounteredDownloader == false)
				carriedDownloader = -1;
		}
		// help to rebroadcast completion message, for downloader might has driven away RSU's communication area
		if (simTime() - prevSlotStartTime < SimTime(3*slotSpan/2, SIMTIME_MS))
		{
			EV << "I am the relay in the last slot, thus help to rebroadcast completion message.\n";
			ContentMessage *dupAckMsg = new ContentMessage(*contentMsg);
			dupAckMsg->setSenderAddress(myAddr);
			sendWSM(dupAckMsg);
		}
		break;
	}
	case ContentMsgCC::LINK_BREAK_DIRECT:
	{
		if (contentMsg->getReceivedOffset() == 0)
		{	
			if (downloader == myAddr)
			{
				EV << "I'm the downloader who need to report downloading status to RSU.\n";

				_reportDownloadingStatus(ContentMsgCC::LINK_BREAK_DIRECT, contentMsg->getSenderAddress());
			}
			else
				EV << "I'm not the downloader who need to report downloading status to RSU, ignore it.\n";
		}
		else
			EV << "it is a downloading status report sent by downloader, ignore it.\n";
		break;
	}
	case ContentMsgCC::LINK_BREAK_DR:
	{
		if (downloader == myAddr)
		{
			if (contentMsg->getReceivedOffset() == 0)
			{
				EV << "get notified that the link between relay and me will break soon, transfer available offset and neighbors' info to RSU.\n";

				_reportDownloadingStatus(ContentMsgCC::LINK_BREAK_DR, contentMsg->getSenderAddress());
			}
			else
				EV << "it is a link break notification rebroadcast by relay, ignore it.\n";
		}
		else if (contentMsg->getReceiver() == myAddr)
		{
			ASSERT( contentMsg->getReceivedOffset() > 0 );
			EV << "I'm the relay who sent this link break notification before, thus transmit it to RSU.\n";
			ContentMessage *dupResponseMsg = new ContentMessage(*contentMsg);
			dupResponseMsg->setSenderAddress(myAddr);
			sendWSM(dupResponseMsg);
		}
		else if (downloaders.find(downloader) != downloaders.end() && downloaders[downloader]->myRole == RELAY)
			EV << "it is a link break notification sent by another relay, ignore it.\n";
		else
			EV << "I'm not aware of downloader [" << downloader << "] or I'm a stranger of it, ignore it.\n";
		break;
	}
	case ContentMsgCC::LINK_BREAK_RR:
	{
		if (downloader == myAddr)
		{
			if (contentMsg->getReceivedOffset() == 0)
			{
				EV << "get notified that the link between relay and RSU will break soon, transfer available offset and neighbors' info to RSU.\n";

				_reportDownloadingStatus(ContentMsgCC::LINK_BREAK_RR, contentMsg->getSenderAddress());
			}
			else
				EV << "it is a link break notification rebroadcast by relay, ignore it.\n";
		}
		else if (contentMsg->getReceiver() == myAddr)
		{
			ContentMessage *dupResponseMsg = new ContentMessage(*contentMsg);
			dupResponseMsg->setSenderAddress(myAddr);
			if (contentMsg->getReceivedOffset() == 0)
			{
				EV << "I'm the relay whose link with RSU will break soon, notify downloader to report its downloading status to RSU.\n";
				dupResponseMsg->setReceiver(downloader);
			}
			else
			{
				EV << "I'm the relay who rebroadcast this link break notification before, thus transmit it to RSU.\n";
			}
			sendWSM(dupResponseMsg);
		}
		else if (downloaders.find(downloader) != downloaders.end() && downloaders[downloader]->myRole == RELAY)
			EV << "it is a link break notification rebroadcast by another relay, ignore it.\n";
		else
			EV << "I'm not aware of downloader [" << downloader << "] or I'm a stranger of it, ignore it.\n";
		break;
	}
	case ContentMsgCC::LINK_BROKEN_DR:
	{
		EV << "it is a link broken notification sent by another relay, ignore it.\n";
		break;
	}
	case ContentMsgCC::DOWNLOADER_DISCOVERY:
	{
		if (downloader == myAddr)
		{
			EV << "I'm the downloader discovered by cooperative RSU, report my status to it.\n";

			_reportDownloadingStatus2(ContentMsgCC::DISCOVERY_RESPONSE, contentMsg->getSenderAddress());
		}
		break;
	}
	case ContentMsgCC::RELAY_DISCOVERY:
	{
		if (contentMsg->getReceiver() == myAddr)
		{
			EV << "I'm the vehicle who is assumed as a potential relay discovered by cooperative RSU.\n";

			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			if ((itN = neighbors.find(downloader)) != neighbors.end() && RoutingUtils::_length(curPosition, itN->second->pos) < 225.0)
			{
				EV << "I'm still a neighbor of the target downloader, response yes to cooperative RSU.\n";

				ContentMessage *queryMsg = dynamic_cast<ContentMessage*>(wsm);
				queryMsg->setControlCode(ContentMsgCC::STATUS_QUERY);
				queryMsg->setReceiver(downloader);
				queryMsg->setDownloader(downloader);
				sendWSM(queryMsg);
			}
			else
			{
				EV << "I'm not a neighbor of the target downloader anymore, response no to cooperative RSU.\n";

				ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);
				responseMsg->setControlCode(ContentMsgCC::DISCOVERY_RESPONSE);
				responseMsg->setReceiver(contentMsg->getSenderAddress());
				responseMsg->setDownloader(downloader);
				sendWSM(responseMsg);
			}
		}
		break;
	}
	case ContentMsgCC::DISCOVERY_RESPONSE:
	{
		EV << "it is a discovery response message sent by another vehicle, ignore it.\n";
		break;
	}
	case ContentMsgCC::STATUS_QUERY:
	{
		if (downloader == myAddr)
		{
			EV << "it is a downloading status query message aims to me, report my status to it.\n";

			_reportDownloadingStatus2(ContentMsgCC::STATUS_QUERY, contentMsg->getSenderAddress());
		}
		else if (contentMsg->getReceiver() == myAddr)
		{
			EV << "it is a downloading status report message aims to me, help to relay it to cooperative RSU.\n";

			ContentMessage *responseMsg = new ContentMessage(*contentMsg);
			responseMsg->setSenderAddress(myAddr);
			responseMsg->setControlCode(ContentMsgCC::DISCOVERY_RESPONSE);
			responseMsg->setReceiver(myAddr); // RSU doesn't check this and other vehicles ignore this kind of control code, so it is safe
			sendWSM(responseMsg);

			if (contentMsg->getReceivedOffset() < contentMsg->getContentSize())
			{
				rebroadcastDownloader = downloader;
				if (!rebroadcastBeaconEvt->isScheduled())
					scheduleAt(simTime() + SimTime(5, SIMTIME_S), rebroadcastBeaconEvt);
			}
			else
				EV << "    downloader [" << downloader << "]'s downloading process has completed, don't rebroadcast its beacon message.\n";
		}
		else
			EV << "it is a downloading status query message which is unrelated to me, ignore it.\n";
		break;
	}
	case ContentMsgCC::STATUS_REPORT:
	{
		EV << "it is a downloading status report message aims to RSU, ignore it.\n";
		break;
	}
	case ContentMsgCC::REBROADCAST_BEACON:
	{
		EV << "it is a rebroadcast beacon message aims to RSU, ignore it.\n";
		break;
	}
	default:
		EV_WARN << "Unknown control code " << contentMsg->getControlCode() << ", ignore it." << std::endl;
	}
}

void ContentClient::onData(DataMessage *dataMsg)
{
	EV << "node[" << myAddr << "]: " << "onData!\n";

	LAddress::L3Type downloader = dataMsg->getDownloader(); // alias
	int dataStartOffset = dataMsg->getCurOffset() - dataMsg->getBytesNum(); // alias
	int dataEndOffset = dataMsg->getCurOffset(); // alias
	if (downloader == myAddr && dataMsg->getReceiver() == myAddr)
	{
		EV << "I'm the downloader this data message aimed at, check the order of received data.\n";
		if (dataStartOffset == downloadingStatus.availableOffset)
		{
			EV << "current order of data is proper, update available offset to " << dataEndOffset << ".\n";
			downloadingStatus.itS = downloadingStatus.segments.begin(); // fetch the first segment
			downloadingStatus.itS->second = dataEndOffset;
			downloadingStatus.unionSegment(); // handle with segment union process
			downloadingStatus.availableOffset = downloadingStatus.itS->second;
		}
		else if (dataStartOffset > downloadingStatus.availableOffset)
		{
			EV << "current order of data is advanced, expected start offset is " << downloadingStatus.availableOffset << " whereas actual start offset is " << dataStartOffset << ".\n";
			downloadingStatus.insertSegment(dataStartOffset, dataEndOffset); // handle with segment insert process
		}
		else // (dataStartOffset < downloadingStatus.availableOffset)
		{
			if (dataEndOffset > downloadingStatus.availableOffset)
			{
				EV << "current order of data is partially backward, expected start offset is " << downloadingStatus.availableOffset
						<< " whereas actual start offset is " << dataStartOffset << ", end offset is " << dataEndOffset << ".\n";
				downloadingStatus.itS = downloadingStatus.segments.begin(); // fetch the first segment
				downloadingStatus.itS->second = dataEndOffset;
				downloadingStatus.unionSegment(); // handle with segment union process
				downloadingStatus.availableOffset = downloadingStatus.itS->second;
			}
			else
			{
				EV << "current order of data is fully backward, expected start offset is " << downloadingStatus.availableOffset
						<< " whereas actual start offset is " << dataStartOffset << ", end offset is " << dataEndOffset << ", ignore it.\n";
			}
		}

		// it is the last data packet in transmission, thus response an acknowledge message
		if (dataMsg->getIsLast() && downloadingStatus.availableOffset < downloadingStatus.totalContentSize)
		{
			if (carrierDownloading)
			{
				EV << "carrier downloading process has finished.\n";
				carrierDownloading = false;
			}
			else
			{
				EV << "data receiving process has finished, response an acknowledge message.\n";
				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *acknowledgeMsg = dynamic_cast<ContentMessage*>(wsm);

				acknowledgeMsg->setControlCode(ContentMsgCC::ACKNOWLEDGEMENT);
				acknowledgeMsg->setReceiver(LAddress::L3BROADCAST());
				acknowledgeMsg->setDownloader(downloader);
				acknowledgeMsg->setReceivedOffset(downloadingStatus.availableOffset);
				EV << "available offsets: ";
				downloadingStatus.lackSegment(&acknowledgeMsg->getLackOffset());
				EV << "lacking offsets: ";
				acknowledgeMsg->getLackOffset().print();

				sendWSM(acknowledgeMsg);
			}
		}

		if (downloadingStatus.availableOffset == downloadingStatus.totalContentSize)
		{
			EV << "downloading process has finished, response an completion message.\n";

			_notifyDownloadingCompletion();
		}
		return;
	}

	if ((itDL = downloaders.find(downloader)) == downloaders.end())
	{
		EV << "I'm not aware of downloader [" << downloader << "] who is aimed by this data message, ignore it.\n";
		return;
	}

	DownloaderInfo *downloaderInfo = itDL->second;
	switch (downloaderInfo->myRole)
	{
	case RELAY:
	{
		if (dataMsg->getReceiver() == myAddr)
		{
			EV << "I'm the relay this data message aimed at, cooperative cache data for downloader [" << dataMsg->getDownloader() << "].\n";

			_cacheDataSegment(downloader, dataStartOffset, dataEndOffset);
		}
		else
			EV << "I'm not the relay this data message aimed at, ignore it.\n";
		break;
	}
	case CARRIER:
	{
		if (dataMsg->getReceiver() == myAddr)
		{
			EV << "I'm the carrier this data message aimed at, cooperative cache data for downloader [" << dataMsg->getDownloader() << "].\n";

			_cacheDataSegment(downloader, dataStartOffset, dataEndOffset);

			if (dataMsg->getIsLast())
			{
				EV << "receiving process is finished, start paying attention to downloader's beacon message.\n";
				carriedDownloader = downloader;
				encounteredDownloader = false;
			}
		}
		else
			EV << "I'm not the carrier this data message aimed at, ignore it.\n";
		break;
	}
	case STRANGER:
	{
		EV << "I'm a stranger of downloader [" << downloader << "], ignore it.\n";
		break;
	}
	default:
		EV_WARN << "Unknown role " << downloaderInfo->myRole << " I am playing, ignore it.\n";
	}
}

void ContentClient::callContent(int size)
{
	if (downloadingStatus.completeAt == SimTime::ZERO && downloadingStatus.totalContentSize != -1)
	{
		EV_WARN << "haven't finished the previous downloading process, cancel this content request.\n";
		return;
	}

	downloadingStatus.reset();
	downloadingStatus.totalContentSize = size;
	downloadingStatus.requestAt = simTime();
	sendContentRequest();

	++ContentStatisticCollector::globalContentRequests;
	ContentStatisticCollector::globalContentSize += size;

	scheduleAt(simTime() + requestTimeoutDuration, requestTimeoutEvt);
	EV << "set request timeout timer, its duration is " << requestTimeoutDuration.dbl() << "s.\n";
}

void ContentClient::sendContentRequest()
{
	// create message by factory method
	WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
	ASSERT( wsm != nullptr );
	ContentMessage *contentMessage = dynamic_cast<ContentMessage*>(wsm);

	// set necessary content info
	contentMessage->setControlCode(ContentMsgCC::CONTENT_REQUEST);
	contentMessage->setDownloader(myAddr);
	contentMessage->setContentSize(downloadingStatus.totalContentSize);
	contentMessage->setPosition(curPosition);
	contentMessage->setSpeed(curSpeed);

	sendWSM(contentMessage);

	EV << "size = " << downloadingStatus.totalContentSize << std::endl;
}

void ContentClient::_cacheDataSegment(const int downloader, const int startOffset, const int endOffset)
{
	DownloaderInfo *downloaderInfo = downloaders[downloader];
	if (endOffset < downloaderInfo->cacheOffset->end)
		return;
	if (downloaderInfo->cacheOffset->begin == -1) // only the first received segment meets this condition
	{
		downloaderInfo->cacheOffset->begin = startOffset;
		EV << "first segment start offset is " << downloaderInfo->cacheOffset->begin << " bytes.\n";
	}
	else if (downloaderInfo->cacheOffset->end < startOffset)
	{
		downloaderInfo->cacheOffset->next = new Segment;
		downloaderInfo->cacheOffset = downloaderInfo->cacheOffset->next;
		downloaderInfo->cacheOffset->begin = startOffset;
		EV << "receiving an advanced segment, its start offset is " << downloaderInfo->cacheOffset->begin << " bytes.\n";
	}
	downloaderInfo->cacheOffset->end = endOffset;
	ContentStatisticCollector::globalStorageAmount += endOffset - startOffset;
	EV << "current segment end offset updates to " << downloaderInfo->cacheOffset->end << " bytes.\n";
}

void ContentClient::_adjustCachedSegment()
{
	std::list<std::pair<int, int> > offsets;
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		itDL->second->cacheOffset = &itDL->second->_cacheOffset;
		EV << "downloader [" << itDL->first << "]'s distributed offset is " << itDL->second->distributedOffset << ".\n";
		EV << "its cached offsets before adjust: ";
		itDL->second->cacheOffset->print();
		Segment *cachedOffsets = &itDL->second->_cacheOffset;
		while (cachedOffsets != nullptr)
		{
			offsets.push_back(std::pair<int, int>(cachedOffsets->begin, cachedOffsets->end));
			cachedOffsets = cachedOffsets->next;
		}
		while (!offsets.empty() && offsets.front().second <= itDL->second->distributedOffset)
			offsets.pop_front();
		if (!offsets.empty() && offsets.front().first < itDL->second->distributedOffset)
			offsets.front().first = itDL->second->distributedOffset;
		if (!offsets.empty())
		{
			cachedOffsets = &itDL->second->_cacheOffset;
			Segment *_cachedOffsets = cachedOffsets;
			while (!offsets.empty())
			{
				cachedOffsets->begin = offsets.front().first;
				cachedOffsets->end = offsets.front().second;
				cachedOffsets = cachedOffsets->next;
				offsets.pop_front();
			}
			while (_cachedOffsets->next != cachedOffsets)
				_cachedOffsets = _cachedOffsets->next;
			delete _cachedOffsets->next;
			_cachedOffsets->next = nullptr;
		}
		else
		{
			itDL->second->cacheOffset->begin = -1;
			itDL->second->cacheOffset->end = -1;
			delete itDL->second->cacheOffset->next;
			itDL->second->cacheOffset->next = nullptr;
		}
		EV << "its cached offsets after adjust: ";
		itDL->second->cacheOffset->print();
		offsets.clear();
	}
}

void ContentClient::_prepareSchemeSwitch()
{
	int prevSlot = itSL->slot;
	if (++itSL != schemeList.end()) // prepare to schedule self message schemeSwitchEvt
	{
		ASSERT( schemeSwitchEvt->isScheduled() == false );
		SimTime dispatchedSlotInterval(slotSpan*(itSL->slot - prevSlot), SIMTIME_MS);
		schemeSwitchInterval = dispatchedSlotInterval - (simTime() - prevSlotStartTime);
		EV << "prepare to switch to the scheme in slot " << itSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
		scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
	}
	else
		schemeList.clear();
}

void ContentClient::_correctPlannedOffset()
{
	// correct itSL->offset by DownloaderInfo->cacheOffset, for relay can only transmit the cached data to downloader
	Segment *offsets = itSL->offset, *_cacheOffsets = &downloaders[itSL->downloader]->_cacheOffset; // operate on duplicate pointers
	EV << "downloader [" << itSL->downloader << "]'s cached offsets: ";
	_cacheOffsets->print();
	EV << "its planned offsets before correct: ";
	itSL->offset->print();
	while (offsets != nullptr)
	{
		Segment *cacheOffsets = _cacheOffsets;
		while (cacheOffsets != nullptr && offsets->begin >= cacheOffsets->end)
			cacheOffsets = cacheOffsets->next;
		// cached    [_______]    [______]                [_______]        [______]
		// it->SL                        |______|    or           |______|
		//  ==>
		if (cacheOffsets == nullptr || offsets->end < cacheOffsets->begin)
			offsets->end = offsets->begin;
		else
		{
			// cached         [___________]
			// it->SL    |__________|
			//  ==>           |_____|
			if (offsets->begin < cacheOffsets->begin)
				offsets->begin = cacheOffsets->begin;
			// cached         [___________]
			// it->SL               |__________|
			//  ==>                 |_____|
			if (offsets->end > cacheOffsets->end)
			{
				bool lostMiddleSegment = cacheOffsets->next != nullptr && cacheOffsets->next->begin < offsets->end;
				// cached     [_____._____]   [____]   [___]   [_____._______]
				// it->SL           |________________________________|
				//  ==>             |_____|   |____|   |___|   |_____|
				while (cacheOffsets->next != nullptr && cacheOffsets->next->begin < offsets->end)
				{
					Segment *newSeg = new Segment;
					newSeg->begin = cacheOffsets->next->begin;
					newSeg->end = offsets->end;
					offsets->end = cacheOffsets->end;
					newSeg->next = offsets->next;
					offsets->next = newSeg;
					offsets = offsets->next;
					cacheOffsets = cacheOffsets->next;
				}
				if (!lostMiddleSegment || offsets->end > cacheOffsets->end)
					offsets->end = cacheOffsets->end;
			}
		}
		offsets = offsets->next;
	}
	EV << "its planned offsets after correct: ";
	itSL->offset->print();
}

void ContentClient::_reportDownloadingStatus(const int contentMsgCC, const LAddress::L3Type receiver)
{
	WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
	ASSERT( wsm != nullptr );
	ContentMessage *reportMsg = dynamic_cast<ContentMessage*>(wsm);

	reportMsg->setControlCode(contentMsgCC);
	reportMsg->setReceiver(receiver);
	reportMsg->setDownloader(myAddr);
	reportMsg->setReceivedOffset(downloadingStatus.segments.back().second); // ignore the lost segments, report the most advanced segment's end offset
	reportMsg->setPosition(curPosition);
	reportMsg->setSpeed(curSpeed);
	NeighborItems &neighborItems = reportMsg->getNeighbors();
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		if (itN->second->speed.x * curSpeed.x > 0) // only count same driving direction neighbors
			neighborItems.push_back(itN->first);

	sendWSM(reportMsg);
}

void ContentClient::_reportDownloadingStatus2(const int contentMsgCC, const LAddress::L3Type receiver)
{
	WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
	ASSERT( wsm != nullptr );
	ContentMessage *reportMsg = dynamic_cast<ContentMessage*>(wsm);

	reportMsg->setControlCode(contentMsgCC);
	reportMsg->setReceiver(receiver);
	reportMsg->setDownloader(myAddr);
	reportMsg->setContentSize(downloadingStatus.totalContentSize);
	reportMsg->setReceivedOffset(downloadingStatus.availableOffset);
	EV << "available offsets: ";
	downloadingStatus.lackSegment(&reportMsg->getLackOffset());
	EV << "lacking offsets: ";
	reportMsg->getLackOffset().print();
	reportMsg->setPosition(curPosition);
	reportMsg->setSpeed(curSpeed);
	reportMsg->getNeighbors().push_back(myAddr);

	sendWSM(reportMsg);
}

void ContentClient::_notifyDownloadingCompletion()
{
	WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
	ASSERT( wsm != nullptr );
	ContentMessage *completionMsg = dynamic_cast<ContentMessage*>(wsm);

	completionMsg->setControlCode(ContentMsgCC::DOWNLOADING_COMPLETED);
	completionMsg->setReceiver(LAddress::L3BROADCAST());
	completionMsg->setDownloader(myAddr);

	sendWSM(completionMsg);

	// record statistic
	downloadingStatus.completeAt = simTime();
	double downloadingTime = downloadingStatus.completeAt.dbl() - downloadingStatus.requestAt.dbl();
	ContentStatisticCollector::globalDownloadingTime += downloadingTime;
#ifdef CLIENT_DEBUG
	std::ofstream fout("complete_at.txt", std::ios_base::out | std::ios_base::app);
	if (!fout.is_open())
		throw cRuntimeError("cannot open file complete_at.txt!");
	else
		fout << "At " << downloadingStatus.completeAt.dbl() << ": D " << myAddr << " T " << downloadingTime << "\n";
	fout.close();
#endif
}

/////////////////////////    internal class implementations    /////////////////////////
void ContentClient::DownloadingInfo::reset()
{
	totalContentSize = availableOffset = 0;
	completeAt = requestAt = SimTime::ZERO;
	segments.clear();
	segments.push_back(std::pair<int, int>(0, 0));
	segmentNum = 1;
	itS = segments.begin();
}

void ContentClient::DownloadingInfo::eraseSegment()
{
	segments.erase(itD);
	--segmentNum;
}

void ContentClient::DownloadingInfo::insertSegment(int startOffset, int endOffset)
{
	itS = segments.begin();
	while (itS != segments.end() && itS->second < startOffset)
		++itS;
	// itS    |________|                            |________|
	// new               |____|      or      |____|
	// ==>    |________| |____|              |____| |________|
	if (itS == segments.end() || itS->first > endOffset)
	{
		segments.insert(itS, std::pair<int, int>(startOffset, endOffset));
		++segmentNum;
	}
	else
	{
		// itS       |________|                |________|
		// new    |______|          or      |_______________|
		// ==>    |___________|             |___________|
		if (itS->first <= endOffset && itS->first > startOffset)
			itS->first = startOffset;
		// itS       |________|             |___________|
		// new            |______|  or      |_______________|
		// ==>       |___________|          |_______________|
		if (itS->second >= startOffset && itS->second < endOffset)
		{
			//       |__itS__|    |_itD_|            |__itS__|    |_itL_|
			// new         |______________|    or          |_______|        or
			// ==>   |____________________|          |__________________|
			//       |__itS__|    |_itD_|  |_itD_|            |__itS__|    |_itD_|  |_itL_|
			// new         |_______________________|    or          |________________|
			// ==>   |_____________________________|          |___________________________|
			std::list<std::pair<int, int> >::iterator itL = itS;
			while (itL != segments.end() && itL->second < endOffset)
				++itL;
			bool eraseItL = itL != segments.end() && itL->first >= endOffset;
			itS->second = eraseItL ? itL->second : endOffset;
			while (true)
			{
				itD = itS;
				if (++itD == itL)
					break;
				eraseSegment();
			}
			if (eraseItL)
				eraseSegment();
		}
	}
}

void ContentClient::DownloadingInfo::unionSegment()
{
	while (segmentNum > 1) // union segment whenever possible
	{
		itD = itS;
		++itD;
		if (itS->second >= itD->first) // there exists intersection between adjacent segment
		{
			if (itS->second < itD->second)
				itS->second = itD->second;
			eraseSegment();
		}
		else
			break;
	}
}

void ContentClient::DownloadingInfo::lackSegment(Segment *lackOffsets)
{
	Segment *ackOffsets = new Segment, *storeAckOffsets = ackOffsets;
	itS = segments.begin();
	ackOffsets->begin = itS->first;
	ackOffsets->end = itS->second;
	for (++itS; itS != segments.end(); ++itS)
	{
		ackOffsets->next = new Segment;
		ackOffsets = ackOffsets->next;
		ackOffsets->begin = itS->first;
		ackOffsets->end = itS->second;
	}
	ackOffsets = storeAckOffsets;
	ackOffsets->print();

	while (ackOffsets->next != nullptr)
	{
		lackOffsets->begin = ackOffsets->end;
		lackOffsets->end = ackOffsets->next->begin;
		if (ackOffsets->next->end < totalContentSize)
		{
			lackOffsets->next = new Segment;
			lackOffsets = lackOffsets->next;
		}
		ackOffsets = ackOffsets->next;
	}
	if (ackOffsets->end < totalContentSize)
	{
		lackOffsets->begin = ackOffsets->end;
		lackOffsets->end = totalContentSize;
	}

	delete ackOffsets;
}
