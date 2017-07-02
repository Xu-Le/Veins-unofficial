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

Define_Module(ContentClient);

void ContentClient::initialize(int stage)
{
	BaseWaveApplLayer::initialize(stage);

	if (stage == 0)
	{
		cModule *baseStationModule = getModuleByPath("scenario.BS");
		baseStation = dynamic_cast<BaseStation*>(baseStationModule);
		ASSERT(baseStation);
		baseStationGate = baseStation->gate(baseStation->wirelessIn);
		cellularHeaderLength = baseStation->wirelessHeaderLength;
		cellularBitsRate = baseStation->wirelessBitsRate;

		startPlaying = false;
		cellularDownloading = false;
		waitingDistribution = false;
		encounteredDownloader = false;
		carriedDownloader = -1;

		slotSpan = par("slotSpan").longValue();
		prevSlotStartTime = SimTime::ZERO;
		relayLinkBytesOnce = 10 * (headerLength + dataLengthBits) / 8; // relayLinkPacketsOnce == 10
		relayApplBytesOnce = 10 * dataLengthBits / 8;
		relayPeriod = SimTime::ZERO;

		requestTimeoutDuration = SimTime(100, SIMTIME_MS);
		interruptTimeoutDuration = SimTime(5, SIMTIME_S);

		relayEvt = new cMessage("relay evt", ContentClientMsgKinds::RELAY_EVT);
		forwardEvt = new cMessage("forward evt", ContentClientMsgKinds::FORWARD_EVT);
		relayEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at downloader
		forwardEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at downloader
		schemeSwitchEvt = new cMessage("scheme switch evt", ContentClientMsgKinds::SCHEME_SWITCH_EVT);
		segmentAdvanceEvt = new cMessage("segment advance evt", ContentClientMsgKinds::SEGMENT_ADVANCE_EVT);
		reportStatusEvt = new cMessage("report status evt", ContentClientMsgKinds::REPORT_STATUS_EVT);
		dataConsumptionEvt = new cMessage("data consumption evt", ContentClientMsgKinds::DATA_CONSUMPTION_EVT);
		requestTimeoutEvt = new cMessage("request timeout evt", ContentClientMsgKinds::REQUEST_TIMEOUT_EVT);
		interruptTimeoutEvt = new cMessage("interrupt timeout evt", ContentClientMsgKinds::INTERRUPT_TIMEOUT_EVT);
		interruptTimeoutEvt->setSchedulingPriority(1); // ensure data consumption event scheduled first to avoid rescheduling interrupt timeout event
		linkBrokenEvt = new cMessage("link broken evt", ContentClientMsgKinds::LINK_BROKEN_EVT);
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
	cancelAndDelete(dataConsumptionEvt);
	cancelAndDelete(requestTimeoutEvt);
	cancelAndDelete(interruptTimeoutEvt);
	cancelAndDelete(linkBrokenEvt);

	BaseWaveApplLayer::finish();
}

void ContentClient::handleSelfMsg(cMessage* msg)
{
	bool isSegmentAdvanceEvt = true;
	bool isInterruptTimeoutEvt = true;
	switch (msg->getKind())
	{
	case ContentClientMsgKinds::SCHEME_SWITCH_EVT:
	{
		if (downloaders.find(itSL->downloader) == downloaders.end())
		{
			EV << "communication link with this downloader has broken, skip current planned scheme.\n";
			_prepareSchemeSwitch();
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

		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(itSL->receiver);
		dataMsg->setDownloader(itSL->downloader);
		DownloaderInfo *downloaderInfo = downloaders[itSL->downloader];
		// estimate transmission rate between self and downloader only taking relative distance into consideration
		int distance = RoutingUtils::_length(curPosition, neighbors[itSL->receiver]->pos);
		ASSERT( distance < 250 );
		int estimatedRate = ContentUtils::rateTable[distance/5];
		EV << "estimated rate between myself and receiver [" << itSL->receiver << "] is " << 128*estimatedRate << " bytes per second.\n";
		if (distance >= 200 && !downloaderInfo->notifiedLinkBreak)
		{
			EV << "the communication link between downloader and relay will break soon, notify downloader to report its downloading status to RSU.\n";
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

			notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_DR);
			notifyMsg->setReceiver(itSL->downloader);
			notifyMsg->setDownloader(itSL->downloader);

			sendWSM(notifyMsg);

			downloaderInfo->notifiedLinkBreak = true;
		}
		if (itSL->offset->end - itSL->offset->begin > relayApplBytesOnce)
		{
			dataMsg->setIsLast(false);
			dataMsg->setBytesNum(relayApplBytesOnce);
			// dataMsg->addBitLength(8*relayLinkBytesOnce);
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
		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(carriedDownloader);
		dataMsg->setDownloader(carriedDownloader);
		DownloaderInfo *downloaderInfo = downloaders[carriedDownloader];
		// estimate transmission rate between self and downloader only taking relative distance into consideration
		int distance = RoutingUtils::_length(curPosition, neighbors[carriedDownloader]->pos);
		ASSERT( distance < 250 );
		int estimatedRate = ContentUtils::rateTable[distance/5];
		EV << "estimated rate between myself and receiver [" << carriedDownloader << "] is " << 128*estimatedRate << " bytes per second.\n";
		SimTime distributeCDPeriod(SimTime::ZERO);
		if (downloaderInfo->cacheOffset->end - downloaderInfo->distributedOffset > relayApplBytesOnce)
		{
			bool endTransmission = distance >= 200 && (curPosition.x -  neighbors[carriedDownloader]->pos.x)*curSpeed.x > 0;
			dataMsg->setIsLast(endTransmission); // due to their opposite direction moving, it is 200 not 225
			dataMsg->setBytesNum(relayApplBytesOnce);
			// dataMsg->addBitLength(8*relayLinkBytesOnce);
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
		}
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
		reportMsg->setConsumedOffset(downloadingStatus.consumedOffset);
		reportMsg->setConsumingRate(downloadingStatus.consumingRate);
		EV << "consumed offset: " << downloadingStatus.consumedOffset << ", available offsets: ";
		downloadingStatus.lackSegment(&reportMsg->getLackOffset());

		sendWSM(reportMsg);
		break;
	}
	case ContentClientMsgKinds::DATA_CONSUMPTION_EVT:
	{
		if (downloadingStatus.availableOffset - downloadingStatus.consumedOffset >= downloadingStatus.consumingRate)
			downloadingStatus.consumedOffset += downloadingStatus.consumingRate;
		else
		{
			if (downloadingStatus.availableOffset < downloadingStatus.totalContentSize) // set the interruption timer, fetch data through cellular approach if timeout
			{
				if (!cellularDownloading && !interruptTimeoutEvt->isScheduled())
				{
					double correctedTimeoutDuration = interruptTimeoutDuration.dbl() -
							static_cast<double>(downloadingStatus.availableOffset-downloadingStatus.consumedOffset)/downloadingStatus.consumingRate;
					scheduleAt(simTime() + correctedTimeoutDuration, interruptTimeoutEvt);
					EV << "set interrupt timeout timer, its duration is " << correctedTimeoutDuration << "s.\n";
				}
			}
			downloadingStatus.consumedOffset = downloadingStatus.availableOffset;
		}
		EV << "downloader [" << myAddr << "]'s consumed offset updates to " << downloadingStatus.consumedOffset << " bytes.\n";

		if (downloadingStatus.consumedOffset < downloadingStatus.totalContentSize) // continue consuming process
			scheduleAt(simTime() + SimTime(slotSpan, SIMTIME_MS), dataConsumptionEvt);
		else // consuming process has finished, record statistic
		{
			downloadingStatus.consumingEndAt = simTime();
			double totalInterruptedTime = downloadingStatus.consumingEndAt.dbl() - downloadingStatus.requestAt.dbl()
					- downloadingStatus.totalContentSize/downloadingStatus.consumingRate - CACHE_TIME_BEFORE_PLAY;
			EV << "consuming process has finished, total interrupt time in consuming process is " << totalInterruptedTime << "s.\n";
			ContentStatisticCollector::globalConsumingTime += downloadingStatus.consumingEndAt.dbl() - downloadingStatus.consumingBeginAt.dbl();
			ContentStatisticCollector::globalInterruptedTime += totalInterruptedTime;
		}
		break;
	}
	case ContentClientMsgKinds::REQUEST_TIMEOUT_EVT:
	{
		EV << "content request is timeout, now fetching data from cellular network.\n";
		isInterruptTimeoutEvt = false;
	}
	case ContentClientMsgKinds::INTERRUPT_TIMEOUT_EVT:
	{
		if (isInterruptTimeoutEvt)
		{
			EV << "data consuming process interruption is timeout, now fetching data from cellular network.\n";
			ContentStatisticCollector::globalInterruptedTime += interruptTimeoutDuration.dbl();
		}

		CellularMessage *cellularMsg = new CellularMessage("cellular-open");
		cellularMsg->setControlCode(CellularMsgCC::TRANSMISSION_STARTING);
		cellularMsg->setDownloader(myAddr);
		cellularMsg->setContentSize(downloadingStatus.totalContentSize);
		cellularMsg->setStartOffset(downloadingStatus.availableOffset);
		if (downloadingStatus.segmentNum > 1)
		{
			downloadingStatus.itS = downloadingStatus.segments.begin();
			++downloadingStatus.itS;
			cellularMsg->setEndOffset(downloadingStatus.itS->first); // download lacking segment
		}
		else
			cellularMsg->setEndOffset(downloadingStatus.totalContentSize);
		cellularMsg->addBitLength(cellularHeaderLength);
		SimTime transmissionDelay(1000*cellularHeaderLength/(cellularBitsRate/1000), SIMTIME_US);
		sendDirect(cellularMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
		cellularDownloading = true;
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

		downloaders.erase(brokenDownloader);
		break;
	}
	default:
		BaseWaveApplLayer::handleSelfMsg(msg);
	}
}

void ContentClient::handleCellularMsg(CellularMessage *cellularMsg)
{
	EV << "node[" << myAddr << "]: " << "handleCellularMsg!\n";

	ASSERT( cellularMsg->getControlCode() == CellularMsgCC::DATA_PACKET_NORMAL || cellularMsg->getControlCode() == CellularMsgCC::DATA_PACKET_LAST );
	ASSERT( cellularMsg->getDownloader() == myAddr ); // sendDirect() ensure this being always true

	downloadingStatus.availableOffset = cellularMsg->getCurOffset();
	EV << "available offset updates to " << downloadingStatus.availableOffset << " bytes.\n";
	downloadingStatus.itS = downloadingStatus.segments.begin();
	downloadingStatus.itS->second = downloadingStatus.availableOffset;

	if (cellularMsg->getControlCode() == CellularMsgCC::DATA_PACKET_LAST) // it is the last data packet, indicating current segment downloading progress has finished
	{
		SimTime transmissionDelay(1000*cellularHeaderLength/(cellularBitsRate/1000), SIMTIME_US);
		if (downloadingStatus.segmentNum > 1)
		{
			downloadingStatus.itD = downloadingStatus.segments.begin();
			++downloadingStatus.itD;
			ASSERT( downloadingStatus.availableOffset == downloadingStatus.itD->first );
			downloadingStatus.availableOffset = downloadingStatus.itD->second; // now available offset becomes next segment end offset
			downloadingStatus.eraseSegment();
			downloadingStatus.itS->second = downloadingStatus.availableOffset;
			EV << "after union segment, available offset updates to " << downloadingStatus.availableOffset << " bytes.\n";

			if (downloadingStatus.segmentNum > 1 || downloadingStatus.availableOffset < downloadingStatus.totalContentSize)
			{
				EV << "downloading process has not finished, continue to fetch data from cellular network.\n";

				CellularMessage *requestMsg = new CellularMessage("cellular-open");
				requestMsg->setControlCode(CellularMsgCC::TRANSMISSION_STARTING);
				requestMsg->setDownloader(myAddr);
				requestMsg->setContentSize(downloadingStatus.totalContentSize);
				requestMsg->setStartOffset(downloadingStatus.availableOffset);
				if (downloadingStatus.segmentNum > 1)
				{
					++downloadingStatus.itS;
					requestMsg->setEndOffset(downloadingStatus.itS->first); // download lacking segment
				}
				else
					requestMsg->setEndOffset(downloadingStatus.totalContentSize);
				requestMsg->addBitLength(cellularHeaderLength);
				sendDirect(requestMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
				cellularDownloading = true;
			}
			else
			{
				EV << "downloading process has finished, response a message to base station.\n";
				CellularMessage *responseMsg = new CellularMessage("cellular-close");
				responseMsg->setControlCode(CellularMsgCC::DOWNLOADING_COMPLETING);
				responseMsg->setDownloader(myAddr);
				responseMsg->addBitLength(cellularHeaderLength);
				sendDirect(responseMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
				cellularDownloading = false;
			}
		}
		else if (downloadingStatus.availableOffset == downloadingStatus.totalContentSize)
		{
			EV << "downloading process has finished, response a message to base station.\n";
			CellularMessage *responseMsg = new CellularMessage("cellular-close");
			responseMsg->setControlCode(CellularMsgCC::DOWNLOADING_COMPLETING);
			responseMsg->setDownloader(myAddr);
			responseMsg->addBitLength(cellularHeaderLength);
			sendDirect(responseMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
			cellularDownloading = false;
		}
	}
	delete cellularMsg;
	cellularMsg = nullptr;
}

void ContentClient::onBeacon(BeaconMessage *beaconMsg)
{
	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias

	BaseWaveApplLayer::onBeacon(beaconMsg);

	if (sender == carriedDownloader && encounteredDownloader == false) // carrier come across downloader
	{
		encounteredDownloader = true;

		WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
		ASSERT( wsm != nullptr );
		ContentMessage *encounterMsg = dynamic_cast<ContentMessage*>(wsm);

		encounterMsg->setControlCode(ContentMsgCC::CARRIER_ENCOUNTER);
		encounterMsg->setReceiver(carriedDownloader);
		encounterMsg->setDownloader(carriedDownloader);

		sendWSM(encounterMsg);
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
			{
				EV << "it is a response message aims to me, cancel request timeout event.\n";
				cancelEvent(requestTimeoutEvt);
			}
			else
			{
				EV << "RSU is distributing data to another downloader, report my downloading status at " << contentMsg->getReportAt().dbl() << "s.\n";
				scheduleAt(contentMsg->getReportAt(), reportStatusEvt);
			}
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
				if ( downloaders.find(itDLI->first) != downloaders.end() ) // update old record
					EV << "    downloader [" << itDLI->first << "] is an old downloader, do nothing.\n";
				else // insert new record
				{
					EV << "    downloader [" << itDLI->first << "] is a new downloader, insert its info.\n";
					downloaderInfo = new DownloaderInfo(contentMsg->getContentSize());
					downloaderInfo->myRole = (neighbors.find(itDLI->first) == neighbors.end() && itDLI->second.x * curSpeed.x > 0) ? STRANGER : RELAY;
					downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(itDLI->first, downloaderInfo));
				}
			}
			else
			{
				EV << "    downloader [" << itDLI->first << "] is me, close cellular connection if currently downloading from base station.\n";
				if (cellularDownloading)
					_closeCellularConnection();
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
			if (simTime() - prevSlotStartTime < SimTime(3*slotSpan/2, SIMTIME_MS))
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

			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *reportMsg = dynamic_cast<ContentMessage*>(wsm);

			reportMsg->setControlCode(ContentMsgCC::CARRIER_ENCOUNTER);
			reportMsg->setReceiver(contentMsg->getSenderAddress());
			reportMsg->setDownloader(myAddr);
			reportMsg->setReceivedOffset(downloadingStatus.availableOffset);

			sendWSM(reportMsg);

			_closeCellularConnection();
		}
		else if (contentMsg->getReceiver() == myAddr)
		{
			EV << "receiving report message from carried downloader.\n";

			downloaderInfo = downloaders[downloader];
			if (contentMsg->getReceivedOffset() < downloaderInfo->cacheOffset->end)
			{
				downloaderInfo->distributedOffset = contentMsg->getReceivedOffset();
				scheduleAt(simTime(), forwardEvt);
			}
			else
			{
				EV << "carried downloader has received the whole file segment cached by me, do nothing.\n";
				carriedDownloader = -1;
				encounteredDownloader = false;
			}
		}
		else
			EV << "I'm not downloader who aimed by this carrier, ignore it.\n";
		break;
	}
	case ContentMsgCC::DOWNLOADING_COMPLETED:
	{
		EV << "    downloader [" << downloader << "]'s downloading process has completed, erase its info.\n";
		downloaders.erase(downloader);
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
	case ContentMsgCC::RELAY_DISCOVERY:
	{
		if (contentMsg->getReceiver() == myAddr)
		{
			EV << "I'm the vehicle who is assumed as a potential relay discovered by cooperative RSU.\n";

			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			if (neighbors.find(downloader) != neighbors.end())
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

			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *reportMsg = dynamic_cast<ContentMessage*>(wsm);

			reportMsg->setControlCode(ContentMsgCC::STATUS_QUERY);
			reportMsg->setReceiver(contentMsg->getSenderAddress());
			reportMsg->setDownloader(downloader);
			reportMsg->setReceivedOffset(downloadingStatus.availableOffset);
			reportMsg->setConsumedOffset(downloadingStatus.consumedOffset);
			reportMsg->setConsumingRate(downloadingStatus.consumingRate);
			EV << "consumed offset: " << downloadingStatus.consumedOffset << ", available offsets: ";
			downloadingStatus.lackSegment(&reportMsg->getLackOffset());
			EV << "lacking offsets: ";
			reportMsg->getLackOffset().print();
			reportMsg->setPosition(curPosition);
			reportMsg->setSpeed(curSpeed);

			sendWSM(reportMsg);
		}
		else if (contentMsg->getReceiver() == myAddr)
		{
			EV << "it is a downloading status report message aims to me, help to relay it to cooperative RSU.\n";

			ContentMessage *responseMsg = new ContentMessage(*contentMsg);
			responseMsg->setSenderAddress(myAddr);
			responseMsg->setControlCode(ContentMsgCC::DISCOVERY_RESPONSE);
			responseMsg->setReceiver(myAddr); // RSU doesn't check this and other vehicles ignore this kind of control code, so it is safe
			responseMsg->setDownloader(downloader);
			responseMsg->getNeighbors().push_back(downloader);
			sendWSM(responseMsg);
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
		if (interruptTimeoutEvt->isScheduled() && downloadingStatus.availableOffset - downloadingStatus.consumedOffset >= downloadingStatus.consumingRate)
		{
			SimTime sinceInterrupt = simTime() - interruptTimeoutEvt->getSendingTime();
			ContentStatisticCollector::globalInterruptedTime += sinceInterrupt.dbl();
			EV << "cached data is adequate again, cancel the interrupt timer which elapsed " << sinceInterrupt.dbl() << "s.\n";
			cancelEvent(interruptTimeoutEvt);

			_closeCellularConnection();
		}

		if (dataMsg->getIsLast()) // it is the last data packet in transmission, thus response an acknowledge message
		{
			// check whether the vedio play starting condition is satisified, note this judgement is checked
			// when a transmission process has just finish for offsets synchronous computation purpose
			if (downloadingStatus.availableOffset > CACHE_TIME_BEFORE_PLAY*downloadingStatus.consumingRate && !startPlaying)
			{
				EV << "cached data amount is enough, start consuming process.\n";
				startPlaying = true;
				downloadingStatus.consumingBeginAt = simTime();
				ContentStatisticCollector::globalConsumptionStartingDelay += downloadingStatus.consumingBeginAt.dbl() - downloadingStatus.requestAt.dbl();
				scheduleAt(simTime() + SimTime(slotSpan, SIMTIME_MS), dataConsumptionEvt);
			}

			if (downloadingStatus.availableOffset < downloadingStatus.totalContentSize)
			{
				EV << "data receiving process has finished, response an acknowledge message.\n";
				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *acknowledgeMsg = dynamic_cast<ContentMessage*>(wsm);

				acknowledgeMsg->setControlCode(ContentMsgCC::ACKNOWLEDGEMENT);
				acknowledgeMsg->setReceiver(LAddress::L3BROADCAST());
				acknowledgeMsg->setDownloader(downloader);
				acknowledgeMsg->setReceivedOffset(downloadingStatus.availableOffset);
				acknowledgeMsg->setConsumedOffset(downloadingStatus.consumedOffset);
				EV << "consumed offset: " << downloadingStatus.consumedOffset << ", available offsets: ";
				downloadingStatus.lackSegment(&acknowledgeMsg->getLackOffset());
				EV << "lacking offsets: ";
				acknowledgeMsg->getLackOffset().print();

				sendWSM(acknowledgeMsg);
			}
		}

		if (downloadingStatus.availableOffset == downloadingStatus.totalContentSize)
		{
			EV << "downloading process has finished, response an completion message.\n";
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *completionMsg = dynamic_cast<ContentMessage*>(wsm);

			completionMsg->setControlCode(ContentMsgCC::DOWNLOADING_COMPLETED);
			completionMsg->setDownloader(downloader);

			sendWSM(completionMsg);

			// record statistic
			downloadingStatus.completeAt = simTime();
			ContentStatisticCollector::globalDownloadingTime += downloadingStatus.completeAt.dbl() - downloadingStatus.requestAt.dbl();
		}
		return;
	}

	if (downloaders.find(downloader) == downloaders.end())
	{
		EV << "I'm not aware of downloader [" << downloader << "] who is aimed by this data message, ignore it.\n";
		return;
	}

	DownloaderInfo *downloaderInfo = downloaders[downloader];
	switch (downloaderInfo->myRole)
	{
	case RELAY:
	{
		if (dataMsg->getReceiver() == myAddr)
		{
			EV << "I'm the relay this data message aimed at, cooperative cache data for downloader [" << dataMsg->getDownloader() << "].\n";

			_cacheDataSegment(downloader, dataMsg->getCurOffset() - dataMsg->getBytesNum(), dataMsg->getCurOffset());
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

			_cacheDataSegment(downloader, dataMsg->getCurOffset() - dataMsg->getBytesNum(), dataMsg->getCurOffset());

			if (dataMsg->getIsLast())
			{
				EV << "receiving process is finished, start paying attention to downloader's beacon message.\n";
				carriedDownloader = downloader;
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
	// create message by factory method
	WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
	ASSERT( wsm != nullptr );
	ContentMessage *contentMessage = dynamic_cast<ContentMessage*>(wsm);

	// handle utils about message's GUID
	int guid = RoutingUtils::generateGUID();
	EV << "GUID = " << guid << ", size = " << size << std::endl;
	guidUsed.push_back(guid);
	scheduleAt(simTime() + guidUsedTime, recycleGUIDEvt);

	downloadingStatus.reset();
	downloadingStatus.totalContentSize = size;
	downloadingStatus.consumingRate = VideoQuailty::_VGA;
	downloadingStatus.requestAt = simTime();
	ASSERT( downloadingStatus.totalContentSize > CACHE_TIME_BEFORE_PLAY*downloadingStatus.consumingRate );

	// set necessary content info
	contentMessage->setGUID(guid);
	contentMessage->setControlCode(ContentMsgCC::CONTENT_REQUEST);
	contentMessage->setDownloader(myAddr);
	contentMessage->setContentSize(size);
	contentMessage->setConsumingRate(downloadingStatus.consumingRate);

	sendWSM(contentMessage);
	++ContentStatisticCollector::globalContentRequests;

	scheduleAt(simTime() + requestTimeoutDuration, requestTimeoutEvt);
	EV << "set request timeout timer, its duration is " << requestTimeoutDuration.dbl() << "s.\n";
}

void ContentClient::_cacheDataSegment(const int downloader, const int startOffset, const int endOffset)
{
	DownloaderInfo *downloaderInfo = downloaders[downloader];
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
	ASSERT( downloaderInfo->cacheOffset->begin < endOffset );
	downloaderInfo->cacheOffset->end = endOffset;
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
				if (!lostMiddleSegment)
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

void ContentClient::_closeCellularConnection()
{
	if (cellularDownloading)
	{
		EV << "close cellular connection.\n";

		CellularMessage *cellularMsg = new CellularMessage("cellular-close");
		cellularMsg->setControlCode(CellularMsgCC::TRANSMISSION_ENDING);
		cellularMsg->setDownloader(myAddr);
		cellularMsg->addBitLength(cellularHeaderLength);
		SimTime transmissionDelay(1000*cellularHeaderLength/(cellularBitsRate/1000), SIMTIME_US);
		sendDirect(cellularMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
		cellularDownloading = false; // calling _closeCellularConnection() more than once is safe
	}
}

/////////////////////////    internal class implementations    /////////////////////////
void ContentClient::DownloadingInfo::reset()
{
	totalContentSize = availableOffset = consumedOffset = consumingRate = 0;
	interruptAt = SimTime::ZERO;
	consumingEndAt = consumingBeginAt = completeAt = requestAt = SimTime::ZERO;
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
	while (itS->second < startOffset && itS != segments.end())
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
			itS->second = endOffset;
			//       |___itS___|    |__itD__|
			// new          |_______|
			// ==>   |______________________|
			itD = itS;
			if (++itD != segments.end() && itS->second == itD->first)
			{
				itS->second = itD->second;
				eraseSegment();
			}
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

