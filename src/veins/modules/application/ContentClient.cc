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

		slotSpan = par("slotSpan").longValue();
		prevSlotStartTime = SimTime::ZERO;
		relayLinkBytesOnce = 100 * (headerLength + dataLengthBits) / 8; // relayLinkPacketsOnce == 1000
		relayApplBytesOnce = 100 * dataLengthBits / 8;
		relayPeriod = SimTime::ZERO;

		requestTimeoutDuration = SimTime(100, SIMTIME_MS);
		interruptTimeoutDuration = SimTime(5, SIMTIME_S);

		relayEvt = new cMessage("relay evt", ContentClientMsgKinds::RELAY_EVT);
		relayEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at downloader
		schemeSwitchEvt = new cMessage("scheme switch evt", ContentClientMsgKinds::SCHEME_SWITCH_EVT);
		dataConsumptionEvt = new cMessage("data consumption evt", ContentClientMsgKinds::DATA_CONSUMPTION_EVT);
		requestTimeoutEvt = new cMessage("request timeout evt", ContentClientMsgKinds::REQUEST_TIMEOUT_EVT);
		interruptTimeoutEvt = new cMessage("interrupt timeout evt", ContentClientMsgKinds::INTERRUPT_TIMEOUT_EVT);
	}
}

void ContentClient::finish()
{
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		delete itDL->second;
	}
	downloaders.clear();

	cancelAndDelete(relayEvt);
	cancelAndDelete(schemeSwitchEvt);
	cancelAndDelete(dataConsumptionEvt);
	cancelAndDelete(requestTimeoutEvt);
	cancelAndDelete(interruptTimeoutEvt);

	BaseWaveApplLayer::finish();
}

void ContentClient::handleSelfMsg(cMessage* msg)
{
	bool isInterruptTimeoutEvt = true;
	switch (msg->getKind())
	{
	case ContentClientMsgKinds::SCHEME_SWITCH_EVT:
	{
		EV << "switched to the scheme in slot " << itSL->slot << ", distributed offset starts at " << itSL->offset->begin << " bytes.\n";
		downloaders[itSL->downloader]->distributedOffset = itSL->offset->begin;
		prevSlotStartTime = simTime();
	}
	case ContentClientMsgKinds::RELAY_EVT:
	{
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
		if (distance >= 200)
		{
			EV << "the communication link between downloader and relay will break soon, notify downloader to report its downloading status to RSU.\n";
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *notifyMsg = dynamic_cast<ContentMessage*>(wsm);

			notifyMsg->setControlCode(ContentMsgCC::LINK_BREAK_DR);
			notifyMsg->setReceiver(itSL->downloader);
			notifyMsg->setDownloader(itSL->downloader);

			sendWSM(notifyMsg);
		}
		if (itSL->amount > relayApplBytesOnce)
		{
			dataMsg->setIsLast(false);
			dataMsg->setBytesNum(relayApplBytesOnce);
			// dataMsg->addBitLength(8*relayLinkBytesOnce);
			dataMsg->addBitLength(headerLength);
			relayPeriod = SimTime(relayLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
			relayPeriod /= 2; // TODO: remove it in the future
			EV << "relay period is " << relayPeriod.dbl() << "s.\n";
			scheduleAt(simTime() + relayPeriod, relayEvt);
			itSL->amount -= relayApplBytesOnce;
			downloaderInfo->distributedOffset += relayApplBytesOnce;
			downloaderInfo->distributedAt = simTime();
			EV << "downloader [" << itSL->downloader << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";
		}
		else
		{
			dataMsg->setIsLast(true);
			int totalLinkBytes = ContentUtils::calcLinkBytes(itSL->amount, headerLength/8, dataLengthBits/8);
			dataMsg->setBytesNum(itSL->amount);
			// dataMsg->addBitLength(8 * totalLinkBytes);
			dataMsg->addBitLength(headerLength);
			relayPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
			relayPeriod /= 2; // TODO: remove it in the future
			EV << "relay period is " << relayPeriod.dbl() << "s.\n";
			downloaderInfo->distributedOffset += itSL->amount;
			EV << "downloader [" << itSL->downloader << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << " bytes.\n";

			int prevSlot = itSL->slot;
			if (++itSL != schemeList.end()) // prepare to schedule self message schemeSwitchEvt
			{
				SimTime dispatchedSlotInterval(slotSpan*(itSL->slot - prevSlot), SIMTIME_MS);
				schemeSwitchInterval = dispatchedSlotInterval - (simTime() - prevSlotStartTime);
				EV << "prepare to switch to the scheme in slot " << itSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
				scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
			}
		}
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		sendDelayedDown(dataMsg, relayPeriod);
		break;
	}
	case ContentClientMsgKinds::DATA_CONSUMPTION_EVT:
	{
		if (downloadingStatus.availableOffset - downloadingStatus.consumedOffset >= downloadingStatus.consumingRate)
			downloadingStatus.consumedOffset += downloadingStatus.consumingRate;
		else
		{
			double correctedTimeoutDuration = interruptTimeoutDuration.dbl() - (downloadingStatus.availableOffset-downloadingStatus.consumedOffset)/downloadingStatus.consumingRate;
			downloadingStatus.consumedOffset = downloadingStatus.availableOffset;
			if (downloadingStatus.consumedOffset < downloadingStatus.totalContentSize) // set the interruption timer, fetch data through cellular approach if timeout
				scheduleAt(simTime() + correctedTimeoutDuration, interruptTimeoutEvt);
			else // consuming process has finished, record statistic
			{
				downloadingStatus.consumingEndAt = simTime();
				double totalInterruptedTime = downloadingStatus.consumingEndAt.dbl() - downloadingStatus.requestAt.dbl()
						- downloadingStatus.totalContentSize/downloadingStatus.consumingRate - CACHE_TIME_BEFORE_PLAY;
				ContentStatisticCollector::globalConsumingTime += downloadingStatus.consumingEndAt.dbl() - downloadingStatus.consumingBeginAt.dbl();
				ContentStatisticCollector::globalInterruptedTime += totalInterruptedTime;
			}
		}
		if (downloadingStatus.consumedOffset < downloadingStatus.totalContentSize) // continue consuming process
			scheduleAt(simTime() + SimTime(slotSpan, SIMTIME_MS), dataConsumptionEvt);
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
			EV << "vedio play interruption is timeout, now fetching data from cellular network.\n";

		CellularMessage *cellularMsg = new CellularMessage("cellular-control");
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
	default:
	{
		BaseWaveApplLayer::handleSelfMsg(msg);
	}
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

				CellularMessage *requestMsg = new CellularMessage("cellular-control");
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
				SimTime transmissionDelay(1000*cellularHeaderLength/(cellularBitsRate/1000), SIMTIME_US);
				sendDirect(requestMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
				cellularDownloading = true;
			}
			else
			{
				EV << "downloading process has finished, response a message to base station.\n";
				CellularMessage *responseMsg = new CellularMessage("cellular-control");
				responseMsg->setControlCode(CellularMsgCC::DOWNLOADING_COMPLETING);
				responseMsg->setDownloader(myAddr);
				responseMsg->addBitLength(cellularHeaderLength);
				SimTime transmissionDelay(1000*cellularHeaderLength/(cellularBitsRate/1000), SIMTIME_US);
				sendDirect(responseMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
				cellularDownloading = false;
			}
		}
		else
		{
			ASSERT( downloadingStatus.availableOffset == downloadingStatus.totalContentSize );

			EV << "downloading process has finished, response a message to base station.\n";
			CellularMessage *responseMsg = new CellularMessage("cellular-control");
			responseMsg->setControlCode(CellularMsgCC::DOWNLOADING_COMPLETING);
			responseMsg->setDownloader(myAddr);
			responseMsg->addBitLength(cellularHeaderLength);
			SimTime transmissionDelay(1000*cellularHeaderLength/(cellularBitsRate/1000), SIMTIME_US);
			sendDirect(responseMsg, SimTime::ZERO, transmissionDelay, baseStationGate);
			cellularDownloading = false;
		}
	}
	delete cellularMsg;
	cellularMsg = nullptr;
}

void ContentClient::onBeacon(BeaconMessage *beaconMsg)
{
	LAddress::L3Type senderAddr = beaconMsg->getSenderAddress(); // alias
	bool isOld = neighbors.find(senderAddr) != neighbors.end();

	BaseWaveApplLayer::onBeacon(beaconMsg);

	if (!isOld)
	{
		if (downloaders.find(senderAddr) != downloaders.end())
			downloaders[senderAddr]->myRole = RELAY; // role changes from carrier to relay because it becomes my neighbor
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
			EV << "it is a response message aims to me, cancel request timeout event.\n";
			cancelEvent(requestTimeoutEvt);
		}
		else
			EV << "it is a response message aims to another downloader, ignore it.\n";
		break;
	}
	case ContentMsgCC::SCHEME_DISTRIBUTION:
	{
		if (downloader != myAddr)
		{
			EV << "it is a scheme distribution message, check if it is the first scheme have received.\n";
			if ( downloaders.find(downloader) != downloaders.end() ) // update old record
			{
				EV << "    downloader [" << downloader << "] is an old downloader, update its info.\n";
				downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
			}
			else // insert new record
			{
				EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
				downloaderInfo = new DownloaderInfo(contentMsg->getContentSize());
				downloaderInfo->myRole = neighbors.find(downloader) != neighbors.end() ? RELAY : STRANGER;
				downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
			}

			SchemeItems &schemeItems = contentMsg->getScheme();
			if (schemeItems.find(myAddr) != schemeItems.end()) // there exists a scheme for me
			{
				schemeList = schemeItems[myAddr];
				itSL = schemeList.begin();
				schemeSwitchInterval = SimTime(slotSpan*(itSL->slot-1), SIMTIME_MS);
				EV << "prepare to switch to the scheme in slot " << itSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
				scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
			}
		}
		else
		{
			EV << "it is a scheme distribution message aims to me, do nothing.\n";
		}
		break;
	}
	case ContentMsgCC::ACKNOWLEDGEMENT:
	{
		if ( downloaders.find(downloader) != downloaders.end() )
		{
			EV << "downloader [" << downloader << "]'s acknowledged offset updates to " << contentMsg->getReceivedOffset() << std::endl;
			downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
			// help to rebroadcast acknowledge message, for downloader might has drived outside of RSU
			if (simTime() - prevSlotStartTime < SimTime(3*slotSpan/2, SIMTIME_MS))
			{
				EV << "I am the relay in the last slot, thus help to rebroadcast acknowledge message.\n";
				ContentMessage *dupAckMsg = new ContentMessage(*contentMsg);
				sendWSM(dupAckMsg);
			}
		}
		break;
	}
	case ContentMsgCC::CARRIER_SELECTION:
	{
		break;
	}
	case ContentMsgCC::DOWNLOADING_COMPLETED:
	{
		EV << "    downloader [" << downloader << "]'s downloading process has completed, erase its info.\n";
		downloaders.erase(downloader);
		break;
	}
	case ContentMsgCC::LINK_BREAK_DIRECT:
	{
		if (contentMsg->getReceivedOffset() == 0)
		{	
			if (downloader == myAddr)
			{
				EV << "I'm the downloader who need to report downloading status to RSU.\n";

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);

				responseMsg->setControlCode(ContentMsgCC::LINK_BREAK_DIRECT);
				responseMsg->setDownloader(downloader);
				responseMsg->setReceivedOffset(downloadingStatus.availableOffset);
				NeighborItems &neighborItems = responseMsg->getNeighborInfo();
				neighborItems.push_back(std::pair<Coord, Coord>(curPosition, curSpeed));
				for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
					neighborItems.push_back(std::pair<Coord, Coord>(itN->second->pos, itN->second->speed));

				sendWSM(responseMsg);
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

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);

				responseMsg->setControlCode(ContentMsgCC::LINK_BREAK_DR);
				responseMsg->setReceiver(contentMsg->getSenderAddress());
				responseMsg->setDownloader(downloader);
				responseMsg->setReceivedOffset(downloadingStatus.availableOffset);
				NeighborItems &neighborItems = responseMsg->getNeighborInfo();
				neighborItems.push_back(std::pair<Coord, Coord>(curPosition, curSpeed));
				for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
					neighborItems.push_back(std::pair<Coord, Coord>(itN->second->pos, itN->second->speed));

				sendWSM(responseMsg);
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

				WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
				ASSERT( wsm != nullptr );
				ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);

				responseMsg->setControlCode(ContentMsgCC::LINK_BREAK_DR);
				responseMsg->setReceiver(contentMsg->getSenderAddress());
				responseMsg->setDownloader(downloader);
				responseMsg->setReceivedOffset(downloadingStatus.availableOffset);
				NeighborItems &neighborItems = responseMsg->getNeighborInfo();
				neighborItems.push_back(std::pair<Coord, Coord>(curPosition, curSpeed));
				for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
					neighborItems.push_back(std::pair<Coord, Coord>(itN->second->pos, itN->second->speed));

				sendWSM(responseMsg);
			}
			else
				EV << "it is a link break notification rebroadcast by relay, ignore it.\n";
		}
		else if (contentMsg->getReceiver() == myAddr)
		{
			ContentMessage *dupResponseMsg = new ContentMessage(*contentMsg);
			if (contentMsg->getReceivedOffset() == 0)
			{
				EV << "I'm the relay whose link with RSU will break soon, notify downloader to report its downloading status to RSU.\n";
				dupResponseMsg->setReceiver(downloader);
			}
			else
			{
				EV << "I'm the relay who rebroadcast this link break notification before, thus transmit it to RSU.\n";
			}
			dupResponseMsg->setSenderAddress(myAddr);
			sendWSM(dupResponseMsg);
		}
		else if (downloaders.find(downloader) != downloaders.end() && downloaders[downloader]->myRole == RELAY)
			EV << "it is a link break notification rebroadcast by another relay, ignore it.\n";
		else
			EV << "I'm not aware of downloader [" << downloader << "] or I'm a stranger of it, ignore it.\n";
		break;
	}
	case ContentMsgCC::RELAY_DISCOVERY:
	{
		break;
	}
	case ContentMsgCC::DISCOVERY_RESPONSE:
	{
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
				EV << "current order of data is partially backward, expected start offset is " << downloadingStatus.availableOffset << " whereas actual start offset is " << dataStartOffset << ", end offset is " << dataEndOffset << ".\n";
				downloadingStatus.itS = downloadingStatus.segments.begin(); // fetch the first segment
				downloadingStatus.itS->second = dataEndOffset;
				downloadingStatus.unionSegment(); // handle with segment union process
				downloadingStatus.availableOffset = downloadingStatus.itS->second;
			}
			else
			{
				EV << "current order of data is fully backward, expected start offset is " << downloadingStatus.availableOffset << " whereas actual start offset is " << dataStartOffset << ", end offset is " << dataEndOffset << ", ignore it.\n";
			}
		}
		if (interruptTimeoutEvt->isScheduled() && downloadingStatus.availableOffset - downloadingStatus.consumedOffset >= downloadingStatus.consumingRate)
		{
			SimTime sinceInterrupt = simTime() - interruptTimeoutEvt->getSendingTime();
			ContentStatisticCollector::globalInterruptedTime += sinceInterrupt.dbl();
			cancelEvent(interruptTimeoutEvt);
		}

		if (dataMsg->getIsLast()) // it is the last data packet in transmission, thus response an acknowledge message
		{
			// check whether the vedio play starting condition is satisified, note this judgement is checked
			// when a transmission process has just finish for offsets synchronous computation purpose
			if (downloadingStatus.availableOffset > CACHE_TIME_BEFORE_PLAY*downloadingStatus.consumingRate && !startPlaying)
			{
				EV << "cached data amount is enough, start playing vedio.\n";
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
				acknowledgeMsg->setDownloader(downloader);
				acknowledgeMsg->setReceivedOffset(downloadingStatus.availableOffset);
				acknowledgeMsg->setConsumedOffset(downloadingStatus.consumedOffset);

				sendWSM(acknowledgeMsg);
			}
		}

		if (downloadingStatus.availableOffset == downloadingStatus.totalContentSize)
		{
			EV << "data receiving process has finished, response an completion message.\n";
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
			if (downloaderInfo->cacheOffset->begin == -1) // only the first received segment meets this condition
			{
				downloaderInfo->cacheOffset->begin = dataMsg->getCurOffset() - dataMsg->getBytesNum();
				EV << "first segment start offset is " << downloaderInfo->cacheOffset->begin << " bytes.\n";
			}
			else if (downloaderInfo->cacheOffset->end < dataMsg->getCurOffset() - dataMsg->getBytesNum())
			{
				EV << "receiving an advanced segment, ";
				downloaderInfo->cacheOffset->next = new struct Segment;
				downloaderInfo->cacheOffset = downloaderInfo->cacheOffset->next;
				downloaderInfo->cacheOffset->begin = dataMsg->getCurOffset() - dataMsg->getBytesNum();
				EV << "its start offset is " << downloaderInfo->cacheOffset->begin << " bytes.\n";
			}
			downloaderInfo->cacheOffset->end = dataMsg->getCurOffset();
			EV << "current segment end offset updates to " << downloaderInfo->cacheOffset->end << " bytes.\n";
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
			if (downloaderInfo->cacheOffset->begin == -1) // only the first received segment meets this condition
			{
				downloaderInfo->cacheOffset->begin = dataMsg->getCurOffset() - dataMsg->getBytesNum();
				EV << "first segment start offset is " << downloaderInfo->cacheOffset->begin << " bytes.\n";
			}
			else if (downloaderInfo->cacheOffset->end < dataMsg->getCurOffset() - dataMsg->getBytesNum())
			{
				EV << "receiving an advanced segment, ";
				downloaderInfo->cacheOffset->next = new struct Segment;
				downloaderInfo->cacheOffset = downloaderInfo->cacheOffset->next;
				downloaderInfo->cacheOffset->begin = dataMsg->getCurOffset() - dataMsg->getBytesNum();
				EV << "its start offset is " << downloaderInfo->cacheOffset->begin << " bytes.\n";
			}
			downloaderInfo->cacheOffset->end = dataMsg->getCurOffset();
			EV << "current segment end offset updates to " << downloaderInfo->cacheOffset->end << " bytes.\n";
		}
		else
			EV << "I'm not the carrier this data message aimed at, ignore it.\n";
		break;
	}
	case STRANGER:
	{
		EV << "I'm a stranger of downloader [" << dataMsg->getDownloader() << "], ignore it.\n";
		break;
	}
	default:
		EV_WARN << "Unknown role " << downloaderInfo->myRole << " I am played, ignore it.\n";
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

	// set necessary content info
	contentMessage->setGUID(guid);
	contentMessage->setControlCode(ContentMsgCC::CONTENT_REQUEST);
	contentMessage->setDownloader(myAddr);
	contentMessage->setContentSize(size);
	contentMessage->setConsumingRate(VideoQuailty::_720P);

	downloadingStatus.reset();
	downloadingStatus.totalContentSize = size;
	downloadingStatus.consumingRate = VideoQuailty::_720P;
	downloadingStatus.requestAt = simTime();
	ASSERT( downloadingStatus.totalContentSize > CACHE_TIME_BEFORE_PLAY*downloadingStatus.consumingRate );

	decorateWSM(contentMessage);
	sendWSM(contentMessage);
	++ContentStatisticCollector::globalContentRequests;

	scheduleAt(simTime() + requestTimeoutDuration, requestTimeoutEvt);
	EV << "set request timeout timer, its duration is " << requestTimeoutDuration.dbl() << "s.\n";
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
	if (itS == segments.end() || itS->first > endOffset)
	{
		segments.insert(itS, std::pair<int, int>(startOffset, endOffset));
		++segmentNum;
	}
	else if (itS->first <= endOffset && itS->first > startOffset)
		itS->first = startOffset;
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

