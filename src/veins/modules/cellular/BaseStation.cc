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

#include "veins/modules/cellular/BaseStation.h"

Define_Module(BaseStation);

void BaseStation::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		EV << "BaseStation::initialize() called.\n";

		wirelessIn = findGate("wirelessIn");
		wiredIn = findGate("wiredIn");
		wiredOut = findGate("wiredOut");
		wiredHeaderLength = par("wiredHeaderLength").longValue();
		wirelessHeaderLength = par("wirelessHeaderLength").longValue();
		wirelessDataLength = par("wirelessDataLength").longValue();
		wirelessBitsRate = par("wirelessBitsRate").longValue() / 25; // user number is 25, thus rate is 500KB/s

		fetchApplBytesOnce = 500 * 1472; // fetchPacketsOnce == 500
		distributeLinkBytesOnce = 20 * (wirelessHeaderLength + wirelessDataLength) / 8; // distributeLinkPacketsOnce == 20
		distributeApplBytesOnce = 20 * wirelessDataLength / 8;
		distributePeriod = SimTime(8 * distributeLinkBytesOnce * 10 * 25, SIMTIME_NS); // 100Mbps TD-LTE downlink channel, 10 is obtained by 1e9 / 100 / 1e6
		EV << "distribute period is " << distributePeriod.dbl() << "s.\n";
		wiredTxDuration = SimTime((wiredHeaderLength + 160) * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6

		fetchRequestEvt = new cMessage("fetch request evt", SelfMsgKinds::FETCH_REQUEST_EVT);
		fetchRequestEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at content server
		rootModule = cSimulation::getActiveSimulation()->getSystemModule();
	}
	else
	{
		EV << "BaseStation initialized.\n";
	}
}

void BaseStation::finish()
{
	EV << "BaseStation::finish() called.\n";

	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		cancelAndDelete(itDL->second->distributeEvt);
		delete itDL->second;
	}
	downloaders.clear();
	cancelAndDelete(fetchRequestEvt);

	cComponent::finish();
}

void BaseStation::handleMessage(cMessage *msg)
{
	if (msg->isSelfMessage())
		handleSelfMsg(msg);
	else if (msg->getArrivalGateId() == wirelessIn)
	{
		CellularMessage *cellularMsg = dynamic_cast<CellularMessage*>(msg);
		handleWirelessIncomingMsg(cellularMsg);
	}
	else if (msg->getArrivalGateId() == wiredIn)
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
		handleWiredIncomingMsg(wiredMsg);
	}
	else if (msg->getArrivalGateId() == -1)
		error("No self message and no gateID?? Check configuration.");
	else
		error("Unknown gateID?? Check configuration or override handleMessage().");
}

void BaseStation::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case SelfMsgKinds::DISTRIBUTE_EVT:
	{
		for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		{
			DownloaderInfo *downloaderInfo = itDL->second; // alias
			if (downloaderInfo->distributedAt == simTime()) // self message aims to this downloader
			{
				int undistributedAmount = downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset; // alias
				if (downloaderInfo->transmissionActive && !downloaderInfo->fetchingActive && undistributedAmount < 3*distributeApplBytesOnce) // need to fetch more data
				{
					downloaderInfo->curFetchEndOffset += fetchApplBytesOnce;
					if (downloaderInfo->curFetchEndOffset > downloaderInfo->requiredEndOffset)
						downloaderInfo->curFetchEndOffset = downloaderInfo->requiredEndOffset;
					if (downloaderInfo->curFetchEndOffset > downloaderInfo->cacheEndOffset)
					{
						EV << "undistributed data is not adequate, current fetching end offset is " << downloaderInfo->curFetchEndOffset << ".\n";
						_sendFetchingRequest(itDL->first, downloaderInfo->cacheEndOffset);
					}
				}

				downloaderInfo->distributedAt = simTime() + distributePeriod;

				if (undistributedAmount > distributeApplBytesOnce) // has enough data to filling a cellular normal data packet
				{
					CellularMessage *cellularMsg = new CellularMessage("data", CellularMsgCC::DATA_PACKET_NORMAL);
					if (downloaderInfo->transmissionActive)
						cellularMsg->setControlCode(CellularMsgCC::DATA_PACKET_NORMAL);
					else
						cellularMsg->setControlCode(CellularMsgCC::DATA_PACKET_LAST);
					cellularMsg->setDownloader(itDL->first);
					cellularMsg->addBitLength(8 * distributeLinkBytesOnce);
					ContentStatisticCollector::globalCellularFlux += distributeApplBytesOnce;
					downloaderInfo->distributedOffset += distributeApplBytesOnce;
					if (downloaderInfo->transmissionActive)
						scheduleAt(downloaderInfo->distributedAt, downloaderInfo->distributeEvt);
					EV << "downloader [" << itDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << ".\n";
					cellularMsg->setCurOffset(downloaderInfo->distributedOffset);
					sendDirect(cellularMsg, SimTime::ZERO, distributePeriod, downloaderInfo->correspondingGate);
				}
				else if (downloaderInfo->cacheEndOffset > downloaderInfo->distributedOffset) // has data but not enough to filling a cellular normal data packet
				{
					 // data fetch process has finished or cellular connection is closed by downloader, thus send the last half-filled cellular data packet
					if (downloaderInfo->cacheEndOffset == downloaderInfo->requiredEndOffset || !downloaderInfo->transmissionActive)
					{
						if (downloaderInfo->transmissionActive)
							EV << "send the last half-filled data packet because data fetching process has finished.\n";
						else
							EV << "send the last half-filled data packet because cellular connection is closed by downloader.\n";

						CellularMessage *cellularMsg = new CellularMessage("data", CellularMsgCC::DATA_PACKET_LAST);
						cellularMsg->setControlCode(CellularMsgCC::DATA_PACKET_LAST);
						cellularMsg->setDownloader(itDL->first);
						int lastPktAmount = downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset; // alias
						int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, wirelessHeaderLength/8, wirelessDataLength/8);
						cellularMsg->addBitLength(8 * totalLinkBytes);
						ContentStatisticCollector::globalCellularFlux += lastPktAmount;
						downloaderInfo->distributedOffset = downloaderInfo->cacheEndOffset;
						cellularMsg->setCurOffset(downloaderInfo->distributedOffset);
						EV << "downloader [" << itDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << ".\n";
						SimTime transmissionDelay((totalLinkBytes*1000)/(wirelessBitsRate/1000) * 8, SIMTIME_US);
						sendDirect(cellularMsg, SimTime::ZERO, transmissionDelay, downloaderInfo->correspondingGate);
					}
					else // data fetch process has not finished, thus don't send half-filled cellular data packet
					{
						EV << "don't send half-filled data packet because data fetch process has not finished, wait a moment.\n";
						scheduleAt(downloaderInfo->distributedAt, downloaderInfo->distributeEvt);
					}
				}
				else
				{
					EV << "there is no data to send currently, check later.\n";
					scheduleAt(downloaderInfo->distributedAt, downloaderInfo->distributeEvt);
				}
				break;
			}
		}
		break;
	}
	case SelfMsgKinds::FETCH_REQUEST_EVT:
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(fetchMsgQueue.pop());
		sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
		EV << "send downloader [" << wiredMsg->getDownloader() << "]'s fetching request to content server.\n";
		if (fetchMsgQueue.getLength() > 0)
			scheduleAt(simTime() + wiredTxDuration, fetchRequestEvt);
		break;
	}
	default:
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << std::endl;
	}
}

void BaseStation::handleWirelessIncomingMsg(CellularMessage *cellularMsg)
{
	EV << "receiving a message from vehicle " << cellularMsg->getDownloader() << ".\n";

	ASSERT( cellularMsg->getControlCode() == CellularMsgCC::TRANSMISSION_STARTING || cellularMsg->getControlCode() == CellularMsgCC::TRANSMISSION_ENDING || cellularMsg->getControlCode() == CellularMsgCC::DOWNLOADING_COMPLETING );

	LAddress::L3Type downloader = cellularMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	if (cellularMsg->getControlCode() == CellularMsgCC::TRANSMISSION_STARTING)
	{
		// handle with downloader's information
		if (downloaders.find(downloader) != downloaders.end()) // update old record
		{
			EV << "    downloader [" << downloader << "] is an old downloader, update its info.\n";
			downloaderInfo = downloaders[downloader];
			downloaderInfo->distributedOffset = cellularMsg->getStartOffset();
			downloaderInfo->requiredEndOffset = cellularMsg->getEndOffset();
		}
		else // insert new record
		{
			EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
			downloaderInfo = new DownloaderInfo(cellularMsg->getContentSize(), cellularMsg->getStartOffset(), cellularMsg->getEndOffset());
			downloaderInfo->correspondingGate = rootModule->getSubmodule("node", downloader)->gate("veinscellularIn");
			downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
		}
		EV << "    its required start offset: " << cellularMsg->getStartOffset() << ", required end offset: " << cellularMsg->getEndOffset() << ".\n";
		downloaderInfo->curFetchEndOffset = downloaderInfo->distributedOffset + fetchApplBytesOnce;
		if (downloaderInfo->curFetchEndOffset > downloaderInfo->requiredEndOffset)
			downloaderInfo->curFetchEndOffset = downloaderInfo->requiredEndOffset;

		if (downloaderInfo->curFetchEndOffset > downloaderInfo->cacheEndOffset)
		{
			int curFetchStartOffset = std::max(cellularMsg->getStartOffset(), downloaderInfo->cacheEndOffset);
			EV << "current fetching start offset: " << curFetchStartOffset << ", end offset: " << downloaderInfo->curFetchEndOffset << ".\n";
			_sendFetchingRequest(downloader, curFetchStartOffset);
		}
	}
	else if (cellularMsg->getControlCode() == CellularMsgCC::TRANSMISSION_ENDING)
	{
		EV << "it is a transmission ending message.\n";
		// notify content server to stop data transmission
		WiredMessage *wiredMsg = new WiredMessage("stop", WiredMsgCC::END_TRANSMISSION);
		wiredMsg->setControlCode(WiredMsgCC::END_TRANSMISSION);
		wiredMsg->setDownloader(downloader);
		wiredMsg->addBitLength(wiredHeaderLength);
		sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
		// set transmission inactive
		downloaderInfo = downloaders[downloader];
		downloaderInfo->transmissionActive = false;
		downloaderInfo->closingWhileFetching = downloaderInfo->fetchingActive;
	}
	else // (cellularMsg->getControlCode() == CellularMsgCC::DOWNLOADING_COMPLETING)
	{
		EV << "it is a downloading completing message.\n";
		// notify content server to release corresponding resources
		WiredMessage *wiredMsg = new WiredMessage("completion", WiredMsgCC::COMPLETE_DOWNLOADING);
		wiredMsg->setControlCode(WiredMsgCC::COMPLETE_DOWNLOADING);
		wiredMsg->setDownloader(downloader);
		wiredMsg->addBitLength(wiredHeaderLength);
		sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
		// release ourself resources corresponding to this downloader
		if ((itDL = downloaders.find(downloader)) != downloaders.end())
		{
			cancelAndDelete(itDL->second->distributeEvt);
			delete itDL->second;
			downloaders.erase(itDL);
		}
	}
	delete cellularMsg;
	cellularMsg = nullptr;
}

void BaseStation::handleWiredIncomingMsg(WiredMessage *wiredMsg)
{
	ASSERT( wiredMsg->getControlCode() == WiredMsgCC::NORMAL_DATA_PACKET || wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET );

	LAddress::L3Type downloader = wiredMsg->getDownloader();   // alias
	DownloaderInfo *downloaderInfo = downloaders[downloader];  // this downloader's record certainly exists
	downloaderInfo->cacheEndOffset = wiredMsg->getCurOffset(); // the order of data is assured due to wired link
	ContentStatisticCollector::globalStorageAmount += wiredMsg->getBytesNum();
	EV << "downloader [" << downloader << "]'s cache end offset updates to " << downloaderInfo->cacheEndOffset << " bytes.\n";

	if (!downloaderInfo->distributeEvt->isScheduled())
	{
		downloaderInfo->transmissionActive = !downloaderInfo->closingWhileFetching;
		downloaderInfo->distributedAt = simTime();
		scheduleAt(downloaderInfo->distributedAt, downloaderInfo->distributeEvt);
	}

	if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET) // it is the last data packet, indicating data fetching progress has finished
	{
		EV << "its data fetching progress has finished.\n";
		downloaderInfo->fetchingActive = false;
		downloaderInfo->closingWhileFetching = false;
	}

	delete wiredMsg;
	wiredMsg = nullptr;
}

void BaseStation::_sendFetchingRequest(const LAddress::L3Type downloader, const int curFetchStartOffset)
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

/////////////////////////    internal class implementations    /////////////////////////
BaseStation::DownloaderInfo::DownloaderInfo(int t, int s, int r) : transmissionActive(false), fetchingActive(false), closingWhileFetching(false), totalContentSize(t),
		cacheStartOffset(s), cacheEndOffset(0), distributedOffset(s), requiredEndOffset(r), curFetchEndOffset(0), distributedAt(), correspondingGate(nullptr)
{
	distributeEvt = new cMessage("distribute evt", SelfMsgKinds::DISTRIBUTE_EVT);
	distributeEvt->setSchedulingPriority(1); // ensure when to send the next packet, the previous packet has arrived at vehicle
}
