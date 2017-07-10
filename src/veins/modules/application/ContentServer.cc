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

#include "veins/modules/application/ContentServer.h"

Define_Module(ContentServer);

void ContentServer::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		rsuNum = par("rsuNum").longValue();
		rsuIn = new int[rsuNum];
		rsuOut = new int[rsuNum];
		for (int i = 0; i < rsuNum; ++i)
		{
			rsuIn[i] = findGate("rsuIn", i);
			rsuOut[i] = findGate("rsuOut", i);
			activeDownloaderQs.push_back(std::queue<LAddress::L3Type, std::list<LAddress::L3Type> >());
		}
		cellularIn = findGate("cellularIn");
		cellularOut = findGate("cellularOut");
		headerLength = par("headerLength").longValue();

		distributeRSULinkBytesOnce = 500 * 1500; // MTU is 1500 bytes, distributeRSULinkPacketsOnce == 500
		distributeBSLinkBytesOnce = 1000 * 1500; // MTU is 1500 bytes, distributeBSLinkPacketsOnce == 1000
		distributeRSUApplBytesOnce = 500 * 1472; // UDP/IP header length is 28 bytes
		distributeBSApplBytesOnce = 1000 * 1472; // UDP/IP header length is 28 bytes
		distributeRSUPeriod = SimTime(8 * distributeRSULinkBytesOnce * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6
		distributeBSPeriod = SimTime(8 * distributeBSLinkBytesOnce * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6
		EV << "distribute period for RSU is " << distributeRSUPeriod.dbl() << "s, for BS is " << distributeBSPeriod.dbl() << "s.\n";

		distributeRSUEvt = new cMessage("distribute rsu evt", SelfMsgKinds::DISTRIBUTE_RSU_EVT);
		distributeBSEvt = new cMessage("distribute bs evt", SelfMsgKinds::DISTRIBUTE_BS_EVT);
		distributeRSUEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at RSU
		distributeBSEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at BS
	}
	else
	{
		EV << "ContentServer initialized.\n";
	}
}

void ContentServer::finish()
{
	EV << "ContentServer::finish() called.\n";

	cancelAndDelete(distributeRSUEvt);
	cancelAndDelete(distributeBSEvt);

	activeDownloaderQs.clear();
	delete []rsuIn;
	delete []rsuOut;

	cComponent::finish();
}

void ContentServer::handleMessage(cMessage *msg)
{
	if (msg->isSelfMessage())
		handleSelfMsg(msg);
	else if (msg->getArrivalGateId() >= rsuIn[0] && msg->getArrivalGateId() <= rsuIn[rsuNum-1])
	{
		WiredMessage *rsuMsg = dynamic_cast<WiredMessage*>(msg);
		for (int rsuIdx = 0; rsuIdx < rsuNum; ++rsuIdx)
		{
			if (msg->getArrivalGateId() == rsuIn[rsuIdx])
			{
				handleRSUIncomingMsg(rsuMsg, rsuIdx);
				delete rsuMsg;
				rsuMsg = nullptr;
				break;
			}
		}
	}
	else if (msg->getArrivalGateId() == cellularIn)
	{
		WiredMessage *lteMsg = dynamic_cast<WiredMessage*>(msg);
		handleLTEIncomingMsg(lteMsg);
		delete lteMsg;
		lteMsg = nullptr;
	}
	else if (msg->getArrivalGateId() == -1)
		error("No self message and no gateID! Check configuration.");
	else
		error("Unknown gateID! Check configuration or override handleMessage().");
}

void ContentServer::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case SelfMsgKinds::DISTRIBUTE_RSU_EVT:
	{
		for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		{
			DownloaderInfo *downloaderInfo = itDL->second; // alias
			if (downloaderInfo->distributedAt == simTime()) // self message aims to this downloader
			{
				WiredMessage *dataMsg = new WiredMessage("data", WiredMsgCC::NORMAL_DATA_PACKET);
				dataMsg->setDownloader(itDL->first);
				if (downloaderInfo->requiredEndOffset - downloaderInfo->distributedOffset > distributeRSUApplBytesOnce)
				{
					dataMsg->setControlCode(WiredMsgCC::NORMAL_DATA_PACKET);
					dataMsg->setBytesNum(distributeRSUApplBytesOnce);
					dataMsg->addBitLength(8*distributeRSULinkBytesOnce);
					downloaderInfo->distributedOffset += distributeRSUApplBytesOnce;
					downloaderInfo->distributedAt = simTime() + distributeRSUPeriod;
					scheduleAt(downloaderInfo->distributedAt, distributeRSUEvt);
				}
				else
				{
					dataMsg->setControlCode(WiredMsgCC::LAST_DATA_PACKET);
					int lastPktAmount = downloaderInfo->requiredEndOffset - downloaderInfo->distributedOffset; // alias
					int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, 28, 1472);
					dataMsg->setBytesNum(lastPktAmount);
					dataMsg->addBitLength(8*totalLinkBytes);
					downloaderInfo->distributedOffset = downloaderInfo->requiredEndOffset;
					std::queue<LAddress::L3Type, std::list<LAddress::L3Type> > &activeQ = activeDownloaderQs[downloaderInfo->rsuIndex];
					activeQ.pop();
					if (!activeQ.empty())
					{
						SimTime transmissionDelay(8*totalLinkBytes * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6
						downloaders[activeQ.front()]->distributedAt = simTime() + transmissionDelay;
						scheduleAt(simTime() + transmissionDelay, distributeRSUEvt);
					}
				}
				EV << "downloader [" << itDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << std::endl;
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayed(dataMsg, SimTime::ZERO, "rsuOut", downloaderInfo->rsuIndex);
				break;
			}
		}
		break;
	}
	case SelfMsgKinds::DISTRIBUTE_BS_EVT:
	{
		for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
		{
			DownloaderInfo *downloaderInfo = itDL->second; // alias
			if (downloaderInfo->distributedAt == simTime()) // self message aims to this downloader
			{
				WiredMessage *dataMsg = new WiredMessage("data", WiredMsgCC::NORMAL_DATA_PACKET);
				dataMsg->setDownloader(itDL->first);
				if (downloaderInfo->requiredEndOffset - downloaderInfo->distributedOffset > distributeBSApplBytesOnce)
				{
					dataMsg->setControlCode(WiredMsgCC::NORMAL_DATA_PACKET);
					dataMsg->setBytesNum(distributeBSApplBytesOnce);
					dataMsg->addBitLength(8*distributeBSLinkBytesOnce);
					downloaderInfo->distributedOffset += distributeBSApplBytesOnce;
					downloaderInfo->distributedAt = simTime() + distributeBSPeriod;
					scheduleAt(downloaderInfo->distributedAt, distributeBSEvt);
				}
				else
				{
					dataMsg->setControlCode(WiredMsgCC::LAST_DATA_PACKET);
					int lastPktAmount = downloaderInfo->requiredEndOffset - downloaderInfo->distributedOffset; // alias
					int totalLinkBytes = ContentUtils::calcLinkBytes(lastPktAmount, 28, 1472);
					dataMsg->setBytesNum(lastPktAmount);
					dataMsg->addBitLength(8*totalLinkBytes);
					downloaderInfo->distributedOffset = downloaderInfo->requiredEndOffset;
				}
				EV << "downloader [" << itDL->first << "]'s distributed offset updates to " << downloaderInfo->distributedOffset << std::endl;
				dataMsg->setCurOffset(downloaderInfo->distributedOffset);
				sendDelayed(dataMsg, SimTime::ZERO, "cellularOut");
				break;
			}
		}
		break;
	}
	default:
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << std::endl;
	}
}

void ContentServer::handleRSUIncomingMsg(WiredMessage *rsuMsg, int rsuIdx)
{
	EV << "receiving a message from rsu[" << rsuIdx << "].\n";

	ASSERT( rsuMsg->getControlCode() == WiredMsgCC::START_TRANSMISSION || rsuMsg->getControlCode() == WiredMsgCC::END_TRANSMISSION || rsuMsg->getControlCode() == WiredMsgCC::COMPLETE_DOWNLOADING );

	LAddress::L3Type downloader = rsuMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	if (rsuMsg->getControlCode() == WiredMsgCC::START_TRANSMISSION)
	{
		if ( downloaders.find(downloader) != downloaders.end() ) // update old record
		{
			EV << "    downloader [" << downloader << "] is an old downloader, update its info.\n";
			downloaderInfo = downloaders[downloader];
			downloaderInfo->distributedOffset = rsuMsg->getStartOffset();
			downloaderInfo->requiredEndOffset = rsuMsg->getEndOffset();
		}
		else // insert new record
		{
			EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
			downloaderInfo = new DownloaderInfo(rsuMsg->getContentSize());
			ASSERT( rsuMsg->getStartOffset() == 0 );
			downloaderInfo->requiredEndOffset = rsuMsg->getEndOffset();
			downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
		}
		EV << "downloader [" << downloader << "]'s required end offset is " << downloaderInfo->requiredEndOffset << std::endl;

		downloaderInfo->rsuIndex = rsuIdx;
		if (activeDownloaderQs[rsuIdx].empty())
		{
			EV << "active downloader queue is empty, start distribution now.\n";
			downloaderInfo->distributedAt = simTime() + SimTime((headerLength + 160)*10, SIMTIME_NS);
			scheduleAt(downloaderInfo->distributedAt, distributeRSUEvt);
		}
		else
			EV << "active downloader queue is not empty, wait other downloaders' distribution to finish.\n";
		activeDownloaderQs[rsuIdx].push(downloader);
	}
	else if (rsuMsg->getControlCode() == WiredMsgCC::END_TRANSMISSION)
	{
		EV << "it is a end transmission message.\n";
		// if (distributeRSUEvt->isScheduled())
		//	cancelEvent(distributeRSUEvt);
	}
	else // (rsuMsg->getControlCode() == WiredMsgCC::COMPLETE_DOWNLOADING)
	{
		EV << "it is a complete downloading message.\n";
		// release resources corresponding to this downloader
		if ((itDL = downloaders.find(downloader)) != downloaders.end())
		{
			delete itDL->second;
			downloaders.erase(itDL);
		}
	}
}

void ContentServer::handleLTEIncomingMsg(WiredMessage *lteMsg)
{
	EV << "receiving a message from cellular base station.\n";

	ASSERT( lteMsg->getControlCode() == WiredMsgCC::START_TRANSMISSION || lteMsg->getControlCode() == WiredMsgCC::END_TRANSMISSION || lteMsg->getControlCode() == WiredMsgCC::COMPLETE_DOWNLOADING );

	LAddress::L3Type downloader = lteMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	if (lteMsg->getControlCode() == WiredMsgCC::START_TRANSMISSION)
	{
		if ( downloaders.find(downloader) != downloaders.end() ) // update old record
		{
			EV << "    downloader [" << downloader << "] is an old downloader, update its info.\n";
			downloaderInfo = downloaders[downloader];
			downloaderInfo->distributedOffset = lteMsg->getStartOffset();
			downloaderInfo->requiredEndOffset = lteMsg->getEndOffset();
		}
		else // insert new record
		{
			EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
			downloaderInfo = new DownloaderInfo(lteMsg->getContentSize());
			ASSERT( lteMsg->getStartOffset() == 0 );
			downloaderInfo->requiredEndOffset = lteMsg->getEndOffset();
			downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
		}
		EV << "downloader [" << downloader << "]'s required end offset is " << downloaderInfo->requiredEndOffset << std::endl;

		downloaderInfo->distributedAt = simTime();
		scheduleAt(downloaderInfo->distributedAt, distributeBSEvt);
	}
	else if (lteMsg->getControlCode() == WiredMsgCC::END_TRANSMISSION)
	{
		EV << "it is a end transmission message.\n";
		if (distributeBSEvt->isScheduled())
			cancelEvent(distributeBSEvt);
	}
	else // (lteMsg->getControlCode() == WiredMsgCC::COMPLETE_DOWNLOADING)
	{
		EV << "it is a complete downloading message.\n";
		// release resources corresponding to this downloader
		if ((itDL = downloaders.find(downloader)) != downloaders.end())
		{
			delete itDL->second;
			downloaders.erase(itDL);
		}
	}
}
