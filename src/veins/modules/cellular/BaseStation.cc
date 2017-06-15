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

		distributeLinkBytesOnce = 200 * (wirelessHeaderLength + wirelessDataLength) / 8; // distributeLinkPacketsOnce == 200
		distributeApplBytesOnce = 200 * wirelessDataLength / 8;
		distributePeriod = SimTime(8 * distributeLinkBytesOnce * 10 * 25, SIMTIME_NS); // 100Mbps TD-LTE downlink channel, 10 is obtained by 1e9 / 100 / 1e6

		distributeEvt = new cMessage("distribute evt", SelfMsgKinds::DISTRIBUTE_EVT);
		distributeEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at vehicle
		rootModule = omnetpp::cSimulation::getActiveSimulation()->getSystemModule();
	}
	else
	{
		EV << "BaseStation initialized.\n";
	}
}

void BaseStation::finish()
{
	EV << "BaseStation::finish() called.\n";

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
            if (downloaderInfo->distributedAt == simTime() - distributePeriod) // self message aims to this downloader
            {
                if (downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset > distributeApplBytesOnce) // has enough data to filling a cellular normal data packet
                {
                    CellularMessage *cellularMsg = new CellularMessage("data");
                    cellularMsg->setControlCode(CellularMsgCC::DATA_PACKET_NORMAL);
                    cellularMsg->setDownloader(itDL->first);
                    cellularMsg->addBitLength(8 * distributeLinkBytesOnce);
                    scheduleAt(simTime() + distributePeriod, distributeEvt);
                    downloaderInfo->distributedOffset += distributeApplBytesOnce;
                    downloaderInfo->distributedAt = simTime();
                    EV << "downloader [" << itDL->first << "]'s distributed offset updated to " << downloaderInfo->distributedOffset << std::endl;
                    cellularMsg->setCurOffset(downloaderInfo->distributedOffset);
                    sendDirect(cellularMsg, SimTime::ZERO, distributePeriod, downloaderInfo->correspondingGate);
                }
                else if (downloaderInfo->cacheEndOffset > downloaderInfo->distributedOffset) // has data but not enough to filling a cellular normal data packet
                {
                    if (downloaderInfo->cacheEndOffset == downloaderInfo->requiredEndOffset) // data fetch process has finished, thus send the last half-filled cellular data packet
                    {
                        EV << "send the last half-filled data packet because data fetch process has finished.\n";
                        CellularMessage *cellularMsg = new CellularMessage("data");
                        cellularMsg->setControlCode(CellularMsgCC::DATA_PACKET_LAST);
                        cellularMsg->setDownloader(itDL->first);
                        int completePacketNum = (downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset) / (wirelessDataLength/8);
                        int lastPacketLength = 0;
                        if ((lastPacketLength = (downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset) % (wirelessDataLength/8)) != 0)
                            lastPacketLength += wirelessHeaderLength/8; // plus header length
                        int totalLinkBytes = (wirelessHeaderLength+wirelessDataLength)/8 * completePacketNum + lastPacketLength;
                        cellularMsg->addBitLength(8 * totalLinkBytes);
                        downloaderInfo->distributedOffset = downloaderInfo->requiredEndOffset;
                        cellularMsg->setCurOffset(downloaderInfo->distributedOffset);
                        EV << "downloader [" << itDL->first << "]'s distributed offset updated to " << downloaderInfo->distributedOffset << std::endl;
                        SimTime transmissionDelay((totalLinkBytes*1000)/(wirelessBitsRate/1000) * 8, SIMTIME_US);
                        sendDirect(cellularMsg, SimTime::ZERO, transmissionDelay, downloaderInfo->correspondingGate);
                    }
                    else // data fetch process has not finished, thus don't send half-filled cellular data packet
                    {
                        EV << "don't send half-filled data packet because data fetch process has not finished, wait a moment.\n";
                        scheduleAt(simTime() + distributePeriod, distributeEvt);
                    }
                }
                else
                {
                    EV << "there is no data to send currently, check later.\n";
                    scheduleAt(simTime() + distributePeriod, distributeEvt);
                }
                break;
            }
        }
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
	        downloaderInfo->requiredEndOffset = cellularMsg->getEndOffset();
	    }
	    else // insert new record
	    {
	        EV << "    downloader [" << downloader << "] is a new downloader, insert its info.\n";
	        EV << "    its start offset: " << cellularMsg->getStartOffset() << ", end offset: " << cellularMsg->getEndOffset() << std::endl;
	        downloaderInfo = new DownloaderInfo(cellularMsg->getContentSize(), cellularMsg->getStartOffset(), cellularMsg->getEndOffset());
	        downloaderInfo->correspondingGate = rootModule->getSubmodule("node", downloader)->gate("veinscellularIn");
	        downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
	    }

	    // deliver request to content server
        WiredMessage *wiredMsg = new WiredMessage("wired");
        wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
        wiredMsg->setDownloader(downloader);
        wiredMsg->setContentSize(cellularMsg->getContentSize());
        wiredMsg->setStartOffset(cellularMsg->getStartOffset());
        wiredMsg->setEndOffset(cellularMsg->getEndOffset());
        wiredMsg->addBitLength(wiredHeaderLength);
        sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
	}
	else if (cellularMsg->getControlCode() == CellularMsgCC::TRANSMISSION_ENDING)
	{
	    EV << "it is a transmission ending message.\n";
	    // notify content server to stop data transmission
        WiredMessage *wiredMsg = new WiredMessage("wired");
        wiredMsg->setControlCode(WiredMsgCC::END_TRANSMISSION);
        wiredMsg->setDownloader(downloader);
        wiredMsg->addBitLength(wiredHeaderLength);
        sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
	}
	else // (cellularMsg->getControlCode() == CellularMsgCC::DOWNLOADING_COMPLETING)
	{
	    EV << "it is a downloading completing message.\n";
	    // notify content server to release corresponding resources
        WiredMessage *wiredMsg = new WiredMessage("wired");
        wiredMsg->setControlCode(WiredMsgCC::COMPLETE_DOWNLOADING);
        wiredMsg->setDownloader(downloader);
        wiredMsg->addBitLength(wiredHeaderLength);
        sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
        // release ourself resources corresponding to this downloader
        downloaders.erase(downloader);
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
    EV << "downloader [" << downloader << "]'s cache end offset updates to " << downloaderInfo->cacheEndOffset << " bytes.\n";

    if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET) // it is the last data packet, indicating date fetch progress has finished
    {
        EV << "its content fetch progress has finished.\n";
        ASSERT( downloaderInfo->cacheEndOffset == downloaderInfo->requiredEndOffset );
    }

    if (downloaderInfo->cacheEndOffset - downloaderInfo->distributedOffset > distributeApplBytesOnce && !distributeEvt->isScheduled()) // has enough data to filling a cellular normal data packet
    {
        CellularMessage *cellularMsg = new CellularMessage("data");
        cellularMsg->setControlCode(CellularMsgCC::DATA_PACKET_NORMAL);
        cellularMsg->setDownloader(downloader);
        cellularMsg->addBitLength(8 * distributeLinkBytesOnce);
        scheduleAt(simTime() + distributePeriod, distributeEvt);
        downloaderInfo->distributedOffset += distributeApplBytesOnce;
        downloaderInfo->distributedAt = simTime();
        cellularMsg->setCurOffset(downloaderInfo->distributedOffset);
        EV << "downloader [" << downloader << "]'s distributed offset updated to " << downloaderInfo->distributedOffset << std::endl;
        sendDirect(cellularMsg, SimTime::ZERO, distributePeriod, downloaderInfo->correspondingGate);
    }
	delete wiredMsg;
	wiredMsg = nullptr;
}
