//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
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

        slotSpan = par("slotSpan").longValue();
        relayLinkBytesOnce = 1000 * (headerLength + dataLengthBits) / 8; // relayLinkPacketsOnce == 1000
        relayApplBytesOnce = 1000 * headerLength / 8;
        relayPeriod = SimTime::ZERO;

        requestTimeoutDuration = SimTime(100, SIMTIME_MS);
        interruptTimeoutDuration = SimTime(5, SIMTIME_S);

        relayEvt = new cMessage("relay evt", ContentClientMsgKinds::RELAY_EVT);
        relayEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at downloader
        schemeSwitchEvt = new cMessage("scheme switch evt", ContentClientMsgKinds::SCHEME_SWITCH_EVT);
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
    selfInfo = nullptr;

    if (relayEvt->isScheduled())
        cancelAndDelete(relayEvt);
    else
        delete relayEvt;
    if (schemeSwitchEvt->isScheduled())
        cancelAndDelete(schemeSwitchEvt);
    else
        delete schemeSwitchEvt;
    if (requestTimeoutEvt->isScheduled())
        cancelAndDelete(requestTimeoutEvt);
    else
        delete requestTimeoutEvt;
    if (interruptTimeoutEvt->isScheduled())
        cancelAndDelete(interruptTimeoutEvt);
    else
        delete interruptTimeoutEvt;

    BaseWaveApplLayer::finish();
}

void ContentClient::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind())
    {
    case ContentClientMsgKinds::RELAY_EVT:
    {
        break;
    }
    case ContentClientMsgKinds::SCHEME_SWITCH_EVT:
    {
        break;
    }
    case ContentClientMsgKinds::REQUEST_TIMEOUT_EVT:
    {
        break;
    }
    case ContentClientMsgKinds::INTERRUPT_TIMEOUT_EVT:
    {
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
    EV << "ContentClient::handleCellularMsg called.\n";
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
    EV << logName() << ": " << "onContent!\n";

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
        if ( downloaders.find(downloader) != downloaders.end() ) // update old record
        {
            EV << "    downloader: node[" << downloader << "] is an old downloader, update its info.\n";
            downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
        }
        else // insert new record
        {
            EV << "    downloader: node[" << downloader << "] is a new downloader, insert its info.\n";
            downloaderInfo = new DownloaderInfo(contentMsg->getContentSize());
            downloaderInfo->myRole = neighbors.find(downloader) != neighbors.end() ? RELAY : CARRIER;
            downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));
        }
        break;
    }
    case ContentMsgCC::ACKNOWLEDGEMENT:
    {
        if ( downloaders.find(downloader) != downloaders.end() )
        {
            EV << "downloader [" << downloader << "]'s acknowledged offset updates to " << contentMsg->getReceivedOffset() << std::endl;
            downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
        }
        break;
    }
    case ContentMsgCC::CARRIER_SELECTION:
    {
        break;
    }
    case ContentMsgCC::LINK_BREAK_DIRECT:
    {
        break;
    }
    case ContentMsgCC::LINK_BREAK_DR:
    {
        break;
    }
    case ContentMsgCC::LINK_BREAK_RR:
    {
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
    EV << logName() << ": " << "onData!\n";

    if (downloaders.find(dataMsg->getDownloader()) == downloaders.end())
    {
        EV << "I'm not aware of downloader [" << dataMsg->getDownloader() << "] who is aimed by this data message, ignore it.\n";
        return;
    }

    DownloaderInfo *downloaderInfo = downloaders[dataMsg->getDownloader()];
    switch (downloaderInfo->myRole)
    {
    case RELAY:
    {
        if (dataMsg->getReceiver() == myAddr)
        {
            EV << "I'm the relay this data message aimed at, cooperative cache data for downloader [" << dataMsg->getDownloader() << "].\n";
            if (downloaderInfo->cacheStartOffset == -1)
            {
                downloaderInfo->cacheStartOffset = dataMsg->getCurOffset() - dataMsg->getBytesNum();
                EV << "cached start offset is " << downloaderInfo->cacheStartOffset << std::endl;
            }
            downloaderInfo->cacheEndOffset = dataMsg->getCurOffset();
            EV << "cached end offset updates to " << downloaderInfo->cacheEndOffset << std::endl;
        }
        else
        {
            EV << "I'm not the relay this data message aimed at, ignore it.\n";
        }
        break;
    }
    case DOWNLOADER:
    {
        ASSERT( dataMsg->getReceiver() == myAddr );
        EV << "I'm the downloader this data message aimed at, check the order of received data.\n";
        if (dataMsg->getCurOffset() - dataMsg->getBytesNum() == downloaderInfo->acknowledgedOffset)
        {
            EV << "the order of data is ok, update acknowledged offset to " << dataMsg->getCurOffset() << ".\n";
            downloaderInfo->acknowledgedOffset = dataMsg->getCurOffset();
            if (dataMsg->getIsLast()) // it is the last data packet in transmission, thus response an acknowledge message
            {
                EV << "data receiving process has finished, response an acknowledge message.\n";
                WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, type_CCH, contentPriority, -1);
                if (wsm == nullptr) return;
                ContentMessage *acknowledgeMsg = dynamic_cast<ContentMessage*>(wsm);

                acknowledgeMsg->setControlCode(ContentMsgCC::ACKNOWLEDGEMENT);
                acknowledgeMsg->setDownloader(dataMsg->getDownloader());
                acknowledgeMsg->setReceivedOffset(downloaderInfo->acknowledgedOffset);

                sendWSM(acknowledgeMsg);
            }
        }
        else
        {
            EV << "the order of data is error, expected start offset is " << downloaderInfo->acknowledgedOffset << " whereas actual start offset is " << dataMsg->getCurOffset() - dataMsg->getBytesNum() << ".\n";
        }
        break;
    }
    case STRANGER:
    {
        EV << "I'm a stranger of downloader [" << dataMsg->getDownloader() << "], ignore it.\n";
        break;
    }
    case CARRIER:
    {
        EV << "I'm the carrier this data message aimed at, cooperative cache data for downloader [" << dataMsg->getDownloader() << "].\n";
        if (downloaderInfo->cacheStartOffset == -1)
        {
            downloaderInfo->cacheStartOffset = dataMsg->getCurOffset() - dataMsg->getBytesNum();
            EV << "cached start offset is " << downloaderInfo->cacheStartOffset << std::endl;
        }
        downloaderInfo->cacheEndOffset = dataMsg->getCurOffset();
        EV << "cached end offset updates to " << downloaderInfo->cacheEndOffset << std::endl;
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
    if (wsm == nullptr) return;
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

    selfInfo = new DownloaderInfo(size);
    selfInfo->myRole = DOWNLOADER;
    downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(myAddr, selfInfo));

    decorateWSM(contentMessage);
    sendWSM(contentMessage);

    scheduleAt(simTime() + requestTimeoutDuration, requestTimeoutEvt);
    EV << "set request timeout timer, its duration is " << requestTimeoutDuration.dbl() << "s.\n";

    /*
    CellularMessage *cellularMsg = new CellularMessage("cellular");
    cellularMsg->setControlCode(0);
    cellularMsg->setDownloader(myAddr);
    cellularMsg->addBitLength(headerLength);
    sendDirect(cellularMsg, SimTime::ZERO, SimTime::ZERO, baseStationGate);
    */
}

ContentClient::~ContentClient()
{

}
