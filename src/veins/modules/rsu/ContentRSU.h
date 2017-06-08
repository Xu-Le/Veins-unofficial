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

#ifndef __CONTENTRSU_H__
#define __CONTENTRSU_H__

#include "veins/modules/rsu/BaseRSU.h"
#include "veins/modules/rsu/STAG.h"

/**
 * @brief A concrete RSU class which provides content download service.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRSU
 */
class ContentRSU : public BaseRSU
{
public:
    /** @brief The message kinds content RSU uses. */
    enum ContentRSUMsgKinds {
        DISTRIBUTE_V_EVT = LAST_BASE_RSU_MESSAGE_KIND,
        DISTRIBUTE_R_EVT,
        SCHEME_SWITCH_EVT,
        LAST_CONTENT_RSU_MESSAGE_KIND
    };

    /** @name constructors, destructor. */
    ///@{
    ContentRSU() : BaseRSU() {}
    ContentRSU(unsigned stacksize) : BaseRSU(stacksize) {}
    virtual ~ContentRSU() {}
    ///@}

    void initialize(int stage) override;
    void finish() override;

private:
    /** @brief handle self messages. */
    void handleSelfMsg(cMessage *msg) override;
    /** @brief Handle wired incoming messages. */
    void handleWiredMsg(WiredMessage *wiredMsg) override;

    /** @brief wave short message decorate method. */
    void decorateWSM(WaveShortMessage *wsm) override;

    /** @brief call-back method of receiving beacon message. */
    void onBeacon(BeaconMessage *beaconMsg) override;
    /** @brief call-back method of receiving routing message. */
    void onRouting(RoutingMessage *routingMsg) override;
    /** @brief call-back method of receiving content message. */
    void onContent(ContentMessage *contentMsg) override;
    /** @brief call-back method of receiving data message. */
    void onData(DataMessage *dataMsg) override;

    /** @brief calculate the proper data amount to prefetch from content server. */
    SimTime obtainCooperativeScheme();
    /** @brief predict links' bandwidth between nodes in a short time according to the mobility information of vehicles. */
    void predictLinkBandwidth();
    /** @brief print links' bandwidth between nodes for debug purpose. */
    void printLinkBandwidth();

private:
    /** @brief The class to store downloader's information. */
    class DownloaderInfo
    {
    public:
        DownloaderInfo(int t, int c) : totalContentSize(t), cacheStartOffset(0), cacheEndOffset(0), distributedOffset(0),
            acknowledgedOffset(0), remainingDataAmount(0), consumingRate(c), prefetchDataAmount(0), distributedAt(SimTime::ZERO) {}

        int totalContentSize;
        int cacheStartOffset;
        int cacheEndOffset;
        int distributedOffset;
        int acknowledgedOffset;
        int remainingDataAmount;
        int consumingRate;
        int prefetchDataAmount;
        SimTime distributedAt;
    };

    /** @name STAG related variables and containers. */
    ///@{
    int nodeNum;
    int linkNum;
    int downloaderNum;
    int slotNum;  ///< prediction time slot number, the result of slotSpan*slotNum is prediction period.
    int slotSpan; ///< unit prediction time slot span measured in millisecond.
    std::vector<int> downloaderArray;
    std::map<int, int> downloaderTable;
    std::map<int, int> remainingTable;
    std::map<int, int> playTable;
    std::list<LinkTuple> linkTuples;
    ///@}

    SimTime prevSlotStartTime; ///< store the time when transmission in previous slot is started.

    /** @name performance consideration. */
    ///@{
    int distributeVLinkBytesOnce; ///< how many bytes measured in link layer to distribute to vehicle once in transmission.
    int distributeRLinkBytesOnce; ///< how many bytes measured in link layer to distribute to other RSU once in transmission.
    int distributeVApplBytesOnce; ///< how many bytes measured in application layer to distribute to vehicle once in transmission.
    int distributeRApplBytesOnce; ///< how many bytes measured in application layer to distribute to other RSU once in transmission.
    SimTime distributeVPeriod;  ///< period to handle self message distributeVEvt.
    SimTime distributeRPeriod;  ///< period to handle self message distributeREvt.
    SimTime schemeSwitchInterval; ///< interval from current time to the time schemeSwitchEvt happens(this interval is time-varying).
    ///@}

    cMessage *distributeVEvt; ///< self message used to periodically distribute data to vehicle.
    cMessage *distributeREvt; ///< self message used to periodically distribute data to other RSU.
    cMessage *schemeSwitchEvt;     ///< self message used to handle with transmission scheme switch in different time slot.

    std::list<SchemeTuple> rsuSchemeList; ///< describe how this RSU transmit in each slot.
    std::list<SchemeTuple>::iterator itRSL; ///< an iterator used to iterate container rsuSchemeList.
    SchemeItems schemeItemList; ///< describe how each vehicle transmit in each slot.
    std::map<LAddress::L3Type, DownloaderInfo*> downloaders; ///< a map from a downloader's identifier to all its related info.
    std::map<LAddress::L3Type, DownloaderInfo*>::iterator itDL; ///< an iterator used to traverse container downloaders.
};

#endif /* __CONTENTRSU_H__ */
