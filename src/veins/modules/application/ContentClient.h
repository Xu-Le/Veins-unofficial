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

#ifndef __CONTENTCLIENT_H__
#define __CONTENTCLIENT_H__

#include "veins/modules/wave/BaseWaveApplLayer.h"

#define STRANGER      1
#define DOWNLOADER    2
#define RELAY         3
#define CARRIER       4

/**
 * @brief File Content Client.
 *
 * @author Xu Le
 * @ingroup applLayer
 * @see BaseRoutingLayer
 */
class ContentClient : public BaseWaveApplLayer
{
public:
    /** @brief The message kinds routing layer uses. */
    enum ContentClientMsgKinds {
        RELAY_EVT = LAST_WAVE_APPL_MESSAGE_KIND,
        SCHEME_SWITCH_EVT,
        REQUEST_TIMEOUT_EVT,
        INTERRUPT_TIMEOUT_EVT
    };

    /** @name constructor, destructor. */
    ///@{
    ContentClient() : BaseWaveApplLayer() {}
    ContentClient(unsigned stacksize) : BaseWaveApplLayer(stacksize) {}
    virtual ~ContentClient();
    ///@}

    void initialize(int stage) override;
    void finish() override;

private:
    /** @brief Called every time a message arrives. */
    void handleSelfMsg(omnetpp::cMessage *msg) override;

    /** @brief Handle wireless cellular incoming messages. */
    void handleCellularMsg(CellularMessage *cellularMsg) override;

    /** @brief call-back method of receiving beacon message. */
    void onBeacon(BeaconMessage *beaconMsg) override;
    /** @brief call-back method of receiving routing message(aims to unicast protocols). */
    void onRouting(RoutingMessage *routingMsg) override;
    /** @brief call-back method of receiving warning message(aims to broadcast and geocast protocols). */
    void onWarning(WarningMessage *warningMsg) override;
    /** @brief call-back method of receiving content message(aims to download application). */
    void onContent(ContentMessage *contentMsg) override;
    /** @brief call-back method of receiving data message. */
    void onData(DataMessage *dataMsg) override;

    /** @brief call a content request for certain size determined by contentPlanList. */
    void callContent(int size) override;

private:
    /** @brief Data consuming rate of some common video quality kinds. */
    enum VideoQuailty {
        _QCIF = 38400, // 38KB/s
        _CIF = 102400, // 100KB/s
        _VGA = 230400, // 225KB/s
        _720P = 448000, // 438KB/s
        _1080P = 1088000 // 1063KB/s
    };
    /** @brief The class to store downloader's information. */
    class DownloaderInfo
    {
    public:
        DownloaderInfo(int t) : myRole(1), totalContentSize(t), cacheStartOffset(-1), cacheEndOffset(0), acknowledgedOffset(0) {}

        int myRole;
        int totalContentSize;
        int cacheStartOffset;
        int cacheEndOffset;
        int acknowledgedOffset;
    };

    BaseStation *baseStation;
    cGate *baseStationGate;

    int slotSpan; ///< unit prediction time slot span measured in millisecond.
    /** @name performance consideration. */
    ///@{
    int relayLinkBytesOnce; ///< how many bytes measured in link layer to relay to downloader once in transmission.
    int relayApplBytesOnce; ///< how many bytes measured in application layer to relay to downloader once in transmission.
    SimTime relayPeriod;    ///< period to handle self message relayEvt.
    SimTime schemeSwitchInterval; ///< interval from current time to the time schemeSwitchEvt happens(this interval is time-varying).
    ///@}

    simtime_t requestTimeoutDuration;   ///< when the request timer elapsed this duration, timeout event happens.
    simtime_t interruptTimeoutDuration; ///< when the interrupt timer elapsed this duration, timeout event happens.

    cMessage *relayEvt;            ///< self message used to handle with relay data packets to downloader.
    cMessage *schemeSwitchEvt;     ///< self message used to handle with transmission scheme switch in different time slot.
    cMessage *requestTimeoutEvt;   ///< self message used to handle with content request timeout event(this happens when the vehicle is not in any RSU's communication range area).
    cMessage *interruptTimeoutEvt; ///< self message used to handle with download interrupt timeout event.

    DownloaderInfo *selfInfo;
    std::list<SchemeTuple> schemeList; ///< describe how this vehicle transmit in each slot.
    std::map<LAddress::L3Type, DownloaderInfo*> downloaders; ///< a map from a downloader's identifier to all my related its info.
    std::map<LAddress::L3Type, DownloaderInfo*>::iterator itDL; ///< a iterator used to traverse container downloaders.
};

#endif /* __CONTENTCLIENT_H__ */
