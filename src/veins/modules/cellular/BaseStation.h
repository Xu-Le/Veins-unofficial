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

#ifndef __BASESTATION_H__
#define __BASESTATION_H__

#include <omnetpp/csimplemodule.h>
#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/messages/CellularMessage_m.h"
#include "veins/modules/messages/WiredMessage_m.h"

using omnetpp::SimTime;
using omnetpp::cMessage;

/**
 * @brief LTE Base Station.
 *
 * @author Xu Le
 */
class BaseStation : public ::omnetpp::cSimpleModule
{
public:
	/** @name constructor, destructor. */
	///@{
	BaseStation() : cSimpleModule() {}
	BaseStation(unsigned stacksize) : cSimpleModule(stacksize) {}
	virtual ~BaseStation() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
    /** @brief The self message kinds. */
    enum SelfMsgKinds {
        DISTRIBUTE_EVT
    };

	/** @brief Called every time a message arrives. */
	void handleMessage(omnetpp::cMessage *msg) override;

	/** @brief Handle self messages. */
	void handleSelfMsg(omnetpp::cMessage *msg);
	/** @brief Handle wireless incoming messages. */
	void handleWirelessIncomingMsg(CellularMessage *cellularMsg);
	/** @brief Handle wired incoming messages. */
	void handleWiredIncomingMsg(WiredMessage *wiredMsg);

public:
    /** @brief The class to store downloader's information. */
    class DownloaderInfo
    {
    public:
        DownloaderInfo(int t, int s, int r) : totalContentSize(t), cacheStartOffset(s), cacheEndOffset(0), distributedOffset(s), requiredEndOffset(r), distributedAt(SimTime::ZERO), correspondingGate(nullptr) {}

        int totalContentSize;
        int cacheStartOffset;
        int cacheEndOffset;
        int distributedOffset;
        int requiredEndOffset;
        SimTime distributedAt;
        cGate *correspondingGate;
    };

	/** @name gate IDs. */
	///@{
	int wirelessIn; ///< receive packets from vehicles.
	int wiredIn;    ///< receive packets from file content server.
	int wiredOut;   ///< send packets to file content server.
	///@}

	int wiredHeaderLength; ///< length of the UDP/IP packet header measured in bits.
	int wirelessHeaderLength; ///< length of the cellular packet header measured in bits.
	int wirelessDataLength; ///< length of the cellular packet data measured in bits.
	int wirelessBitsRate; ///< data transmission rate measured in bps of wireless radio.

    /** @name performance consideration. */
    ///@{
    int distributeLinkBytesOnce; ///< how many bytes measured in link layer to distribute to vehicle once in transmission.
    int distributeApplBytesOnce; ///< how many bytes measured in application layer to distribute to vehicle once in transmission.
    SimTime distributePeriod;  ///< period to handle self message distributeEvt.
    ///@}

    cMessage *distributeEvt; ///< self message used to periodically distribute data to vehicle.

	cModule *rootModule; ///< store the pointer to system module to find the sender vehicle's compound module.

    std::map<LAddress::L3Type, DownloaderInfo*> downloaders; ///< a map from a downloader's identifier to all its related info.
    std::map<LAddress::L3Type, DownloaderInfo*>::iterator itDL; ///< a iterator used to traverse container downloaders.
};

#endif /* __BASESTATION_H__ */
