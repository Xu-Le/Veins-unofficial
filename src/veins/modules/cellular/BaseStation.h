//
// Copyright (C) 2017-2018 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/messages/CellularMessage_m.h"
#include "veins/modules/messages/WiredMessage_m.h"

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
		LAST_BASE_STATION_MESSAGE_KIND
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
	class VehicleInfo
	{
	public:
		VehicleInfo();

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

	cModule *rootModule; ///< store the pointer to system module to find the sender vehicle's compound module.

	std::map<LAddress::L3Type, VehicleInfo*> vehicles; ///< a map from a vehicle's identifier to all its related info.
	std::map<LAddress::L3Type, VehicleInfo*>::iterator itV; ///< a iterator used to traverse container vehicles.
};

#endif /* __BASESTATION_H__ */
