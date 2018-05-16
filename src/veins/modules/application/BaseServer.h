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

#ifndef __BASESERVER_H__
#define __BASESERVER_H__

#include <omnetpp/csimplemodule.h>
#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/messages/WiredMessage_m.h"

/**
 * @brief Base class for Server design.
 *
 * @author Xu Le
 * @ingroup applLayer
 * @see BaseStation
 */
class BaseServer : public ::omnetpp::cSimpleModule
{
public:
	/** @name constructor, destructor. */
	///@{
	BaseServer() : cSimpleModule() {}
	BaseServer(unsigned stacksize) : cSimpleModule(stacksize) {}
	virtual ~BaseServer() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief The self message kinds. */
	enum SelfMsgKinds {
		LAST_SERVER_MESSAGE_KIND
	};

	/** @brief Called every time a message arrives. */
	void handleMessage(omnetpp::cMessage *msg) override;

	/** @brief Handle self messages. */
	void handleSelfMsg(omnetpp::cMessage *msg);
	/** @brief Handle wired incoming messages from cellular base station. */
	void handleLTEIncomingMsg(WiredMessage *lteMsg);
	/** @brief Handle wired incoming messages from road side unit. */
	void handleRSUIncomingMsg(WiredMessage *rsuMsg, int rsuIdx);

private:
	int rsuNum; ///< number of RSUs in current scenario.
	/** @name gate IDs. */
	///@{
	int *rsuIn;      ///< receive packets from road side units.
	int *rsuOut;     ///< send packets to road side units.
	int cellularIn;  ///< receive packets from cellular base station.
	int cellularOut; ///< send packets to cellular base station.
	///@}
	int headerLength; ///< header length of the wired UDP/IP packet in bits.
};

#endif /* __BASESERVER_H__ */
