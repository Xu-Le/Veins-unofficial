//
// Copyright (C) 2016-2019 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __ROUTINGRSU_H__
#define __ROUTINGRSU_H__

#include "veins/modules/rsu/BaseRSU.h"
#include "veins/modules/messages/RoutingMessage_m.h"
#include "veins/modules/messages/DataMessage_m.h"
#include "veins/modules/routing/RoutingStatisticCollector.h"

/**
 * @brief A concrete RSU class which is aware of routing protocols.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRSU
 */
class RoutingRSU : public BaseRSU
{
public:
	/** @name constructors, destructor. */
	///@{
	RoutingRSU() : BaseRSU() {}
	RoutingRSU(unsigned stacksize) : BaseRSU(stacksize) {}
	~RoutingRSU() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from lower layer. */
	void handleLowerMsg(cMessage *msg) override;

	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *routingMsg);
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg);

private:
	int routingLengthBits; ///< the length of routing message measured in bits.
	int routingPriority;   ///< the priority of routing message.
	int dataLengthBits;    ///< the length of data message measured in bits.
	int dataPriority;      ///< the priority of data message.
};

#endif /* __ROUTINGRSU_H__ */
