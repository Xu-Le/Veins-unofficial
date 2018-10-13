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

#include "veins/modules/rsu/RoutingRSU.h"

Define_Module(RoutingRSU);

void RoutingRSU::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();

		scheduleAt(simTime() + dblrand()*forgetMemoryInterval, forgetMemoryEvt);
	}
}

void RoutingRSU::finish()
{
	BaseRSU::finish();
}

void RoutingRSU::handleSelfMsg(cMessage *msg)
{
	BaseRSU::handleSelfMsg(msg);
}

void RoutingRSU::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "routing") == 0)
		DYNAMIC_CAST_CMESSAGE(Routing, routing)
	else if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseRSU::handleLowerMsg(msg);
}

void RoutingRSU::onRouting(RoutingMessage *routingMsg)
{
	EV << logName() << ": onRouting!\n";

	int guid = routingMsg->getGUID(); // alias

	if ( messageMemory.find(guid) != messageMemory.end() )
	{
		EV << "routing message(GUID=" << guid << ") has been rebroadcast recently, discard it.\n";
		return;
	}

	// check if the hop count of the message exceeds
	if ( routingMsg->getHopCount() > maxHopConstraint )
	{
		EV << "routing message(GUID=" << guid << ") exceeds its maximum hop count, discard it.\n";
		return;
	}

	// catch a new routing message
	EV << "catch a routing message(GUID=" << guid << "), help to rebroadcast it.\n";

	messageMemory.insert(std::pair<int, simtime_t>(guid, simTime()));

	RoutingMessage *dupWSM = new RoutingMessage(*routingMsg);
	sendWSM(dupWSM);
}

void RoutingRSU::onData(DataMessage *dataMsg)
{
	EV << logName() << ": onData!\n";
}
