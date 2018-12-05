//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/uav/RoutingUAV.h"

Define_Module(RoutingUAV);

void RoutingUAV::initialize(int stage)
{
	BaseUAV::initialize(stage);

	if (stage == 0)
	{
		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();

		flyingEvt = new cMessage("flying evt", RoutingUAVMessageKinds::FLYING_EVT);
		decideEvt = new cMessage("decide evt", RoutingUAVMessageKinds::DECIDE_EVT);
		scheduleAt(simTime(), decideEvt);
	}
}

void RoutingUAV::finish()
{
	cancelAndDelete(flyingEvt);
	cancelAndDelete(decideEvt);

	BaseUAV::finish();
}

void RoutingUAV::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case RoutingUAVMessageKinds::FLYING_EVT:
	{

		break;
	}
	case RoutingUAVMessageKinds::DECIDE_EVT:
	{

		break;
	}
	default:
		BaseUAV::handleSelfMsg(msg);
	}
}

void RoutingUAV::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "routing") == 0)
		DYNAMIC_CAST_CMESSAGE(Routing, routing)
	else if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseUAV::handleLowerMsg(msg);
}

void RoutingUAV::handleLowerControl(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		onDataLost(dynamic_cast<DataMessage*>(msg));
}

void RoutingUAV::decorateUavBeacon(UavBeaconMessage *uavBeaconMsg)
{

}

void RoutingUAV::onUavBeacon(UavBeaconMessage *uavBeaconMsg)
{
	EV << logName() << ": onUavBeacon!\n";

	LAddress::L3Type sender = uavBeaconMsg->getSenderAddress(); // alias
	RoutingNeighborInfo *neighborInfo = nullptr;
	if ((itN = neighbors.find(sender)) != neighbors.end()) // update old record
	{
		EV << "    sender [" << sender << "] is an old neighbor, update its info.\n";
		neighborInfo = dynamic_cast<RoutingNeighborInfo*>(itN->second);
		neighborInfo->pos = uavBeaconMsg->getSenderPos();
		neighborInfo->speed = uavBeaconMsg->getSenderSpeed();
		neighborInfo->receivedAt = simTime();
		neighborInfo->reserved = uavBeaconMsg->getReserved();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new neighbor, insert its info.\n";
		neighborInfo = new RoutingNeighborInfo(uavBeaconMsg->getSenderPos(), uavBeaconMsg->getSenderSpeed(), simTime(), uavBeaconMsg->getReserved());
		neighbors.insert(std::pair<LAddress::L3Type, NeighborInfo*>(sender, neighborInfo));
	}
}

void RoutingUAV::onRouting(RoutingMessage *routingMsg)
{
	EV << logName() << ": onRouting!\n";

	// int guid = routingMsg->getGUID(); // alias
}

void RoutingUAV::onData(DataMessage* dataMsg)
{
	EV << logName() << ": onData!\n";
}

void RoutingUAV::onDataLost(DataMessage *lostDataMsg)
{
	EV << logName() << ": onDataLost!\n";
}
