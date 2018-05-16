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

#include "veins/modules/application/BaseServer.h"

Define_Module(BaseServer);

void BaseServer::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		rsuNum = par("rsuNum").longValue();
		rsuIn = new int[rsuNum];
		rsuOut = new int[rsuNum];
		for (int i = 0; i < rsuNum; ++i)
		{
			rsuIn[i] = findGate("rsuIn", i);
			rsuOut[i] = findGate("rsuOut", i);
		}
		cellularIn = findGate("cellularIn");
		cellularOut = findGate("cellularOut");
		headerLength = par("headerLength").longValue();
	}
	else
	{
		EV << "BaseServer initialized.\n";
	}
}

void BaseServer::finish()
{
	EV << "BaseServer::finish() called.\n";

	delete []rsuIn;
	delete []rsuOut;

	cComponent::finish();
}

void BaseServer::handleMessage(cMessage *msg)
{
	if (msg->isSelfMessage())
		handleSelfMsg(msg);
	else if (msg->getArrivalGateId() >= rsuIn[0] && msg->getArrivalGateId() <= rsuIn[rsuNum-1])
	{
		WiredMessage *rsuMsg = dynamic_cast<WiredMessage*>(msg);
		for (int rsuIdx = 0; rsuIdx < rsuNum; ++rsuIdx)
		{
			if (msg->getArrivalGateId() == rsuIn[rsuIdx])
			{
				handleRSUIncomingMsg(rsuMsg, rsuIdx);
				delete rsuMsg;
				rsuMsg = nullptr;
				break;
			}
		}
	}
	else if (msg->getArrivalGateId() == cellularIn)
	{
		WiredMessage *lteMsg = dynamic_cast<WiredMessage*>(msg);
		handleLTEIncomingMsg(lteMsg);
		delete lteMsg;
		lteMsg = nullptr;
	}
	else if (msg->getArrivalGateId() == -1)
		error("No self message and no gateID! Check configuration.");
	else
		error("Unknown gateID! Check configuration or override handleMessage().");
}

void BaseServer::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case SelfMsgKinds::LAST_SERVER_MESSAGE_KIND:
	{
		break;
	}
	default:
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << std::endl;
	}
}

void BaseServer::handleRSUIncomingMsg(WiredMessage *rsuMsg, int rsuIdx)
{
	EV << "receiving a message from rsu[" << rsuIdx << "].\n";
}

void BaseServer::handleLTEIncomingMsg(WiredMessage *lteMsg)
{
	EV << "receiving a message from cellular base station.\n";
}
