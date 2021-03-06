//
// Copyright (C) 2019 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/rsu/DummyRSU.h"

Define_Module(DummyRSU);

void DummyRSU::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();
	}
}

void DummyRSU::finish()
{
	BaseRSU::finish();
}

void DummyRSU::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case RSUMsgKinds::DUMMY_RSU_EVT:
	{

		break;
	}
	default:
		BaseRSU::handleSelfMsg(msg);
	}
}

void DummyRSU::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseRSU::handleLowerMsg(msg);
}

void DummyRSU::handleLowerControl(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		onDataLost(dynamic_cast<DataMessage*>(msg));
}

void DummyRSU::onData(DataMessage *dataMsg)
{
	EV << logName() << ": onData!\n";
}

void DummyRSU::onDataLost(DataMessage *lostDataMsg)
{
	EV << logName() << ": onDataLost!\n";
}
