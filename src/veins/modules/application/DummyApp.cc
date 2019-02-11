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

#include "veins/modules/application/DummyApp.h"

Define_Module(DummyApp);

void DummyApp::initialize(int stage)
{
	BaseWaveApplLayer::initialize(stage);

	if (stage == 0)
	{
		// parse parameters specified in .ned file

		// create and schedule messages

	}
}

void DummyApp::finish()
{
	// clear containers

	// cancel and delete messages

	BaseWaveApplLayer::finish();
}

void DummyApp::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case AppMsgKinds::DUMMY_APP_EVT:
	{

		break;
	}
	default:
		BaseWaveApplLayer::handleSelfMsg(msg);
	}
}

void DummyApp::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		DYNAMIC_CAST_CMESSAGE(Data, data)

	BaseWaveApplLayer::handleLowerMsg(msg);
}

void DummyApp::handleLowerControl(cMessage *msg)
{
	if (strcmp(msg->getName(), "data") == 0)
		onDataLost(dynamic_cast<DataMessage*>(msg));
}

void DummyApp::onData(DataMessage *dataMsg)
{
	EV << "node[" << myAddr << "]: onData!\n";
}

void DummyApp::onDataLost(DataMessage *lostDataMsg)
{
	EV << "node[" << myAddr << "]: onDataLost!\n";
}
