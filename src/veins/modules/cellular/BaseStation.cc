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

#include "veins/modules/cellular/BaseStation.h"

Define_Module(BaseStation);

void BaseStation::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		EV << "BaseStation::initialize() called.\n";

		wirelessIn = findGate("wirelessIn");
		wiredIn = findGate("wiredIn");
		wiredOut = findGate("wiredOut");
		wiredHeaderLength = par("wiredHeaderLength").longValue();
		wirelessHeaderLength = par("wirelessHeaderLength").longValue();
		wirelessDataLength = par("wirelessDataLength").longValue();
		wirelessBitsRate = par("wirelessBitsRate").longValue();

		rootModule = cSimulation::getActiveSimulation()->getSystemModule();
	}
	else
	{
		EV << "BaseStation initialized.\n";
	}
}

void BaseStation::finish()
{
	EV << "BaseStation::finish() called.\n";

	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		delete itV->second;
	vehicles.clear();

	cComponent::finish();
}

void BaseStation::handleMessage(cMessage *msg)
{
	if (msg->isSelfMessage())
		handleSelfMsg(msg);
	else if (msg->getArrivalGateId() == wirelessIn)
	{
		CellularMessage *cellularMsg = dynamic_cast<CellularMessage*>(msg);
		handleWirelessIncomingMsg(cellularMsg);
		delete cellularMsg;
		cellularMsg = nullptr;
	}
	else if (msg->getArrivalGateId() == wiredIn)
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
		handleWiredIncomingMsg(wiredMsg);
		delete wiredMsg;
		wiredMsg = nullptr;
	}
	else if (msg->getArrivalGateId() == -1)
		error("No self message and no gateID?? Check configuration.");
	else
		error("Unknown gateID?? Check configuration or override handleMessage().");
}

void BaseStation::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case SelfMsgKinds::LAST_BASE_STATION_MESSAGE_KIND:
	{
		break;
	}
	default:
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << std::endl;
	}
}

void BaseStation::handleWirelessIncomingMsg(CellularMessage *cellularMsg)
{
	LAddress::L3Type vehicle = cellularMsg->getVehicle(); // alias
	EV << "receiving a message from vehicle " << vehicle << ".\n";

	VehicleInfo *info = new VehicleInfo;
	info->correspondingGate = rootModule->getSubmodule("node", vehicle)->gate("veinscellularIn");
	vehicles.insert(std::pair<LAddress::L3Type, VehicleInfo*>(vehicle, info));
}

void BaseStation::handleWiredIncomingMsg(WiredMessage *wiredMsg)
{
	EV << "receiving a message from base server.\n";
}

/////////////////////////    internal class implementations    /////////////////////////
BaseStation::VehicleInfo::VehicleInfo() : correspondingGate(nullptr)
{

}
