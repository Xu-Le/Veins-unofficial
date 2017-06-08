//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "veins/modules/cellular/BaseStation.h"

using omnetpp::SimTime;
using omnetpp::cMessage;

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
        rootModule = omnetpp::cSimulation::getActiveSimulation()->getSystemModule();
    }
    else
    {
        EV << "BaseStation initialized.\n";
    }
}

void BaseStation::finish()
{
    EV << "BaseStation::finish() called.\n";

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
    }
    else if (msg->getArrivalGateId() == wiredIn)
    {
        WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
        handleWiredIncomingMsg(wiredMsg);
    }
    else if (msg->getArrivalGateId() == -1)
        error("No self message and no gateID?? Check configuration.");
    else
        error("Unknown gateID?? Check configuration or override handleMessage().");
}

void BaseStation::handleSelfMsg(cMessage *msg)
{

}

void BaseStation::handleWirelessIncomingMsg(CellularMessage *cellularMsg)
{
    EV << "BaseStation::handleWirelessIncomingMsg called.\n";
    WiredMessage *wiredMsg = new WiredMessage("wired");
    wiredMsg->setControlCode(0);
    wiredMsg->setDownloader(cellularMsg->getDownloader());
    wiredMsg->addBitLength(wiredHeaderLength);
    sendDelayed(wiredMsg, SimTime::ZERO, wiredOut);
    delete cellularMsg;
    cellularMsg = nullptr;
}

void BaseStation::handleWiredIncomingMsg(WiredMessage *wiredMsg)
{
    EV << "BaseStation::handleWiredIncomingMsg called.\n";
    cModule *senderVehicleModule = rootModule->getSubmodule("node", wiredMsg->getDownloader());
    CellularMessage *cellularMsg = new CellularMessage("cellular");
    cellularMsg->setControlCode(0);
    cellularMsg->setDownloader(wiredMsg->getDownloader());
    cellularMsg->addBitLength(wirelessHeaderLength);
    sendDirect(cellularMsg, SimTime::ZERO, SimTime::ZERO, senderVehicleModule->gate("veinscellularIn"));
    delete wiredMsg;
    wiredMsg = nullptr;
}
