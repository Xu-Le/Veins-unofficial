//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/rsu/BaseRSU.h"

void BaseRSU::initialize(int stage)
{
	BaseApplLayer::initialize(stage);

	if (stage == 0)
	{
		wiredIn = findGate("wiredIn");
		wiredOut = findGate("wiredOut");
		westIn = findGate("westIn");
		westOut = findGate("westOut");
		eastIn = findGate("eastIn");
		eastOut = findGate("eastOut");
		wiredHeaderLength = par("wiredHeaderLength").longValue();

		myAddr += RSU_ADDRESS_OFFSET;

		curPosition.x = par("positionX").doubleValue();
		curPosition.y = par("positionY").doubleValue();
		curPosition.z = par("positionZ").doubleValue();

		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(getParentModule());
		ASSERT(myMac);
		annotations = Veins::AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		helpRoutings = par("helpRoutings").boolValue();
		sendWarnings = par("sendWarnings").boolValue();
		sendContents = par("sendContents").boolValue();
		dataOnSch = par("dataOnSch").boolValue();

		warningLengthBits = par("warningLengthBits").longValue();
		warningPriority = par("warningPriority").longValue();
		contentLengthBits = par("contentLengthBits").longValue();
		contentPriority = par("contentPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();
		maxHopConstraint = par("maxHopConstraint").longValue();
		whichSide = par("whichSide").longValue();

		examineVehiclesInterval = par("examineVehiclesInterval").doubleValue();
		forgetMemoryInterval = par("forgetMemoryInterval").doubleValue();
		vehicleElapsed = par("vehicleElapsed").doubleValue();
		memoryElapsed = par("memoryElapsed").doubleValue();

		examineVehiclesEvt = new cMessage("examine vehicles evt", RSUMessageKinds::EXAMINE_VEHICLES_EVT);
		forgetMemoryEvt = new cMessage("forget memory evt", RSUMessageKinds::FORGET_MEMORY_EVT);
		scheduleAt(simTime() + dblrand()*forgetMemoryInterval, forgetMemoryEvt);
	}
}

void BaseRSU::finish()
{
	EV << "base RSU module finish ..." << std::endl;

	// clear containers
	messageMemory.clear();

	// delete handle self message
	cancelAndDelete(examineVehiclesEvt);
	cancelAndDelete(forgetMemoryEvt);

	BaseLayer::finish();
}

void BaseRSU::handleMessage(cMessage *msg)
{
	if (msg->isSelfMessage())
		handleSelfMsg(msg);
	else if (msg->getArrivalGateId() == lowerLayerIn)
	{
		recordPacket(PassedMessage::INCOMING, PassedMessage::LOWER_DATA, msg);
		handleLowerMsg(msg);
	}
	else if (msg->getArrivalGateId() == lowerControlIn)
	{
		recordPacket(PassedMessage::INCOMING, PassedMessage::LOWER_CONTROL, msg);
		handleLowerControl(msg);
	}
	else if (msg->getArrivalGateId() == wiredIn)
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
		handleWiredMsg(wiredMsg);
		delete wiredMsg;
		wiredMsg = nullptr;
	}
	else if (msg->getArrivalGateId() == westIn)
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
		handleWestMsg(wiredMsg);
		delete wiredMsg;
		wiredMsg = nullptr;
	}
	else if (msg->getArrivalGateId() == eastIn)
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
		handleEastMsg(wiredMsg);
		delete wiredMsg;
		wiredMsg = nullptr;
	}
	else if (msg->getArrivalGateId() == -1)
		throw cRuntimeError("No self message and no gateID! Check configuration.");
	else
		throw cRuntimeError("Unknown gateID! Check configuration or override handleMessage().");
}

void BaseRSU::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case RSUMessageKinds::EXAMINE_VEHICLES_EVT:
	{
		examineVehicles();
		scheduleAt(simTime() + examineVehiclesInterval, examineVehiclesEvt);
		break;
	}
	case RSUMessageKinds::FORGET_MEMORY_EVT:
	{
		forgetMemory();
		scheduleAt(simTime() + forgetMemoryInterval, forgetMemoryEvt);
		break;
	}
	default:
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
	}
}

void BaseRSU::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "beacon") == 0)
	{
		BeaconMessage* beaconMsg = dynamic_cast<BeaconMessage*>(msg);
		ASSERT(beaconMsg);
		onBeacon(beaconMsg);
	}
	else if (strcmp(msg->getName(), "routing") == 0)
	{
		if ( helpRoutings )
		{
			RoutingMessage* routingMsg = dynamic_cast<RoutingMessage*>(msg);
			ASSERT(routingMsg);
			onRouting(routingMsg);
		}
		else
			EV << "RSU doesn't help routings.\n";
	}
	else if (strcmp(msg->getName(), "warning") == 0)
		EV << "RSU doesn't relay warning message received from vehicles.\n";
	else if (strcmp(msg->getName(), "content") == 0)
	{
		ContentMessage* contentMsg = dynamic_cast<ContentMessage*>(msg);
		ASSERT(contentMsg);
		onContent(contentMsg);
	}
	else if (strcmp(msg->getName(), "data") == 0)
	{
		DataMessage* dataMsg = dynamic_cast<DataMessage*>(msg);
		ASSERT(dataMsg);
		onData(dataMsg);
	}
	else
		EV << "unknown message (" << msg->getName() << ") received.\n";

	delete msg;
	msg = nullptr;
}

WaveShortMessage* BaseRSU::prepareWSM(std::string name, int dataLength, t_channel channel, int priority, int serial)
{
	ASSERT( channel == type_CCH || channel == type_SCH );

	WaveShortMessage* wsm = nullptr;
	if (name == "warning")
	{
		EV << "Creating Warning with Priority " << priority << " at BaseRSU at " << simTime() << std::endl;
		wsm = new WarningMessage("warning");
	}
	else if (name == "content")
	{
		EV << "Creating Content with Priority " << priority << " at BaseRSU at " << simTime() << std::endl;
		wsm = new ContentMessage("content");
	}
	else if (name == "data")
	{
		EV << "Creating Data with Priority " << priority << " at BaseRSU at " << simTime() << std::endl;
		wsm = new DataMessage("data");
	}
	else
	{
		EV << "Undefined name, don't create it." << std::endl;
		return wsm;
	}

	wsm->addBitLength(headerLength + dataLength);

	if (channel == type_CCH)
		wsm->setChannelNumber(Channels::CCH);
	else // channel == type_SCH
		wsm->setChannelNumber(Channels::SCH1); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed

	wsm->setPriority(priority);
	wsm->setSerial(serial);
	wsm->setSenderAddress(myAddr);
	wsm->setSenderPos(curPosition);
	wsm->setTimestamp(simTime());

	return wsm;
}

void BaseRSU::decorateWSM(WaveShortMessage *wsm)
{
	if (strcmp(wsm->getName(), "beacon") == 0)
	{
		EV << "RSUs don't decorate beacon messages since they don't send beacons.\n";
		delete wsm;
		wsm = nullptr;
	}
	else if (strcmp(wsm->getName(), "routing") == 0)
	{
		if (helpRoutings)
		{
			// TODO: if RSU set to help routings, write codes here.
		}
		else
		{
			EV << "RSUs don't decorate routing messages since they don't help routings.\n";
			delete wsm;
			wsm = nullptr;
		}
	}
	else if (strcmp(wsm->getName(), "warning") == 0)
	{
		WarningMessage *warningMsg = dynamic_cast<WarningMessage*>(wsm);
		Coord senderSpeed; // it seems strange, but vehicle need this info to determine whether it is a target vehicle of the warning message
		if ( whichSide == (int)SideDirection::EAST_SIDE || whichSide == (int)SideDirection::SOUTH_SIDE )
			senderSpeed.x = 1.0;
		else // ( whichSide == (int)SideDirection::WEST_SIDE || whichSide == (int)SideDirection::NORTH_SIDE )
			senderSpeed.x = -1.0;
		warningMsg->setSenderSpeed(senderSpeed);

		int hopCount = warningMsg->getHopCount();
		++hopCount;
		warningMsg->setHopCount(hopCount);

		HopItems &hopInfo = warningMsg->getHopInfo();
		HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, 0.0, 0.0, 0.0);
		hopInfo.push_back(hopItem);
	}
	else if (strcmp(wsm->getName(), "content") == 0)
	{
		// Unused now
	}
	else if (strcmp(wsm->getName(), "data") == 0)
	{
		// Unused now
	}
	else
	{
		EV << "unknown message (" << wsm->getName() << ") decorated, delete it\n";
		delete wsm;
		wsm = nullptr;
	}
}

void BaseRSU::sendWSM(WaveShortMessage *wsm)
{
	sendDelayedDown(wsm, individualOffset);
}

void BaseRSU::onBeacon(BeaconMessage *beaconMsg)
{
	EV << logName() << ": " << "onBeacon!\n";

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias
	VehicleInfo *vehicleInfo = nullptr;
	if ( vehicles.find(sender) != vehicles.end() ) // update old record
	{
		EV << "    sender [" << sender << "] is an old vehicle, update its info.\n";
		vehicleInfo = vehicles[sender];
		vehicleInfo->pos = beaconMsg->getSenderPos();
		vehicleInfo->speed = beaconMsg->getSenderSpeed();
		vehicleInfo->receivedAt = simTime();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new vehicle, insert its info.\n";
		vehicleInfo = new VehicleInfo(beaconMsg->getSenderPos(), beaconMsg->getSenderSpeed(), simTime());
		vehicles.insert(std::pair<LAddress::L3Type, VehicleInfo*>(sender, vehicleInfo));
	}
	beaconMsg->removeControlInfo();

#if ROUTING_DEBUG_LOG
	EV << "    senderPos: " << beaconMsg->getSenderPos() << ", senderSpeed: " << beaconMsg->getSenderSpeed() << std::endl;
	EV << "display all vehicles' information of " << logName() << std::endl;
	for (std::map<LAddress::L3Type, VehicleInfo*>::iterator iter = vehicles.begin(); iter != vehicles.end(); ++iter)
	{
		EV << "vehicle[" << iter->first << "]:  pos:" << iter->second->pos << ", speed:" << iter->second->speed << std::endl;
	}
#endif
}

void BaseRSU::examineVehicles()
{
	double curTime = simTime().dbl(); // alias
	for (std::map<LAddress::L3Type, VehicleInfo*>::iterator iter = vehicles.begin(); iter != vehicles.end();)
	{
		if ( curTime - iter->second->receivedAt.dbl() > vehicleElapsed )
		{
			EV << logName() << " disconnected from vehicle[" << iter->first << "], delete its info.\n";
			/* derived class's extension write here before it is deleted */
			delete iter->second;
			vehicles.erase(iter++);
		}
		else
			++iter;
	}
}

void BaseRSU::forgetMemory()
{
	double curTime = simTime().dbl(); // alias
	for (std::map<int, simtime_t>::iterator iter = messageMemory.begin(); iter != messageMemory.end();)
	{
		if (curTime - iter->second.dbl() > memoryElapsed)
		{
			EV << logName() << " forgets message(GUID=" << iter->first << "), delete it from message memory.\n";
			messageMemory.erase(iter++);
		}
		else
			++iter;
	}
}

