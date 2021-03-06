//
// Copyright (C) 2016-2018 Xu Le <xmutongxinXuLe@163.com>
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
		westDist = par("westDistance").longValue();
		eastDist = par("eastDistance").longValue();
		wiredHeaderLength = par("wiredHeaderLength").longValue();

		myAddr += RSU_ADDRESS_OFFSET;

		curPosition.x = par("positionX").doubleValue();
		curPosition.y = par("positionY").doubleValue();
		curPosition.z = par("positionZ").doubleValue();

		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(getParentModule());
		ASSERT(myMac);
		annotations = Veins::AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		V2XRadius = BaseConnectionManager::maxInterferenceDistance;
		U2URadius = BaseConnectionManager::maxInterferenceDistance2;

		dataOnSch = par("dataOnSch").boolValue();

		whichSide = par("whichSide").longValue();

		examineVehiclesInterval = par("examineVehiclesInterval").doubleValue();
		forgetMemoryInterval = par("forgetMemoryInterval").doubleValue();
		vehicleElapsed = par("vehicleElapsed").doubleValue();
		memoryElapsed = par("memoryElapsed").doubleValue();

		examineVehiclesEvt = new cMessage("examine vehicles evt", BaseRSUMsgKinds::EXAMINE_VEHICLES_EVT);
		forgetMemoryEvt = new cMessage("forget memory evt", BaseRSUMsgKinds::FORGET_MEMORY_EVT); // derived classes schedule it
		scheduleAt(simTime() + dblrand()*examineVehiclesInterval, examineVehiclesEvt);
	}
}

void BaseRSU::finish()
{
	EV << "base RSU module finish ..." << std::endl;

	// clear containers
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		delete itV->second;
	vehicles.clear();
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
		WaveShortMessage *wsm = dynamic_cast<WaveShortMessage*>(msg);
		if (wsm->getSenderAddress() >= RSU_ADDRESS_OFFSET
			|| curPosition.sqrdist(wsm->getSenderPos()) <= BaseConnectionManager::maxDistSquared)
		{
			recordPacket(PassedMessage::INCOMING, PassedMessage::LOWER_DATA, msg);
			handleLowerMsg(msg);
		}
		DELETE_SAFELY(msg);
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
		DELETE_SAFELY(wiredMsg);
	}
	else if (msg->getArrivalGateId() == westIn)
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
		handleWestMsg(wiredMsg);
		DELETE_SAFELY(wiredMsg);
	}
	else if (msg->getArrivalGateId() == eastIn)
	{
		WiredMessage *wiredMsg = dynamic_cast<WiredMessage*>(msg);
		handleEastMsg(wiredMsg);
		DELETE_SAFELY(wiredMsg);
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
	case BaseRSUMsgKinds::EXAMINE_VEHICLES_EVT:
	{
		examineVehicles();
		scheduleAt(simTime() + examineVehiclesInterval, examineVehiclesEvt);
		break;
	}
	case BaseRSUMsgKinds::FORGET_MEMORY_EVT:
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
		DYNAMIC_CAST_CMESSAGE(Beacon, beacon)
	else if (strcmp(msg->getName(), "warning") == 0)
		EV << "RSU doesn't relay warning message received from vehicles.\n";
	//else
	//	EV << "unknown message (" << msg->getName() << ") received.\n";
}

void BaseRSU::prepareWSM(WaveShortMessage *wsm, int dataLength, t_channel channel, int priority, LAddress::L2Type recipient)
{
	ASSERT(wsm != nullptr);
	ASSERT(channel == type_CCH || channel == type_SCH);

	wsm->setBitLength(headerLength+dataLength);

	WAVEInformationElement channelNumber(15, 1, channel == type_CCH ? Channels::CCH : Channels::SCH1);
	WAVEInformationElement dataRate(16, 1, 12);
	WAVEInformationElement transmitPowerUsed(4, 1, 30);
	WAVEInformationElement channelLoad(23, 1, 0);
	wsm->setChannelNumber(channelNumber); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
	wsm->setDataRate(dataRate);
	wsm->setTransmitPowerUsed(transmitPowerUsed);
	wsm->setChannelLoad(channelLoad);

	wsm->setPriority(priority);
	wsm->setSenderAddress(myAddr);
	wsm->setRecipientAddress(recipient);
}

void BaseRSU::sendWSM(WaveShortMessage *wsm)
{
	sendDelayedDown(wsm, individualOffset);
}

void BaseRSU::onBeacon(BeaconMessage *beaconMsg)
{
	EV << logName() << ": onBeacon!\n";

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
	EV << "    senderPos: " << beaconMsg->getSenderPos() << ", senderSpeed: " << beaconMsg->getSenderSpeed() << "\n";
	EV << "display all vehicles' information as follows:\n";
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		EV << "vehicle[" << itV->first << "]:  pos:" << itV->second->pos << ", speed:" << itV->second->speed << "\n";
	EV << std::endl;
#endif
}

void BaseRSU::examineVehicles()
{
	simtime_t curTime = simTime(); // alias
	for (itV = vehicles.begin(); itV != vehicles.end();)
	{
		if ( curTime - itV->second->receivedAt > vehicleElapsed )
		{
			EV << logName() << " disconnected from vehicle[" << itV->first << "], delete its info.\n";
			/* derived class's extension write here before it is deleted */
			delete itV->second;
			vehicles.erase(itV++);
		}
		else
			++itV;
	}
}

void BaseRSU::forgetMemory()
{
	simtime_t curTime = simTime(); // alias
	for (std::map<int, simtime_t>::iterator iter = messageMemory.begin(); iter != messageMemory.end();)
	{
		if (curTime - iter->second > memoryElapsed)
		{
			EV << logName() << " forgets message(GUID=" << iter->first << "), delete it from message memory.\n";
			messageMemory.erase(iter++);
		}
		else
			++iter;
	}
}

