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

		maxHopConstraint = par("maxHopConstraint").longValue();
		whichSide = par("whichSide").longValue();

		examineVehiclesInterval = par("examineVehiclesInterval").doubleValue();
		forgetMemoryInterval = par("forgetMemoryInterval").doubleValue();
		vehicleElapsed = par("vehicleElapsed").doubleValue();
		memoryElapsed = par("memoryElapsed").doubleValue();

		examineVehiclesEvt = new cMessage("examine vehicles evt", RSUMessageKinds::EXAMINE_VEHICLES_EVT);
		forgetMemoryEvt = new cMessage("forget memory evt", RSUMessageKinds::FORGET_MEMORY_EVT); // derived classes schedule it
#ifndef ATTAIN_VEHICLE_DENSITY_BY_GOD_VIEW
		scheduleAt(simTime() + dblrand()*examineVehiclesInterval, examineVehiclesEvt);
#endif
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
		DELETE_SAFELY(msg);
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
		DYNAMIC_CAST_CMESSAGE(Beacon, beacon)
	else if (strcmp(msg->getName(), "warning") == 0)
		EV << "RSU doesn't relay warning message received from vehicles.\n";
	else
		EV << "unknown message (" << msg->getName() << ") received.\n";
}

void BaseRSU::prepareWSM(WaveShortMessage *wsm, int dataLength, t_channel channel, int priority, int serial)
{
	ASSERT(wsm != nullptr);
	ASSERT(channel == type_CCH || channel == type_SCH);

	wsm->addBitLength(headerLength);
	wsm->addBitLength(dataLength);

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
	EV << "    senderPos: " << beaconMsg->getSenderPos() << ", senderSpeed: " << beaconMsg->getSenderSpeed() << std::endl;
	EV << "display all vehicles' information of " << logName() << std::endl;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		EV << "vehicle[" << itV->first << "]:  pos:" << itV->second->pos << ", speed:" << itV->second->speed << std::endl;
#endif
}

void BaseRSU::examineVehicles()
{
	double curTime = simTime().dbl(); // alias
	for (itV = vehicles.begin(); itV != vehicles.end();)
	{
		if ( curTime - itV->second->receivedAt.dbl() > vehicleElapsed )
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

