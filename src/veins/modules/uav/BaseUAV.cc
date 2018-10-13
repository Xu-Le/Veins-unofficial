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

#include "veins/modules/uav/BaseUAV.h"

const simsignalwrap_t BaseUAV::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void BaseUAV::initialize(int stage)
{
	BaseApplLayer::initialize(stage);

	if (stage == 0)
	{
		myAddr += UAV_ADDRESS_OFFSET;

		mobility = FindModule<AircraftMobility*>::findSubModule(getParentModule());
		ASSERT(mobility);
		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(getParentModule());
		ASSERT(myMac);
		annotations = Veins::AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		MobilityObserver::Instance2()->insert(myAddr, Coord::ZERO, Coord::ZERO);

		V2XRadius = BaseConnectionManager::maxInterferenceDistance;
		U2URadius = BaseConnectionManager::maxInterferenceDistance2;

		dataOnSch = par("dataOnSch").boolValue();

		beaconLengthBits = par("beaconLengthBits").longValue();
		beaconPriority = par("beaconPriority").longValue();

		beaconInterval = par("beaconInterval").doubleValue();
		examineVehiclesInterval = par("examineVehiclesInterval").doubleValue();
		examineNeighborsInterval = par("examineNeighborsInterval").doubleValue();
		vehicleElapsed = par("vehicleElapsed").doubleValue();
		neighborElapsed = par("neighborElapsed").doubleValue();

		findHost()->subscribe(mobilityStateChangedSignal, this);

		sendUavBeaconEvt = new cMessage("send uav beacon evt", UAVMessageKinds::SEND_UAV_BEACON_EVT);
		examineVehiclesEvt = new cMessage("examine vehicles evt", UAVMessageKinds::EXAMINE_VEHICLES_EVT);
		examineNeighborsEvt = new cMessage("examine neighbors evt", UAVMessageKinds::EXAMINE_NEIGHBORS_EVT);
		scheduleAt(simTime() + dblrand()*beaconInterval, sendUavBeaconEvt);
		scheduleAt(simTime() + dblrand()*examineVehiclesInterval, examineVehiclesEvt);
		scheduleAt(simTime() + dblrand()*examineNeighborsInterval, examineNeighborsEvt);
	}
}

void BaseUAV::finish()
{
	EV << "base UAV module finish ..." << std::endl;

	MobilityObserver::Instance2()->erase(myAddr);

	// clear containers
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		delete itV->second;
	vehicles.clear();
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		delete itN->second;
	neighbors.clear();

	// delete handle self message
	cancelAndDelete(sendUavBeaconEvt);
	cancelAndDelete(examineVehiclesEvt);
	cancelAndDelete(examineNeighborsEvt);

	findHost()->unsubscribe(mobilityStateChangedSignal, this);

	BaseLayer::finish();
}

void BaseUAV::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
	Enter_Method_Silent();

	if (signalID == mobilityStateChangedSignal)
		handleMobilityUpdate(obj);
}

/////////////////////////    protected implementations    /////////////////////////
void BaseUAV::handleMessage(cMessage *msg)
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
	else if (msg->getArrivalGateId() == -1)
		throw cRuntimeError("No self message and no gateID! Check configuration.");
	else
		throw cRuntimeError("Unknown gateID! Check configuration or override handleMessage().");
}

void BaseUAV::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case UAVMessageKinds::SEND_UAV_BEACON_EVT:
	{
		sendUavBeacon();
		scheduleAt(simTime() + beaconInterval, sendUavBeaconEvt);
		break;
	}
	case UAVMessageKinds::EXAMINE_VEHICLES_EVT:
	{
		examineVehicles();
		scheduleAt(simTime() + examineVehiclesInterval, examineVehiclesEvt);
		break;
	}
	case UAVMessageKinds::EXAMINE_NEIGHBORS_EVT:
	{
		examineNeighbors();
		scheduleAt(simTime() + examineNeighborsInterval, examineNeighborsEvt);
		break;
	}
	default:
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
	}
}

void BaseUAV::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "beacon") == 0)
		DYNAMIC_CAST_CMESSAGE(Beacon, beacon)
	else if (strcmp(msg->getName(), "uavBeacon") == 0)
		DYNAMIC_CAST_CMESSAGE(UavBeacon, uavBeacon)
	else
		EV_WARN << "unknown message (" << msg->getName() << ") received.\n";
}

void BaseUAV::handleMobilityUpdate(cObject *obj)
{
	mobility->getMove(curPosition, curSpeed);
	MobilityObserver::Instance2()->update(myAddr, curPosition, curSpeed);
	EV << "position: " << curPosition.info() << ", speed: " << curSpeed << std::endl;
}

void BaseUAV::prepareWSM(WaveShortMessage *wsm, int dataLength, t_channel channel, int priority, int serial)
{
	ASSERT(wsm != nullptr);
	ASSERT(channel == type_CCH || channel == type_SCH);

	wsm->addBitLength(headerLength);
	wsm->addBitLength(dataLength);

	if (channel == type_CCH)
		wsm->setChannelNumber(Channels::CCH);
	else // channel == type_SCH
		wsm->setChannelNumber(Channels::SCH1); // will be rewritten at Mac1609_4 to actual Service Channel.

	wsm->setPriority(priority);
	wsm->setSerial(serial);
	wsm->setSenderAddress(myAddr);
	wsm->setSenderPos(curPosition);
	wsm->setSenderSpeed(curSpeed);
	wsm->setTimestamp(simTime());
}

void BaseUAV::sendWSM(WaveShortMessage *wsm)
{
	sendDelayedDown(wsm, individualOffset);
}

void BaseUAV::sendUavBeacon()
{
	EV << "Creating UAV Beacon with Priority " << beaconPriority << " at BaseUAV at " << simTime() << std::endl;
	UavBeaconMessage *uavBeaconMsg = new UavBeaconMessage("uavBeacon");
	prepareWSM(uavBeaconMsg, beaconLengthBits, t_channel::type_CCH, beaconPriority, -1);
	decorateUavBeacon(uavBeaconMsg);
	sendWSM(uavBeaconMsg);
}

void BaseUAV::onBeacon(BeaconMessage *beaconMsg)
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

void BaseUAV::onUavBeacon(UavBeaconMessage *uavBeaconMsg)
{
	EV << logName() << ": onUavBeacon!\n";

	LAddress::L3Type sender = uavBeaconMsg->getSenderAddress(); // alias
	NeighborInfo *neighborInfo = nullptr;
	if ((itN = neighbors.find(sender)) != neighbors.end()) // update old record
	{
		EV << "    sender [" << sender << "] is an old neighbor, update its info.\n";
		neighborInfo = itN->second;
		neighborInfo->pos = uavBeaconMsg->getSenderPos();
		neighborInfo->speed = uavBeaconMsg->getSenderSpeed();
		neighborInfo->receivedAt = simTime();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new neighbor, insert its info.\n";
		neighborInfo = new NeighborInfo(uavBeaconMsg->getSenderPos(), uavBeaconMsg->getSenderSpeed(), simTime());
		neighbors.insert(std::pair<LAddress::L3Type, NeighborInfo*>(sender, neighborInfo));
	}
}

void BaseUAV::examineVehicles()
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

void BaseUAV::examineNeighbors()
{
	double curTime = simTime().dbl(); // alias
	for (itN = neighbors.begin(); itN != neighbors.end();)
	{
		if ( curTime - itN->second->receivedAt.dbl() > neighborElapsed )
		{
			EV << logName() << " disconnected from neighbor[" << itN->first << "], delete its info.\n";
			/* derived class's extension write here before it is deleted */
			delete itN->second;
			neighbors.erase(itN++);
		}
		else
			++itN;
	}
}
