//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>, 2016-2018 Xu Le <xmutongxinXuLe@163.com>
//
// Documentation for these modules is at http://veins.car2x.org/
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

#include "veins/modules/wave/BaseWaveApplLayer.h"

const simsignalwrap_t BaseWaveApplLayer::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);
const simsignalwrap_t BaseWaveApplLayer::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

void BaseWaveApplLayer::initialize(int stage)
{
	BaseApplLayer::initialize(stage);

	if (stage == 0)
	{
		cellularIn = findGate("cellularIn");

		mobility = Veins::TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(getParentModule());
		ASSERT(myMac);
		annotations = Veins::AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		MobilityObserver::Instance()->insert(myAddr, Coord::ZERO, Coord::ZERO);

		transmissionRadius = BaseConnectionManager::maxInterferenceDistance;

		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();
		sendBeacons = par("sendBeacons").boolValue();
		dataOnSch = par("dataOnSch").boolValue();

		beaconLengthBits = par("beaconLengthBits").longValue();
		beaconPriority = par("beaconPriority").longValue();

		neighborElapsed = par("neighborElapsed").doubleValue();
		memoryElapsed = par("memoryElapsed").doubleValue();
		beaconInterval = par("beaconInterval").doubleValue();
		examineNeighborsInterval = par("examineNeighborsInterval").doubleValue();
		forgetMemoryInterval = par("forgetMemoryInterval").doubleValue();
		maxStoreTime = par("maxStoreTime").doubleValue();
		guidUsedTime = par("guidUsedTime").doubleValue();

		findHost()->subscribe(mobilityStateChangedSignal, this);
		findHost()->subscribe(parkingStateChangedSignal, this);

		if (sendBeacons)
		{
			sendBeaconEvt = new cMessage("beacon evt", WaveApplMsgKinds::SEND_BEACON_EVT);
			examineNeighborsEvt = new cMessage("examine neighbors evt", WaveApplMsgKinds::EXAMINE_NEIGHBORS_EVT);
			forgetMemoryEvt = new cMessage("forget memory evt", WaveApplMsgKinds::FORGET_MEMORY_EVT); // derived classes schedule it
			recycleGUIDEvt = new cMessage("recycle guid evt", WaveApplMsgKinds::RECYCLE_GUID_EVT);
			simtime_t beaconOffset = dblrand() * beaconInterval;
			simtime_t examineOffset = dblrand() * examineNeighborsInterval;
#if ROUTING_DEBUG_LOG
			nextBeaconInstant = beaconOffset.dbl();
			nextExamineInstant = examineOffset.dbl();
			WATCH(nextBeaconInstant);
			WATCH(nextExamineInstant);
#endif
			scheduleAt(simTime() + beaconOffset, sendBeaconEvt);
			scheduleAt(simTime() + examineOffset, examineNeighborsEvt);
		}
		else
		{
			sendBeaconEvt = nullptr;
			examineNeighborsEvt = nullptr;
			forgetMemoryEvt = nullptr;
			recycleGUIDEvt = nullptr;
		}
	}
}

void BaseWaveApplLayer::finish()
{
	EV << "base wave appl layer module finishing ..." << std::endl;

	MobilityObserver::Instance()->erase(myAddr);

	// clear containers and their stored elements
	guidUsed.clear();
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		delete itN->second;
	neighbors.clear();
	for (std::map<int, WaveShortMessage*>::iterator iter = messageMemory.begin(); iter != messageMemory.end(); ++iter)
		delete iter->second;
	messageMemory.clear();

	// delete handle self message
	cancelAndDelete(sendBeaconEvt);
	cancelAndDelete(examineNeighborsEvt);
	cancelAndDelete(forgetMemoryEvt);
	cancelAndDelete(recycleGUIDEvt);

	findHost()->unsubscribe(mobilityStateChangedSignal, this);
	findHost()->unsubscribe(parkingStateChangedSignal, this);

	BaseLayer::finish();
}

void BaseWaveApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
	Enter_Method_Silent();

	if (signalID == mobilityStateChangedSignal)
	{
		handleMobilityUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal)
	{
		handleParkingUpdate(obj);
	}
}

/////////////////////////    protected implementations    /////////////////////////
void BaseWaveApplLayer::handleMessage(cMessage *msg)
{
	if (msg->isSelfMessage())
		handleSelfMsg(msg);
	else if (msg->getArrivalGateId() == lowerLayerIn)
	{
		recordPacket(PassedMessage::INCOMING, PassedMessage::LOWER_DATA, msg);
		handleLowerMsg(msg);
		DELETE_SAFELY(msg);
	}
	else if (msg->getArrivalGateId() == lowerControlIn)
	{
		recordPacket(PassedMessage::INCOMING, PassedMessage::LOWER_CONTROL, msg);
		handleLowerControl(msg);
		DELETE_SAFELY(msg);
	}
	else if (msg->getArrivalGateId() == cellularIn)
	{
		CellularMessage *cellularMsg = dynamic_cast<CellularMessage*>(msg);
		handleCellularMsg(cellularMsg);
	}
	else if (msg->getArrivalGateId() == -1)
		throw cRuntimeError("No self message and no gateID! Check configuration.");
	else
		throw cRuntimeError("Unknown gateID! Check configuration or override handleMessage().");
}

void BaseWaveApplLayer::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case WaveApplMsgKinds::SEND_BEACON_EVT:
	{
		sendBeacon();
#if ROUTING_DEBUG_LOG
		nextBeaconInstant = simTime().dbl() + beaconInterval.dbl();
#endif
		scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
		break;
	}
	case WaveApplMsgKinds::EXAMINE_NEIGHBORS_EVT:
	{
		examineNeighbors();
#if ROUTING_DEBUG_LOG
		nextExamineInstant = simTime().dbl() + examineNeighborsInterval.dbl();
#endif
		scheduleAt(simTime() + examineNeighborsInterval, examineNeighborsEvt);
		break;
	}
	case WaveApplMsgKinds::FORGET_MEMORY_EVT:
	{
		forgetMemory();
		scheduleAt(simTime() + forgetMemoryInterval, forgetMemoryEvt);
		break;
	}
	case WaveApplMsgKinds::RECYCLE_GUID_EVT:
	{
		WaveUtils::recycleGUID(guidUsed.front());
		guidUsed.pop_front();
		break;
	}
	default:
	{
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
	}
	}
}

void BaseWaveApplLayer::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "beacon") == 0)
		DYNAMIC_CAST_CMESSAGE(Beacon, beacon)
	//else
	//	EV_WARN << "unknown message (" << msg->getName() << ") received.\n";
}

void BaseWaveApplLayer::prepareWSM(WaveShortMessage *wsm, int dataLength, t_channel channel, int priority, LAddress::L2Type recipient)
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

void BaseWaveApplLayer::sendWSM(WaveShortMessage *wsm)
{
	if ( !isParking || sendWhileParking )
		sendDelayedDown(wsm, individualOffset);
	else
	{
		EV << "this vehicle doesn't send WaveShortMessage since it is parking now.\n";
		DELETE_SAFELY(wsm);
	}
}

void BaseWaveApplLayer::sendBeacon()
{
	EV << "Creating Beacon with Priority " << beaconPriority << " at BaseWaveApplLayer at " << simTime() << std::endl;
	BeaconMessage *beaconMsg = new BeaconMessage("beacon");
	prepareWSM(beaconMsg, beaconLengthBits, type_CCH, beaconPriority, -1);
	beaconMsg->setSenderPos(curPosition);
	beaconMsg->setSenderSpeed(curSpeed);
	decorateBeacon(beaconMsg);
	sendWSM(beaconMsg);
}

void BaseWaveApplLayer::onBeacon(BeaconMessage *beaconMsg)
{
	EV << "node[" << myAddr << "]: onBeacon!\n";

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias
	NeighborInfo *neighborInfo = nullptr;
	if ( neighbors.find(sender) != neighbors.end() ) // update old record
	{
		EV << "    sender [" << sender << "] is an old neighbor, update its info.\n";
		neighborInfo = neighbors[sender]; // alias for efficiency
		neighborInfo->pos = beaconMsg->getSenderPos();
		neighborInfo->speed = beaconMsg->getSenderSpeed();
		neighborInfo->receivedAt = simTime();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new neighbor, insert its info.\n";
		neighborInfo = new NeighborInfo(beaconMsg->getSenderPos(), beaconMsg->getSenderSpeed(), simTime());
		neighbors.insert(std::pair<LAddress::L3Type, NeighborInfo*>(sender, neighborInfo));
	}

#if ROUTING_DEBUG_LOG
	EV << "    senderPos: " << beaconMsg->getSenderPos() << ", senderSpeed: " << beaconMsg->getSenderSpeed() << "\n";
	EV << "display all neighbors' information as follows:\n";
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		EV << "neighbor[" << itN->first << "]:  pos:" << itN->second->pos << ", speed:" << itN->second->speed << "\n";
	EV << std::endl;
#endif
}

void BaseWaveApplLayer::examineNeighbors()
{
	simtime_t curTime = simTime(); // alias
	for (itN = neighbors.begin(); itN != neighbors.end();)
	{
		if ( curTime - itN->second->receivedAt > neighborElapsed )
		{
			EV << "disconnected from neighbor[" << itN->first << "], delete its info.\n";
			onNeighborLost();
			delete itN->second;
			neighbors.erase(itN++);
		}
		else
			++itN;
	}
}

void BaseWaveApplLayer::forgetMemory()
{
	simtime_t curTime = simTime(); // alias
	for (std::map<int, WaveShortMessage*>::iterator iter = messageMemory.begin(); iter != messageMemory.end();)
	{
		if ( curTime - iter->second->getTimestamp() > memoryElapsed )
		{
			EV << "forgets message(GUID=" << iter->first << "), delete it from message memory.\n";
			/* derived class's extension write here before it is deleted */
			delete iter->second;
			messageMemory.erase(iter++);
		}
		else
			++iter;
	}
}

void BaseWaveApplLayer::handleMobilityUpdate(cObject *obj)
{
	curPosition = mobility->getCurrentPosition();
	curSpeed = mobility->getCurrentSpeed();

	MobilityObserver::Instance()->update(myAddr, curPosition, curSpeed);
}

void BaseWaveApplLayer::handleParkingUpdate(cObject *obj)
{
	isParking = mobility->getParkingState();

	if (!sendWhileParking)
	{
		if (isParking)
		{
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(getParentModule()->getSubmodule("nic"));
		}
		else
		{
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(getParentModule()->getSubmodule("nic"), (ChannelAccess *)getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}

BaseWaveApplLayer::~BaseWaveApplLayer()
{
	EV << "base wave appl layer module destructing ..." << std::endl;
}

