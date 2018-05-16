//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>, 2016 Xu Le <xmutongxinXuLe@163.com>
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
		laneId = mobility->getLaneId();
		EV << "laneId start at: " << laneId << std::endl;
		traciLane = mobility->getLaneCommandInterface(laneId);
		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(getParentModule());
		ASSERT(myMac);
		annotations = Veins::AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		std::list<Coord> roadheads = traciLane->getShape();
		ASSERT( roadheads.size() == 2 ); // must be non-internal lane
		fromRoadhead = roadheads.front();
		toRoadhead = roadheads.back();
#if ROUTING_DEBUG_LOG
		EV << "fromRoadhead.x: " << fromRoadhead.x << ", fromRoadhead.y: " << fromRoadhead.y << ", fromRoadhead.z: " << fromRoadhead.z << ".\n";
		EV << "toRoadhead.x: " << toRoadhead.x << ", toRoadhead.y: " << toRoadhead.y << ", toRoadhead.z: " << toRoadhead.z << ".\n";
#endif
		MobilityObserver::Instance()->insert(myAddr, Coord::ZERO, Coord::ZERO);

		transmissionRadius = BaseConnectionManager::maxInterferenceDistance;
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();
		sendBeacons = par("sendBeacons").boolValue();
		callRoutings = par("callRoutings").boolValue();
		callWarnings = par("callWarnings").boolValue();
		dataOnSch = par("dataOnSch").boolValue();

		beaconLengthBits = par("beaconLengthBits").longValue();
		beaconPriority = par("beaconPriority").longValue();
		routingLengthBits = par("routingLengthBits").longValue();
		routingPriority = par("routingPriority").longValue();
		warningLengthBits = par("warningLengthBits").longValue();
		warningPriority = par("warningPriority").longValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataPriority = par("dataPriority").longValue();
		maxHopConstraint = par("maxHopConstraint").longValue();

		neighborElapsed = par("neighborElapsed").doubleValue();
		memoryElapsed = par("memoryElapsed").doubleValue();
		beaconInterval = par("beaconInterval").doubleValue();
		examineNeighborsInterval = par("examineNeighborsInterval").doubleValue();
		forgetMemoryInterval = par("forgetMemoryInterval").doubleValue();
		maxStoreTime = par("maxStoreTime").doubleValue();
		guidUsedTime = par("guidUsedTime").doubleValue();

		findHost()->subscribe(mobilityStateChangedSignal, this);
		findHost()->subscribe(parkingStateChangedSignal, this);

		if ( callRoutings )
			initializeRoutingPlanList(par("routingPlan").xmlValue());
		if ( callWarnings )
			initializeWarningPlanList(par("warningPlan").xmlValue());

		// simulate asynchronous channel access
		double offSet = static_cast<double>(myAddr%100)/100.0 * (beaconInterval/2);
		offSet = offSet + floor(offSet/0.050)*0.050;

		if ( sendBeacons )
		{
			sendBeaconEvt = new cMessage("beacon evt", WaveApplMsgKinds::SEND_BEACON_EVT);
			examineNeighborsEvt = new cMessage("examine neighbors evt", WaveApplMsgKinds::EXAMINE_NEIGHBORS_EVT);
			forgetMemoryEvt = new cMessage("forget memory evt", WaveApplMsgKinds::FORGET_MEMORY_EVT); // derived classes schedule it
			recycleGUIDEvt = new cMessage("recycle guid evt", WaveApplMsgKinds::RECYCLE_GUID_EVT);
			scheduleAt(simTime() + offSet, sendBeaconEvt);
			scheduleAt(simTime() + dblrand()*examineNeighborsInterval, examineNeighborsEvt);
			if ( !routingPlanList.empty() )
			{
				callRoutingEvt = new cMessage("call routing evt", WaveApplMsgKinds::CALL_ROUTING_EVT);
				scheduleAt(routingPlanList.front().first, callRoutingEvt);
			}
			else
				callRoutingEvt = nullptr;
			if ( !warningPlanList.empty() )
			{
				callWarningEvt = new cMessage("call warning evt", WaveApplMsgKinds::CALL_WARNING_EVT);
				scheduleAt(warningPlanList.front().first, callWarningEvt);
			}
			else
				callWarningEvt = nullptr;
		}
		else
		{
			sendBeaconEvt = nullptr;
			examineNeighborsEvt = nullptr;
			callRoutingEvt = nullptr;
			callWarningEvt = nullptr;
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
	routingPlanList.clear();
	warningPlanList.clear();
	for (std::map<LAddress::L3Type, NeighborInfo*>::iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter)
		delete iter->second;
	neighbors.clear();
	for (std::map<int, WaveShortMessage*>::iterator iter = messageMemory.begin(); iter != messageMemory.end(); ++iter)
		delete iter->second;
	messageMemory.clear();

	// delete handle self message
	cancelAndDelete(sendBeaconEvt);
	cancelAndDelete(examineNeighborsEvt);
	cancelAndDelete(callRoutingEvt);
	cancelAndDelete(callWarningEvt);
	cancelAndDelete(forgetMemoryEvt);
	cancelAndDelete(recycleGUIDEvt);
	for (std::map<simtime_t, PacketExpiredMessage*>::iterator iter = packetExpiresEvts.begin(); iter != packetExpiresEvts.end(); ++iter)
		cancelAndDelete(iter->second);
	packetExpiresEvts.clear();

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
	}
	else if (msg->getArrivalGateId() == lowerControlIn)
	{
		recordPacket(PassedMessage::INCOMING, PassedMessage::LOWER_CONTROL, msg);
		handleLowerControl(msg);
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

void BaseWaveApplLayer::handleSelfMsg(cMessage* msg)
{
	switch (msg->getKind())
	{
	case WaveApplMsgKinds::SEND_BEACON_EVT:
	{
		sendBeacon();
		scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
		break;
	}
	case WaveApplMsgKinds::EXAMINE_NEIGHBORS_EVT:
	{
		examineNeighbors();
		scheduleAt(simTime() + examineNeighborsInterval, examineNeighborsEvt);
		break;
	}
	case WaveApplMsgKinds::CALL_ROUTING_EVT:
	{
		callRouting(routingPlanList.front().second);
		routingPlanList.pop_front();
		if (!routingPlanList.empty())
			scheduleAt(routingPlanList.front().first, callRoutingEvt);
		break;
	}
	case WaveApplMsgKinds::CALL_WARNING_EVT:
	{
		callWarning(warningPlanList.front().second);
		warningPlanList.pop_front();
		if (!warningPlanList.empty())
			scheduleAt(warningPlanList.front().first, callWarningEvt);
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
		RoutingUtils::recycleGUID(guidUsed.front());
		guidUsed.pop_front();
		break;
	}
	case WaveApplMsgKinds::PACKET_EXPIRES_EVT:
	{
		simtime_t curTime(simTime());
		curTime -= maxStoreTime;
		std::map<simtime_t, PacketExpiredMessage*>::iterator itExpiredPacket = packetExpiresEvts.find(curTime);
		EV << "packet(GUID=" << itExpiredPacket->second->getGUID() << ") expired, discard it.\n";
		/* derived class's extension write here before it is deleted */

		// delete this packet expired message and erase it from packetExpiresEvts
		delete itExpiredPacket->second;
		packetExpiresEvts.erase(itExpiredPacket);
		break;
	}
	default:
	{
		EV_WARN << "Warning: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
	}
	}
}

void BaseWaveApplLayer::handleLowerMsg(cMessage* msg)
{
	if (strcmp(msg->getName(), "routing") == 0)
	{
		RoutingMessage *routingMessage = dynamic_cast<RoutingMessage*>(msg);
		ASSERT(routingMessage);
		onRouting(routingMessage);
	}
	else if (strcmp(msg->getName(), "warning") == 0)
	{
		WarningMessage *warningMessage = dynamic_cast<WarningMessage*>(msg);
		ASSERT(warningMessage);
		onWarning(warningMessage);
	}
	else
	{
		if (strcmp(msg->getName(), "beacon") == 0)
		{
			BeaconMessage *beaconMessage = dynamic_cast<BeaconMessage*>(msg);
			onBeacon(beaconMessage);
		}
		else if (strcmp(msg->getName(), "data") == 0)
		{
			DataMessage *dataMessage = dynamic_cast<DataMessage*>(msg);
			ASSERT(dataMessage);
			onData(dataMessage);
		}
		else
			EV_WARN << "unknown message (" << msg->getName() << ") received.\n";
		delete msg;
		msg = nullptr;
	}
}

WaveShortMessage* BaseWaveApplLayer::prepareWSM(std::string name, int dataLength, t_channel channel, int priority, int serial)
{
	ASSERT( channel == type_CCH || channel == type_SCH );

	WaveShortMessage* wsm = nullptr;
	if (name == "beacon")
	{
		EV << "Creating Beacon with Priority " << priority << " at BaseWaveApplLayer at " << simTime() << std::endl;
		wsm = new BeaconMessage("beacon");
	}
	else if (name == "routing")
	{
		EV << "Creating Routing with Priority " << priority << " at BaseWaveApplLayer at " << simTime() << std::endl;
		wsm = new RoutingMessage("routing");
	}
	else if (name == "warning")
	{
		EV << "Creating Warning with Priority " << priority << " at BaseWaveApplLayer at " << simTime() << std::endl;
		wsm = new WarningMessage("warning");
	}
	else if (name == "data")
	{
		EV << "Creating Data with Priority " << priority << " at BaseWaveApplLayer at " << simTime() << std::endl;
		wsm = new DataMessage("data");
	}
	else
	{
		EV << "Undefined name, don't create it." << std::endl;
		return wsm;
	}

	wsm->addBitLength(headerLength);
	if (name != "data")
		wsm->addBitLength(dataLength);

	if (channel == type_CCH)
		wsm->setChannelNumber(Channels::CCH);
	else // channel == type_SCH
		wsm->setChannelNumber(Channels::SCH1); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed

	wsm->setTimestamp();
	wsm->setPriority(priority);
	wsm->setSerial(serial);
	wsm->setSenderAddress(myAddr);
	wsm->setSenderAngle(curAngle);
	wsm->setSenderPos(curPosition);
	wsm->setSenderSpeed(curSpeed);

	return wsm;
}

void BaseWaveApplLayer::decorateWSM(WaveShortMessage* wsm)
{
	if (strcmp(wsm->getName(), "beacon") == 0)
	{
		BeaconMessage *beaconMessage = dynamic_cast<BeaconMessage*>(wsm);
		beaconMessage->setSenderFrom(fromRoadhead);
		beaconMessage->setSenderTo(toRoadhead);
		// If subclass using adaptive beaconing, here maybe execute some codes
	}
	else if (strcmp(wsm->getName(), "routing") == 0)
	{
		// Determined by concrete unicast routing protocols
	}
	else if (strcmp(wsm->getName(), "warning") == 0)
	{
		// Determined by concrete broadcast or geocast routing protocols
	}
	else if (strcmp(wsm->getName(), "data") == 0)
	{
		// Determined by concrete applications
	}
	else
	{
		EV << "unknown message (" << wsm->getName() << ") decorated, delete it\n";
		delete wsm;
		wsm = nullptr;
	}
}

void BaseWaveApplLayer::sendWSM(WaveShortMessage* wsm)
{
	if ( !isParking || sendWhileParking )
		sendDelayedDown(wsm, individualOffset);
	else
	{
		EV << "this vehicle doesn't send WaveShortMessage since it is parking now.\n";
		delete wsm;
		wsm = nullptr;
	}
}

void BaseWaveApplLayer::onBeacon(BeaconMessage* beaconMsg)
{
	EV << "node[" << myAddr << "]: " << "onBeacon!\n";

	LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias
	NeighborInfo *neighborInfo = nullptr;
	if ( neighbors.find(sender) != neighbors.end() ) // update old record
	{
		EV << "    sender [" << sender << "] is an old neighbor, update its info.\n";
		neighborInfo = neighbors[sender]; // alias for efficiency
		neighborInfo->angle = beaconMsg->getSenderAngle();
		neighborInfo->pos = beaconMsg->getSenderPos();
		neighborInfo->speed = beaconMsg->getSenderSpeed();
		neighborInfo->from = beaconMsg->getSenderFrom();
		neighborInfo->to = beaconMsg->getSenderTo();
		neighborInfo->receivedAt = simTime();
	}
	else // insert new record
	{
		EV << "    sender [" << sender << "] is a new neighbor, insert its info.\n";
		neighborInfo = new NeighborInfo(beaconMsg->getSenderAngle(), beaconMsg->getSenderPos(), beaconMsg->getSenderSpeed(), beaconMsg->getSenderFrom(), beaconMsg->getSenderTo(), simTime());
		neighbors.insert(std::pair<LAddress::L3Type, NeighborInfo*>(sender, neighborInfo));
	}
	beaconMsg->removeControlInfo();

#if ROUTING_DEBUG_LOG
	EV << "    senderPos: " << beaconMsg->getSenderPos() << ", senderSpeed: " << beaconMsg->getSenderSpeed() << std::endl;
	EV << "display all neighbors' information of " << logName() << std::endl;
	for (std::map<LAddress::L3Type, NeighborInfo*>::iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter)
	{
		EV << "neighbor[" << iter->first << "]:  pos:" << iter->second->pos << ", speed:" << iter->second->speed << std::endl;
	}
#endif
}

void BaseWaveApplLayer::examineNeighbors()
{
	double curTime = simTime().dbl(); // alias
	for (std::map<LAddress::L3Type, NeighborInfo*>::iterator iter = neighbors.begin(); iter != neighbors.end();)
	{
		if ( curTime - iter->second->receivedAt.dbl() > neighborElapsed )
		{
			EV << logName() << " disconnected from neighbor[" << iter->first << "], delete its info.\n";
			/* derived class's extension write here before it is deleted */
			delete iter->second;
			neighbors.erase(iter++);
		}
		else
			++iter;
	}
}

void BaseWaveApplLayer::forgetMemory()
{
	double curTime = simTime().dbl(); // alias
	for (std::map<int, WaveShortMessage*>::iterator iter = messageMemory.begin(); iter != messageMemory.end();)
	{
		if ( curTime - iter->second->getTimestamp().dbl() > memoryElapsed )
		{
			EV << logName() << " forgets message(GUID=" << iter->first << "), delete it from message memory.\n";
			/* derived class's extension write here before it is deleted */
			delete iter->second;
			messageMemory.erase(iter++);
		}
		else
			++iter;
	}
}

void BaseWaveApplLayer::handleMobilityUpdate(cObject* obj)
{
	Veins::TraCIMobility* const mobility = check_and_cast<Veins::TraCIMobility*>(obj);
	curAngle = mobility->getAngleRad();
	curPosition = mobility->getCurrentPosition();
	curSpeed = mobility->getCurrentSpeed();

	if (laneId != mobility->getLaneId())
	{
		laneId = mobility->getLaneId();
		EV << "laneId changed to: " << laneId << std::endl;
		traciLane->setLaneId(laneId);
		std::list<Coord> roadheads = traciLane->getShape();
		ASSERT( roadheads.size() == 2 ); // must be non-internal lane
		fromRoadhead = roadheads.front();
		toRoadhead = roadheads.back();
		if ((curSpeed.x >= 0 && fromRoadhead.x > toRoadhead.x) || (curSpeed.x < 0 && fromRoadhead.x < toRoadhead.x))
			std::swap(fromRoadhead, toRoadhead);
#if ROUTING_DEBUG_LOG
		EV << "fromRoadhead.x: " << fromRoadhead.x << ", fromRoadhead.y: " << fromRoadhead.y << ", fromRoadhead.z: " << fromRoadhead.z << ".\n";
		EV << "toRoadhead.x: " << toRoadhead.x << ", toRoadhead.y: " << toRoadhead.y << ", toRoadhead.z: " << toRoadhead.z << ".\n";
#endif
		// check if the vehicle has turned driving direction
		switch (RoutingUtils::relativeDirection(oldAngle, curAngle))
		{
		case RoutingUtils::SAME:
			onStraight();
			break;
		case RoutingUtils::LEFT:
			onTurnLeft();
			break;
		case RoutingUtils::RIGHT:
			onTurnRight();
			break;
		case RoutingUtils::OPPOSITE:
			onTurnAround();
			break;
		}
	}
	if (!mobility->getAtIntersection())
		oldAngle = curAngle;

	MobilityObserver::Instance()->update(myAddr, curPosition, curSpeed);
}

void BaseWaveApplLayer::handleParkingUpdate(cObject* obj)
{
	isParking = mobility->getParkingState();

	if (sendWhileParking == false)
	{
		if (isParking == true)
		{
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		}
		else
		{
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}

void BaseWaveApplLayer::initializeRoutingPlanList(cXMLElement* xmlConfig)
{
	if (xmlConfig == 0)
		throw cRuntimeError("No routing plan configuration file specified.");

	cXMLElementList planList = xmlConfig->getElementsByTagName("RoutingPlan");

	if (planList.empty())
		EV << "No routing plan configuration items specified.\n";

	for (cXMLElementList::iterator iter = planList.begin(); iter != planList.end(); ++iter)
	{
		cXMLElement *routingPlan = *iter;

		const char* name = routingPlan->getAttribute("type");

		if (atoi(name) == myAddr)
		{
			cXMLElementList parameters = routingPlan->getElementsByTagName("parameter");

			ASSERT( parameters.size() % 2 == 0 );

			EV << logName() << "'s routing plan list as follows:\n";

			for (size_t i = 0; i < parameters.size(); i += 2)
			{
				double simtime = 0.0;
				long receiver = 0;

				const char *name = parameters[i]->getAttribute("name");
				const char *type = parameters[i]->getAttribute("type");
				const char *value = parameters[i]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value");

				if (strcmp(name, "simtime") == 0 && strcmp(type, "double") == 0)
					simtime = atof(value);

				name = parameters[i+1]->getAttribute("name");
				type = parameters[i+1]->getAttribute("type");
				value = parameters[i+1]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value");

				if (strcmp(name, "receiver") == 0 && strcmp(type, "long") == 0)
					receiver = atoi(value);

				EV << "    simtime: " << simtime << ", receiver: " << receiver << std::endl;
				routingPlanList.push_back(std::pair<double, LAddress::L3Type>(simtime, receiver));
			}

			break;
		}
	}
}

void BaseWaveApplLayer::initializeWarningPlanList(cXMLElement* xmlConfig)
{
	if (xmlConfig == 0)
		throw cRuntimeError("No warning plan configuration file specified.");

	cXMLElementList planList = xmlConfig->getElementsByTagName("WarningPlan");

	if (planList.empty())
		EV << "No warning plan configuration items specified.\n";

	for (cXMLElementList::iterator iter = planList.begin(); iter != planList.end(); ++iter)
	{
		cXMLElement *warningPlan = *iter;

		const char* name = warningPlan->getAttribute("type");

		if (atoi(name) == myAddr)
		{
			cXMLElementList parameters = warningPlan->getElementsByTagName("parameter");

			ASSERT( parameters.size() % 2 == 0 );

			EV << logName() << "'s warning plan list as follows:\n";

			for (size_t i = 0; i < parameters.size(); i += 2)
			{
				double simtime = 0.0;
				double distance = 0.0;

				const char *name = parameters[i]->getAttribute("name");
				const char *type = parameters[i]->getAttribute("type");
				const char *value = parameters[i]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value.");

				if (strcmp(name, "simtime") == 0 && strcmp(type, "double") == 0)
					simtime = atof(value);
				else
					throw cRuntimeError("Invalid parameter, name or type undefined.");

				name = parameters[i+1]->getAttribute("name");
				type = parameters[i+1]->getAttribute("type");
				value = parameters[i+1]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value.");

				if (strcmp(name, "distance") == 0 && strcmp(type, "double") == 0)
					distance = atof(value);
				else
					throw cRuntimeError("Invalid parameter, name or type undefined.");

				EV << "    simtime: " << simtime << ", distance: " << distance << std::endl;
				warningPlanList.push_back(std::pair<double, double>(simtime, distance));
			}

			break;
		}
	}
}

void BaseWaveApplLayer::sendBeacon()
{
	WaveShortMessage *wsm = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, -1);
	decorateWSM(wsm);
	sendWSM(wsm);
}

long BaseWaveApplLayer::targetVehicles(bool plusX, Coord xMin, Coord xMax, LAddress::L3Type& farthestOne)
{
	MobilityObserver *mobilityObserver = MobilityObserver::Instance();
	ASSERT( mobilityObserver->globalPosition.size() == mobilityObserver->globalSpeed.size() );

	long targetNumber = 0;
	double distToSourceX = 0.0;

	EV << "target vehicles are: ";

	std::map<LAddress::L3Type, Coord>::iterator iterPos, iterSpeed;
	for (iterPos = mobilityObserver->globalPosition.begin(), iterSpeed = mobilityObserver->globalSpeed.begin();
		 iterPos != mobilityObserver->globalPosition.end() && iterSpeed != mobilityObserver->globalSpeed.end();
		 ++iterPos, ++iterSpeed)
	{
		if ( (plusX && iterSpeed->second.x >= 0) || (!plusX && iterSpeed->second.x < 0) )
		{
			if ( iterPos->second.x >= xMin.x && iterPos->second.x <= xMax.x && RoutingUtils::distanceToLine(iterPos->second, xMin, xMax) < 14.0 )
			{
				if (plusX && distToSourceX < xMax.x - iterPos->second.x)
				{
					distToSourceX = xMax.x - iterPos->second.x;
					farthestOne = iterPos->first;
				}
				else if (!plusX && distToSourceX < iterPos->second.x - xMin.x)
				{
					distToSourceX = iterPos->second.x - xMin.x;
					farthestOne = iterPos->first;
				}
				EV << ' ' << iterPos->first;
				++targetNumber;
			}
		}
	}

	EV << ", the number of them is " << targetNumber << ", the farthest one is " << farthestOne << std::endl;

	return targetNumber;
}

BaseWaveApplLayer::~BaseWaveApplLayer()
{
	EV << "base wave appl layer module destructing ..." << std::endl;
}

