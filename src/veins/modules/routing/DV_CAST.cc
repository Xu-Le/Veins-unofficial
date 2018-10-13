//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/routing/DV_CAST.h"

Define_Module(DV_CAST);

extern std::map<int /* GUID */, LAddress::L3Type> warningMaxDelayHelper;

void DV_CAST::initialize(int stage)
{
	BaseWaveApplLayer::initialize(stage);

	if (stage == 0)
	{
		MDC = false;
		ODC = false;

		WAIT_TIME = par("WAIT_TIME").doubleValue();
		warningLengthBits = par("warningLengthBits").longValue();
		warningPriority = par("warningPriority").longValue();
		callWarnings = par("callWarnings").boolValue();
		if (callWarnings)
			initializeWarningPlanList(par("warningPlan").xmlValue());

		if (sendBeacons && !warningPlanList.empty())
		{
			callWarningEvt = new cMessage("call warning evt", WaveApplMsgKinds::CALL_WARNING_EVT);
			scheduleAt(warningPlanList.front().first, callWarningEvt);
		}
		else
			callWarningEvt = nullptr;
	}
}

void DV_CAST::finish()
{
	// delete handle self message, it is safe to delete nullptr
	for (std::map<simtime_t, WaitTimeElapsedMessage*>::iterator iter = waitTimeElapsedEvts.begin(); iter != waitTimeElapsedEvts.end(); ++iter)
	{
		if (iter->second->isScheduled())
			cancelAndDelete(iter->second);
		else
			delete iter->second;
	}
	waitTimeElapsedEvts.clear();

	// clear containers
	NB_FRONT.clear();
	NB_BACK.clear();
	NB_OPPOSITE.clear();
	curState.clear();
	DFlg.clear();
	warningPlanList.clear();

	cancelAndDelete(callWarningEvt);

	BaseWaveApplLayer::finish();
}

void DV_CAST::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case WaveApplMsgKinds::CALL_WARNING_EVT:
	{
		callWarning(warningPlanList.front().second);
		warningPlanList.pop_front();
		if (!warningPlanList.empty())
			scheduleAt(warningPlanList.front().first, callWarningEvt);
		break;
	}
	case WaveApplMsgKinds::PACKET_EXPIRES_EVT:
	{
		simtime_t curTime(simTime());
		curTime -= maxStoreTime;
		std::map<simtime_t, PacketExpiredMessage*>::iterator itExpiredPacket = packetExpiresEvts.find(curTime);
		int guid = itExpiredPacket->second->getGUID(); // alias
		EV << "packet(GUID=" << guid << ") expired, discard it.\n";
		curState.erase(guid);
		DFlg.erase(guid);
		delete itExpiredPacket->second;
		packetExpiresEvts.erase(itExpiredPacket);
		break;
	}
	case WaveApplMsgKinds::WAIT_TIME_ELAPSED_EVT:
	{
		simtime_t curTime(simTime());
		curTime -= WAIT_TIME;
		std::map<simtime_t, WaitTimeElapsedMessage*>::iterator itElapsedPacket = waitTimeElapsedEvts.find(curTime);
		int guid = itElapsedPacket->second->getGUID(); // alias
#if USE_WEIGHTED_P_PERSISTENCE
		if ( curState[guid] == State::BroadcastSuppression1 )
		{
			EV << "packet(GUID=" << guid << ") 's wait time elapsed, its current state in BroadcastSuppression1.\n";
			double p = itElapsedPacket->second->getProbability(); // alias
			if ( dblrand() < p )
			{
				EV << "dblrand() < p, where p is " << p << ", so decide to rebroadcast.\n";
				WarningMessage *warningMsg = dynamic_cast<WarningMessage*>(messageMemory[guid]);
				rebroadcast(warningMsg);
				curState.erase(guid); // not to handle this packet anymore
				DFlg.erase(guid);
				delete itElapsedPacket->second;
				waitTimeElapsedEvts.erase(itElapsedPacket);
			}
			else // If decides not to rebroadcast, then should buffer the message for an additional WAIT_TIME ms
			{
				EV << "dblrand() >= p, where p is " << p << ", so decide not to rebroadcast, turn state into BroadcastSuppression2.\n";
				curState[guid] = State::BroadcastSuppression2;
				WaitTimeElapsedMessage *waitTimeElapsedEvt = new WaitTimeElapsedMessage("wait time elapsed evt", WaveApplMsgKinds::WAIT_TIME_ELAPSED_EVT);
				waitTimeElapsedEvt->setGUID(guid);
				waitTimeElapsedEvt->setProbability(p);
				// delete old record in the map first
				delete itElapsedPacket->second;
				waitTimeElapsedEvts.erase(itElapsedPacket);
				// insert new record in the map next
				waitTimeElapsedEvts.insert(std::pair<simtime_t, WaitTimeElapsedMessage*>(simTime(), waitTimeElapsedEvt));
				scheduleAt(simTime() + WAIT_TIME, waitTimeElapsedEvt);
			}
		}
		else if ( curState[guid] == State::BroadcastSuppression2 )
		{
			EV << "packet(GUID=" << guid << ") 's wait time elapsed, its current state in BroadcastSuppression2.\n";
			// In order to prevent message die out and guarantee 100 percent reachability,
			// then should rebroadcast the message with probability 1 after WAIT_TIME ms
			// if it does not hear the retransmission from its neighbors.
			if ( !itElapsedPacket->second->getRecvDup2() )
			{
				EV << "found that I hasn't received duplicate packets in state BroadcastSuppression2, so decide to rebroadcast.\n";
				WarningMessage *warningMsg = dynamic_cast<WarningMessage*>(messageMemory[guid]);
				rebroadcast(warningMsg);
			}
			else
				EV << "found that I has received duplicate packets in state BroadcastSuppression2, so decide not to rebroadcast.\n";
			curState.erase(guid); // not to handle this packet anymore
			DFlg.erase(guid);
			delete itElapsedPacket->second;
			waitTimeElapsedEvts.erase(itElapsedPacket);
		}
		else
			error("Error: state is not in BroadcastSuppression");
#endif
		break;
	}
	default:
		BaseWaveApplLayer::handleSelfMsg(msg);
	}
}

void DV_CAST::handleLowerMsg(cMessage *msg)
{
	if (strcmp(msg->getName(), "warning") == 0)
		DYNAMIC_CAST_CMESSAGE(Warning, warning)

	BaseWaveApplLayer::handleLowerMsg(msg);
}

void DV_CAST::onBeacon(BeaconMessage *beaconMsg)
{
	LAddress::L3Type senderAddr = beaconMsg->getSenderAddress(); // alias
	bool isOld = neighbors.find(senderAddr) != neighbors.end();

	BaseWaveApplLayer::onBeacon(beaconMsg);

	// update neighbor sets
	Coord &senderPos = beaconMsg->getSenderPos();
	Coord &senderSpeed = beaconMsg->getSenderSpeed();
	bool xPositiveAxis = curSpeed.x > 0;
	bool isOpposite = curSpeed.x * senderSpeed.x < 0;
	bool ODN = false;

	if (isOpposite)
	{
		if ( isOld )
		{
			EV << "because sender is an old opposite neighbor, doesn't push it into NB_OPPOSITE.\n";
			return;
		}

		ODN = true;
		EV << "sender: node[" << beaconMsg->getSenderAddress() << "] is a new opposite neighbor, push it into NB_OPPOSITE.\n";
		NB_OPPOSITE.insert(senderAddr);
	}
	else
	{
		if ( isOld )
		{
			bool isFront = NB_FRONT.find(senderAddr) != NB_FRONT.end();
			if (isFront)
				EV << "because sender is an old front neighbor, check if relative position has changed.\n";
			else
				EV << "because sender is an old back neighbor, check if relative position has changed.\n";
			if ( (xPositiveAxis && senderPos.x > curPosition.x) || (!xPositiveAxis && senderPos.x < curPosition.x) )
			{
				if (isFront)
				{
					EV << "sender: node[" << senderAddr << "] still is a front neighbor, doesn't insert it into NB_FRONT.\n";
					return;
				}
				else
				{
					EV << "sender: node[" << senderAddr << "] changes to be a front neighbor, insert it into NB_FRONT and erase it from NB_BACK.\n";
					NB_FRONT.insert(senderAddr);
					NB_BACK.erase(senderAddr);
				}
			}
			else // ( (xPositiveAxis && senderPos.x < curPosition.x) || (!xPositiveAxis && senderPos.x > curPosition.x) )
			{
				if (!isFront)
				{
					EV << "sender: node[" << senderAddr << "] still is a back neighbor, doesn't insert it into NB_BACK.\n";
					return;
				}
				else
				{
					EV << "sender: node[" << senderAddr << "] changes to be a back neighbor, insert it into NB_BACK and erase it from NB_FRONT.\n";
					NB_BACK.insert(senderAddr);
					NB_FRONT.erase(senderAddr);
				}
			}
		}
		else
		{
			if ( (xPositiveAxis && senderPos.x > curPosition.x) || (!xPositiveAxis && senderPos.x < curPosition.x) )
			{
				EV << "sender: node[" << senderAddr << "] is a new front neighbor, insert it into NB_FRONT.\n";
				NB_FRONT.insert(senderAddr);
			}
			else // ( (xPositiveAxis && senderPos.x < curPosition.x) || (!xPositiveAxis && senderPos.x > curPosition.x) )
			{
				EV << "sender: node[" << senderAddr << "] is a new back neighbor, insert it into NB_BACK.\n";
				NB_BACK.insert(senderAddr);
			}
		}
	}

	// update bool flags
	MDC = !NB_FRONT.empty() && !NB_BACK.empty();
	ODC = !NB_OPPOSITE.empty();
	EV << "MDC = " << MDC << ", ODC = " << ODC << std::endl;

	// if a message is waiting for rebroadcast, check whether the condition is meet current now
	for (std::map<int, int>::iterator iter = curState.begin(); iter != curState.end();)
	{
		if ( iter->second == (int)State::WAIT_I && ODN && (ODC || MDC) )
		{
			WarningMessage *warningMsg = dynamic_cast<WarningMessage*>(messageMemory[iter->first]);
			rebroadcast(warningMsg);
			EV << "message(GUID=" << warningMsg->getGUID() << ") in state WAIT_I, now rebroadcast it.\n";
			DFlg.erase(iter->first);
			curState.erase(iter++);
			for (std::map<simtime_t, PacketExpiredMessage*>::iterator itCancel = packetExpiresEvts.begin(); itCancel != packetExpiresEvts.end(); ++itCancel)
			{
				if (itCancel->second->getGUID() == warningMsg->getGUID())
				{
					EV << "cancel packet expired message(GUID=" << warningMsg->getGUID() << ").\n";
					cancelAndDelete(itCancel->second);
					packetExpiresEvts.erase(itCancel);
					break;
				}
			}
		}
		else if ( iter->second == (int)State::WAIT_II && ODN )
		{
			WarningMessage *warningMsg = dynamic_cast<WarningMessage*>(messageMemory[iter->first]);
			rebroadcast(warningMsg);
			EV << "message(GUID=" << warningMsg->getGUID() << ") in state WAIT_II, now rebroadcast it.\n";
			if ( DFlg[warningMsg->getGUID()] )
			{
				DFlg.erase(iter->first);
				curState.erase(iter++);
				for (std::map<simtime_t, PacketExpiredMessage*>::iterator itCancel = packetExpiresEvts.begin(); itCancel != packetExpiresEvts.end(); ++itCancel)
				{
					if (itCancel->second->getGUID() == warningMsg->getGUID())
					{
						EV << "cancel packet expired message(GUID=" << warningMsg->getGUID() << ").\n";
						cancelAndDelete(itCancel->second);
						packetExpiresEvts.erase(itCancel);
						break;
					}
				}
			}
			else
			{
				EV << "since DFlg == false, still in state WAIT_II.\n";
				iter->second = (int)State::WAIT_II;
				++iter;
			}
		}
		else
			++iter;
	}
}

void DV_CAST::onWarning(WarningMessage *_warningMsg)
{
	EV << "node[" << myAddr << "]: onWarning!\n";

	// what to operate is a duplication of warningMsg, because it will be deleted in BaseWaveApplLayer::handleLowerMsg()
	WarningMessage *warningMsg = new WarningMessage(*_warningMsg);

	int guid = warningMsg->getGUID(); // alias

	// check if the message has been received recently
	if ( messageMemory.find(guid) != messageMemory.end() )
	{
		EV << "message(GUID=" << guid << ") has been received recently, discard it.\n";
#if NOTIFY_WHEN_ARRIVED_AT_THE_FARTHEST_ONE
		if ( !warningMsg->getWarningSuccess() )
		{
#endif
			++WarningStatisticCollector::globalDuplications;
			if ( curState.find(guid) == curState.end() )
			{
				DELETE_SAFELY(warningMsg);
				return;
			}
#if USE_WEIGHTED_P_PERSISTENCE
			int state = curState[guid]; // alias
			// check if this packet state is broadcast suppression
			if ( state == State::BroadcastSuppression1 )
			{
				for (std::map<simtime_t, WaitTimeElapsedMessage*>::iterator iter = waitTimeElapsedEvts.begin(); iter != waitTimeElapsedEvts.end(); ++iter)
				{
					if (iter->second->getGUID() == guid)
					{
						iter->second->setRecvDup1(true);
						// Note that if receives duplicate packets from multiple sources within the waiting period of WAIT_TIME
						// before retransmission, then selects the smallest p_ij value as our reforwarding probability; that is,
						// each node should use the relative distance to the nearest broadcaster in order to ensure that nodes
						// who are farther away transmit with higher probability.
						double newP, oldP = iter->second->getProbability();
						HopItems &hopInfo = warningMsg->getHopInfo();
						HopItem &hopItem = hopInfo.back();
						double posX = hopItem.posX;
						double posY = hopItem.posY;
						double posZ = hopItem.posZ;
						EV << "found that I am in state BroadcastSuppression1, check if I need to update rebroadcast probability.\n";
						// double speedX = hopItem.speedX;
						// double speedY = hopItem.speedY;
						// double speedZ = hopItem.speedZ;
						double relativeDist = sqrt((curPosition.x - posX)*(curPosition.x - posX) + (curPosition.y - posY)*(curPosition.y - posY) + (curPosition.z - posZ)*(curPosition.z - posZ));
						newP = relativeDist / transmissionRadius;
						if (newP < oldP)
						{
							EV << "newP: " << newP << ", oldP: " << oldP << ", so I need to update rebroadcast probability.\n";
							iter->second->setProbability(newP);
						}
						else
							EV << "newP: " << newP << ", oldP: " << oldP << ", so I don't need to update rebroadcast probability.\n";
						break;
					}
				}
			}
			else if ( state == State::BroadcastSuppression2 )
			{
				for (std::map<simtime_t, WaitTimeElapsedMessage*>::iterator iter = waitTimeElapsedEvts.begin(); iter != waitTimeElapsedEvts.end(); ++iter)
				{
					if (iter->second->getGUID() == guid)
					{
						iter->second->setRecvDup2(true);
						break;
					}
				}
			}
#endif
#if NOTIFY_WHEN_ARRIVED_AT_THE_FARTHEST_ONE
		}
		else
		{
			EV << "notified that this message has arrived at destination, check if I have stored it before.\n";
			if ( curState.find(guid) == curState.end() )
			{
				DELETE_SAFELY(warningMsg);
				return;
			}
			int state = curState[guid]; // alias
			if ( state == State::WAIT_I || state == State::WAIT_II )
			{
				for (std::map<simtime_t, PacketExpiredMessage*>::iterator itCancel = packetExpiresEvts.begin(); itCancel != packetExpiresEvts.end(); ++itCancel)
				{
					if (itCancel->second->getGUID() == guid)
					{
						EV << "found that I have stored it before, cancel its expire event and delete its state record in curState.\n";
						cancelAndDelete(itCancel->second);
						packetExpiresEvts.erase(itCancel);
						break;
					}
				}
				curState.erase(guid); // avoid to rebroadcast it again when coming across opposite vehicle
				DFlg.erase(guid);
			}
#if USE_WEIGHTED_P_PERSISTENCE
			else if ( state == State::BroadcastSuppression1 || state == State::BroadcastSuppression2 )
			{
				for (std::map<simtime_t, WaitTimeElapsedMessage*>::iterator itCancel = waitTimeElapsedEvts.begin(); itCancel != waitTimeElapsedEvts.end(); ++itCancel)
				{
					if (itCancel->second->getGUID() == guid)
					{
						EV << "found that I have stored it before, cancel its wait time elapsed event and delete its state record in curState.\n";
						cancelAndDelete(itCancel->second);
						waitTimeElapsedEvts.erase(itCancel);
						break;
					}
				}
				curState.erase(guid); // not to handle this packet anymore
				DFlg.erase(guid);
			}
#endif
			else
				EV << "found that I haven't stored it before, do nothing.\n";
		}
#endif
		DELETE_SAFELY(warningMsg);
		return;
	}

	// check if the message has disseminated outside of ROI
	Coord &senderPos = warningMsg->getSenderPos();
	double relativeDist = sqrt(square(curPosition.x - senderPos.x) + square(curPosition.y - senderPos.y) + square(curPosition.z - senderPos.z));
	EV << "senderPosX: " << senderPos.x << ", senderPosY: " << senderPos.y << ", senderPosZ: " << senderPos.z << ", relativeDist: " << relativeDist << std::endl;
	bool isOutside = relativeDist > warningMsg->getFarthestDistance();
	if ( isOutside )
	{
		EV << "message(GUID=" << guid << ") has disseminated outside its ROI.\n";
		messageMemory.insert(std::pair<int, WaveShortMessage*>(guid, warningMsg));

		++WarningStatisticCollector::globalInterferences;
		return;
	}
	if ( myAddr == warningMaxDelayHelper[guid] )
	{
		messageMemory.insert(std::pair<int, WaveShortMessage*>(guid, warningMsg));
		// collect statistics
		++WarningStatisticCollector::globalPenetrations;
		double timeDifferentia = simTime().dbl() - warningMsg->getTimestamp().dbl();
		WarningStatisticCollector::globalMaxDelayAccumulation += timeDifferentia;
		EV << "message(GUID=" << guid << ") arrived at the farthest vehicle, which experienced " << warningMsg->getHopCount() << " hop count, the delay time is " << timeDifferentia << "s.\n";
#if NOTIFY_WHEN_ARRIVED_AT_THE_FARTHEST_ONE
		// notify neighbors who are in broadcast suppression state not to rebroadcast since this warning message accomplished its duty.
		// what to send is a duplication of received warningMsg, because it will be encapsulated into Mac80211Pkt as well as be delete when this message memory forgets,
		// however if send the received warningMsg, it will cannot be deleted when memory forgets.
		WaveShortMessage *dupWSM = new WaveShortMessage(*warningMsg);
		warningMsg->setWarningSuccess(true);
		sendwarningMsg(dupWSM);
#endif
		return;
	}
	if (laneId != warningMsg->getLaneId())
	{
		EV << "message(GUID=" << guid << ") has disseminated outside its ROI(disseminated to another road).\n";
		messageMemory.insert(std::pair<int, WaveShortMessage*>(guid, warningMsg));

		++WarningStatisticCollector::globalInterferences;
		return;
	}

	// catch a new warning message
	EV << "catch a new warning message(GUID=" << guid << ").\n";

	bool thisDFlg = false;
	if ( warningMsg->getDirection() )
		thisDFlg = (warningMsg->getSenderSpeed().x >= 0 && curSpeed.x <= 0 && warningMsg->getSenderPos().x < curPosition.x) || (warningMsg->getSenderSpeed().x < 0 && curSpeed.x > 0 && warningMsg->getSenderPos().x > curPosition.x) ? true : false;
	else
		thisDFlg = (warningMsg->getSenderSpeed().x >= 0 && curSpeed.x >= 0 && warningMsg->getSenderPos().x > curPosition.x) || (warningMsg->getSenderSpeed().x < 0 && curSpeed.x < 0 && warningMsg->getSenderPos().x < curPosition.x) ? true : false;
	if (thisDFlg)
	{
		EV << "I belong to target vehicles since DFlg == true.\n";
		++WarningStatisticCollector::globalPenetrations;
	}
	else
		EV << "I don't belong to target vehicles since DFlg == false.\n";
	DFlg.insert(std::pair<int, bool>(guid, thisDFlg));
	messageMemory.insert(std::pair<int, WaveShortMessage*>(guid, warningMsg));
	curState.insert(std::pair<int, int>(guid, (int)State::IDLE));
	if (MDC == true)
	{
		EV << "MDC == true, state turn into BroadcastSuppression.\n";
		curState[guid] = State::BroadcastSuppression1;
		broadcastSuppression(warningMsg);
	}
	else if (ODC == true)
	{
		EV << "MDC == false && ODC == true, state turn into Rebroadcast.\n";
		curState[guid] = State::Rebroadcast;
		rebroadcast(warningMsg);
		if (thisDFlg)
			EV << "after rebroadcast, since DFlg == true, state turn into IDLE.\n";
		else
			EV << "after rebroadcast, since DFlg == false, state turn into WAIT_II.\n";
		curState[guid] = (thisDFlg == true) ? State::IDLE : State::WAIT_II;
	}
	else if (ODC == false)
	{
		EV << "MDC == false && ODC == false, state turn into WAIT_I.\n";
		curState[guid] = State::WAIT_I;
		PacketExpiredMessage *packetExpiresEvt = new PacketExpiredMessage("packet expires evt", WaveApplMsgKinds::PACKET_EXPIRES_EVT);
		packetExpiresEvt->setGUID(guid);
		packetExpiresEvts.insert(std::pair<simtime_t, PacketExpiredMessage*>(simTime(), packetExpiresEvt));
		scheduleAt(simTime() + maxStoreTime, packetExpiresEvt);
	}
}

void DV_CAST::examineNeighbors()
{
	double curTime = simTime().dbl(); // alias
	for (itN = neighbors.begin(); itN != neighbors.end();)
	{
		if ( curTime - itN->second->receivedAt.dbl() > neighborElapsed )
		{
			EV << logName() << " disconnected from neighbor[" << itN->first << "], delete its info\n";
			// update neighbor sets
			Coord &senderPos = itN->second->pos;
			Coord &senderSpeed = itN->second->speed;
			bool xPositiveAxis = curSpeed.x > 0;
			bool isOpposite = curSpeed.x * senderSpeed.x < 0;
			if (isOpposite)
			{
				EV << "disconnected neighbor is an opposite neighbor, erase it from NB_OPPOSITE.\n";
				NB_OPPOSITE.erase(itN->first);
			}
			else
			{
				if ( (xPositiveAxis && senderPos.x > curPosition.x) || (!xPositiveAxis && senderPos.x < curPosition.x) )
				{
					EV << "disconnected neighbor is a front neighbor, erase it from NB_FRONT.\n";
					NB_FRONT.erase(itN->first);
				}
				else // ( (xPositiveAxis && senderPos.x < curPosition.x) || (!xPositiveAxis && senderPos.x > curPosition.x) )
				{
					EV << "disconnected neighbor is a back neighbor, erase it from NB_BACK.\n";
					NB_BACK.erase(itN->first);
				}
			}
			// update bool flags
			MDC = !NB_FRONT.empty() && !NB_BACK.empty();
			ODC = !NB_OPPOSITE.empty();
			EV << "MDC = " << MDC << ", ODC = " << ODC << std::endl;
			// delete this neighbor's info
			delete itN->second;
			neighbors.erase(itN++);
		}
		else
			++itN;
	}
}

void DV_CAST::decorateWarning(WarningMessage *warningMsg)
{
	int hopCount = warningMsg->getHopCount();
	warningMsg->setHopCount(hopCount + 1);

	HopItems &hopInfo = warningMsg->getHopInfo();
	HopItem hopItem(myAddr, curPosition.x, curPosition.y, curPosition.z, curSpeed.x, curSpeed.y, curSpeed.z);
	hopInfo.push_back(hopItem);
}

void DV_CAST::callWarning(double distance)
{
	// create message by factory method
	WarningMessage *warningMsg = new WarningMessage("warning");
	prepareWSM(warningMsg, warningLengthBits, type_CCH, warningPriority, -1);

	// handle utils about message's GUID
	int guid = RoutingUtils::generateGUID();
	EV << "GUID = " << guid << ", distance = " << distance << std::endl;
	guidUsed.push_back(guid);
	scheduleAt(simTime() + guidUsedTime, recycleGUIDEvt);
	// set necessary warning info
	warningMsg->setGUID(guid);
	if (distance > 0)
	{
		warningMsg->setDirection(false);
		warningMsg->setLaneId(laneId.c_str());
	}
	else
	{
		warningMsg->setDirection(true);
		std::string tmpLaneId("-");
		if (laneId[0] == '-') // remove front '-'
		{
			tmpLaneId = laneId.substr(1, laneId.size() - 1);
			warningMsg->setLaneId(tmpLaneId.c_str());
		}
		else // add '-' to front
		{
			tmpLaneId.append(laneId);
			warningMsg->setLaneId(tmpLaneId.c_str());
		}
	}
	double farthestDistance = fabs(distance); // the farthest target vehicle for warning message in ROI
	warningMsg->setFarthestDistance(farthestDistance);
	warningMsg->setHopCount(0);
	// insert into message memory to avoid relaying the message that is rebroadcast back to self by others
	messageMemory.insert(std::pair<int, WaveShortMessage*>(guid, warningMsg));
	// what to send is a duplication of warningMsg, because it will be encapsulated into Mac80211Pkt as well as be delete when this message memory forgets,
	// however if send the warningMsg, it will cannot be deleted when memory forgets.
	WarningMessage *dupWSM = new WarningMessage(*warningMsg);
	decorateWarning(dupWSM);
	sendWSM(dupWSM);

	++WarningStatisticCollector::globalNotifications;

	// calculate the number of target vehicles and the farthest one in ROI
	bool plusX = (curSpeed.x >= 0 && distance > 0) || (curSpeed.x < 0 && distance < 0);
	Coord xMin, xMax;
	LAddress::L3Type farthestOne = -1;
	if (distance > 0)
	{
		double distFromRoadhead = sqrt(square(curPosition.x - fromRoadhead.x) + square(curPosition.y - fromRoadhead.y) + square(curPosition.z - fromRoadhead.z));
		if (distFromRoadhead <= farthestDistance)
		{
			EV << "distFromRoadhead = " << distFromRoadhead << " <= " << farthestDistance << std::endl;
			if (fromRoadhead.x <= toRoadhead.x) // curSpeed.x >= 0
			{
				xMin = fromRoadhead;
				xMax = curPosition;
				xMax.x -= 1.0; // avoid count self vehicle
			}
			else // curSpeed.x < 0
			{
				xMin = curPosition;
				xMin.x += 1.0; // avoid count self vehicle
				xMax = fromRoadhead;
			}
		}
		else
		{
			EV << "distFromRoadhead = " << distFromRoadhead << " > " << farthestDistance << std::endl;
			Coord farthestPos;
			farthestPos.x = curPosition.x + farthestDistance/distFromRoadhead * (fromRoadhead.x - curPosition.x);
			farthestPos.y = curPosition.y + farthestDistance/distFromRoadhead * (fromRoadhead.y - curPosition.y);
			farthestPos.z = curPosition.z + farthestDistance/distFromRoadhead * (fromRoadhead.z - curPosition.z);
			EV << "farthestPos.x: " << farthestPos.x << ", farthestPos.y: " << farthestPos.y << ", farthestPos.z: " << farthestPos.z << std::endl;
			if (fromRoadhead.x <= toRoadhead.x) // curSpeed.x >= 0
			{
				xMin = farthestPos;
				xMax = curPosition;
				xMax.x -= 1.0; // avoid count self vehicle
			}
			else // curSpeed.x < 0
			{
				xMin = curPosition;
				xMin.x += 1.0; // avoid count self vehicle
				xMax = farthestPos;
			}
		}
	}
	else
	{
		double distToRoadhead = sqrt(square(curPosition.x - toRoadhead.x) + square(curPosition.y - toRoadhead.y) + square(curPosition.z - toRoadhead.z));
		if (distToRoadhead <= farthestDistance)
		{
			EV << "distToRoadhead = " << distToRoadhead << " <= " << farthestDistance << std::endl;
			if (fromRoadhead.x <= toRoadhead.x) // curSpeed.x >= 0
			{
				xMin = curPosition;
				xMin.x += 1.0; // avoid count self vehicle
				xMax = toRoadhead;
			}
			else // curSpeed.x < 0
			{
				xMin = toRoadhead;
				xMax = curPosition;
				xMax.x -= 1.0; // avoid count self vehicle
			}
		}
		else
		{
			EV << "distToRoadhead = " << distToRoadhead << " > " << farthestDistance << std::endl;
			Coord farthestPos;
			farthestPos.x = curPosition.x + farthestDistance/distToRoadhead * (toRoadhead.x - curPosition.x);
			farthestPos.y = curPosition.y + farthestDistance/distToRoadhead * (toRoadhead.y - curPosition.y);
			farthestPos.z = curPosition.z + farthestDistance/distToRoadhead * (toRoadhead.z - curPosition.z);
			EV << "farthestPos.x: " << farthestPos.x << ", farthestPos.y: " << farthestPos.y << ", farthestPos.z: " << farthestPos.z << std::endl;
			if (fromRoadhead.x <= toRoadhead.x) // curSpeed.x >= 0
			{
				xMin = curPosition;
				xMin.x += 1.0; // avoid count self vehicle
				xMax = farthestPos;
			}
			else // curSpeed.x < 0
			{
				xMin = farthestPos;
				xMax = curPosition;
				xMax.x -= 1.0; // avoid count self vehicle
			}
		}
	}

	WarningStatisticCollector::globalTargets += targetVehicles(plusX, xMin, xMax, farthestOne);

	warningMaxDelayHelper.insert(std::pair<int, LAddress::L3Type>(guid, farthestOne));

	// curState.insert(std::pair<int, int>(guid, State::IDLE));
}

void DV_CAST::initializeWarningPlanList(cXMLElement *xmlConfig)
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

long DV_CAST::targetVehicles(bool plusX, Coord xMin, Coord xMax, LAddress::L3Type& farthestOne)
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

void DV_CAST::broadcastSuppression(WarningMessage *warningMsg)
{
#if USE_WEIGHTED_P_PERSISTENCE
	/**
	 * Weighted p-Persistence Broadcasting Rule.
	 *
	 * Upon receiving a packet from node i, node j checks the packet ID and rebroadcasts with probability
	 * p_ij if it receives the packet for the first time; otherwise, it discards the packet.
	 */
	double p;
	HopItems &hopInfo = warningMsg->getHopInfo();
	HopItem &hopItem = hopInfo.back();
	double posX = hopItem.posX;
	double posY = hopItem.posY;
	double posZ = hopItem.posZ;
	EV << "posX: " << posX << ", posY: " << posY << ", posZ: " << posZ << std::endl;
	// double speedX = hopItem.speedX;
	// double speedY = hopItem.speedY;
	// double speedZ = hopItem.speedZ;
	double relativeDist = sqrt(square(curPosition.x - posX) + square(curPosition.y - posY) + square(curPosition.z - posZ));
	p = relativeDist / transmissionRadius;
	EV << "relative distance: " << relativeDist << ", p: " << p << std::endl;
	ASSERT( p > 0.0 && p < 1.0 );
	WaitTimeElapsedMessage *waitTimeElapsedEvt = new WaitTimeElapsedMessage("wait time elapsed evt", WaveApplMsgKinds::WAIT_TIME_ELAPSED_EVT);
	waitTimeElapsedEvt->setGUID(warningMsg->getGUID());
	waitTimeElapsedEvt->setProbability(p);
	waitTimeElapsedEvts.insert(std::pair<simtime_t, WaitTimeElapsedMessage*>(simTime(), waitTimeElapsedEvt));
	scheduleAt(simTime() + WAIT_TIME, waitTimeElapsedEvt);
#endif
#if USE_SLOTTED_1_PERSISTENCE
	/**
	 * Slotted 1-Persistence Broadcasting Rule.
	 *
	 * Upon receiving a packet, a node checks the packet ID and rebroadcasts with probability 1 at
	 * the assigned time slot T_{S_ij} if it receives the packet for the first time and has not
	 * received any duplicates before its assigned time slot; otherwise, it discards the packet.
	 */
	rebroadcast(warningMsg);
#endif
#if USE_SLOTTED_P_PERSISTENCE
	/**
	 * Slotted p-Persistence Broadcasting Rule.
	 *
	 * Upon receiving a packet, a node checks the packet ID and rebroadcasts with the pre-determined
	 * probability p at the assigned time slot T_{S_ij}, as expressed by Eq. 2, if it receives the
	 * packet for the first time and has not received any duplicates before its assigned time slot;
	 * otherwise, it discards the packet.
	 */
	rebroadcast(warningMsg);
#endif
}

void DV_CAST::rebroadcast(WarningMessage *warningMsg)
{
	// what to send is a duplication of received warningMsg, because it will be encapsulated into Mac80211Pkt as well as be delete when this message memory forgets,
	// however if send the received warningMsg, it will cannot be deleted when memory forgets.
	WarningMessage *dupWSM = new WarningMessage(*warningMsg);
	decorateWarning(dupWSM);
	sendWSM(dupWSM);
}

DV_CAST::~DV_CAST()
{

}

