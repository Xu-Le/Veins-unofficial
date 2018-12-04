//
// Copyright (C) 2012 David Eckhoff <eckhoff@cs.fau.de>
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

#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/messages/PhyControlMessage_m.h"

#define DBG_MAC if (false) EV_INFO

#define NEXT_MAC_EVT_MSG_KIND                 4
#define NEXT_CHANNEL_SWITCH_MSG_KIND          5
#define STOP_IGNORE_CHANNEL_STATE_MSG_KIND    6

Define_Module(Mac1609_4);

void Mac1609_4::initialize(int stage)
{
	BaseMacLayer::initialize(stage);

	if (stage == 0)
	{
		phy11p = FindModule<Mac80211pToPhy11pInterface*>::findSubModule(getParentModule());
		ASSERT(phy11p);

		// this is required to circumvent double precision issues with constants from CONST80211p.h
		ASSERT(simTime().getScaleExp() == -12);

		sigChannelBusy = registerSignal("sigChannelBusy");
		sigCollision = registerSignal("sigCollision");

		txPower = par("txPower").doubleValue();
		bitrate = par("bitrate").longValue();
		n_dbps = 0;
		setParametersForBitrate(bitrate);

		// unicast parameters
		lastMac = nullptr;
		lastWSM = nullptr;
		dot11RTSThreshold = par("dot11RTSThreshold").longValue();
		dot11ShortRetryLimit = par("dot11ShortRetryLimit").longValue();
		dot11LongRetryLimit = par("dot11LongRetryLimit").longValue();
		ackLength = par("ackLength").longValue();
		useAcks = par("useAcks").boolValue();
		ackErrorRate = par("ackErrorRate").doubleValue();
		rxStartIndication = false;
		ignoreChannelState = false;
		waitUntilAckRXorTimeout = false;
		stopIgnoreChannelStateMsg = new cMessage("ChannelStateMsg", STOP_IGNORE_CHANNEL_STATE_MSG_KIND);

		myId = getParentModule()->getParentModule()->getFullPath();
		// create frequency mappings
		frequency.insert(std::pair<int, double>(Channels::CRIT_SOL, 5.86e9));
		frequency.insert(std::pair<int, double>(Channels::SCH1, 5.87e9));
		frequency.insert(std::pair<int, double>(Channels::SCH2, 5.88e9));
		frequency.insert(std::pair<int, double>(Channels::CCH, 5.89e9));
		frequency.insert(std::pair<int, double>(Channels::SCH3, 5.90e9));
		frequency.insert(std::pair<int, double>(Channels::SCH4, 5.91e9));
		frequency.insert(std::pair<int, double>(Channels::HPPS, 5.92e9));

		// create two edca systems
		myEDCA.resize(2);
		myEDCA[type_CCH] = new EDCA(this, type_CCH,par("queueSize").longValue());
		myEDCA[type_CCH]->myId = myId;
		myEDCA[type_CCH]->myId.append(" CCH");

		myEDCA[type_CCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P+1)/2)-1),AC_VO);
		myEDCA[type_CCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
		myEDCA[type_CCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
		myEDCA[type_CCH]->createQueue(9,CWMIN_11P,(CWMAX_11P+1)/16-1,AC_BK);

		myEDCA[type_SCH] = new EDCA(this, type_SCH,par("queueSize").longValue());
		myEDCA[type_SCH]->myId = myId;
		myEDCA[type_SCH]->myId.append(" SCH");
		myEDCA[type_SCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
		myEDCA[type_SCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
		myEDCA[type_SCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
		myEDCA[type_SCH]->createQueue(9,CWMIN_11P,(CWMAX_11P+1)/16-1,AC_BK);

		useSCH = par("useServiceChannel").boolValue();
		if (useSCH)
		{
			if (useAcks)
				throw cRuntimeError("Unicast model does not support channel switching");
			// set the initial service channel
			switch (par("serviceChannel").longValue())
			{
			case 1:
				mySCH = Channels::SCH1;
				break;
			case 2:
				mySCH = Channels::SCH2;
				break;
			case 3:
				mySCH = Channels::SCH3;
				break;
			case 4:
				mySCH = Channels::SCH4;
				break;
			default:
				throw cRuntimeError("Service Channel must be between 1 and 4");
			}
		}

		headerLength = par("headerLength").longValue();

		nextMacEvent = new cMessage("next Mac Event", NEXT_MAC_EVT_MSG_KIND);

		if (useSCH)
		{
			// introduce a little asynchronization between radios, but no more than .3 milliseconds
			uint64_t currenTime = simTime().raw();
			uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
			double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();
			if ((currenTime / switchingTime) % 2 == 0)
				setActiveChannel(type_CCH);
			else
				setActiveChannel(type_SCH);

			// channel switching active
			nextChannelSwitch = new cMessage("Channel Switch", NEXT_CHANNEL_SWITCH_MSG_KIND);
			// add a little bit of offset between all vehicles, but no more than syncOffset
			simtime_t offset = dblrand() * par("syncOffset").doubleValue();
			scheduleAt(simTime() + offset + timeToNextSwitch, nextChannelSwitch);
		}
		else
		{
			// no channel switching
			nextChannelSwitch = 0;
			setActiveChannel(type_CCH);
		}

		// statistics
		statsReceivedPackets = 0;
		statsReceivedBroadcasts = 0;
		statsSentPackets = 0;
		statsSentAcks = 0;
		statsTXRXLostPackets = 0;
		statsSINRLostPackets = 0;
		statsSyncLostPackets = 0;
		statsCollisionLostPackets = 0;
		statsDroppedPackets = 0;
		statsNumTooLittleTime = 0;
		statsNumInternalContention = 0;
		statsNumBackoff = 0;
		statsSlotsBackoff = 0;
		statsTotalBusyTime = 0;

		idleChannel = true;
		lastBusy = simTime();
		channelIdle(true);
	}
}

void Mac1609_4::finish()
{
	statsNumInternalContention += myEDCA[type_CCH]->statsNumInternalContention;
	statsNumInternalContention += myEDCA[type_SCH]->statsNumInternalContention;
	statsNumBackoff += myEDCA[type_CCH]->statsNumBackoff;
	statsNumBackoff += myEDCA[type_SCH]->statsNumBackoff;
	statsSlotsBackoff += myEDCA[type_CCH]->statsSlotsBackoff;
	statsSlotsBackoff += myEDCA[type_SCH]->statsSlotsBackoff;
	myEDCA[type_CCH]->cleanUp();
	myEDCA[type_SCH]->cleanUp();
	delete myEDCA[type_CCH];
	delete myEDCA[type_SCH];
	myEDCA.clear();

	cancelAndDelete(nextMacEvent);
	cancelAndDelete(nextChannelSwitch);
	cancelAndDelete(stopIgnoreChannelStateMsg);

	//long totalLostPackets = statsTXRXLostPackets + statsSINRLostPackets + statsSyncLostPackets + statsCollisionLostPackets;
	recordScalar("RecvUnicastPackets", statsReceivedPackets);
	recordScalar("RecvBroadcasts", statsReceivedBroadcasts);
	recordScalar("SentPackets", statsSentPackets);
	recordScalar("SentAcknowledgements", statsSentAcks);
	recordScalar("RXTXLostPackets", statsTXRXLostPackets);
	recordScalar("SINRLostPackets", statsSINRLostPackets);
	recordScalar("SyncLostPackets", statsSyncLostPackets);
	//recordScalar("CollisionLostPackets", statsCollisionLostPackets);
	//recordScalar("TotalLostPackets", totalLostPackets);
	//recordScalar("DroppedPacketsInMac", statsDroppedPackets);
	//recordScalar("TooLittleTime", statsNumTooLittleTime);
	//recordScalar("TimesIntoBackoff", statsNumBackoff);
	//recordScalar("SlotsBackoff", statsSlotsBackoff);
	//recordScalar("NumInternalContention", statsNumInternalContention);
	//recordScalar("totalBusyTime", statsTotalBusyTime.dbl());
}

/* Will change the Service Channel on which the mac layer is listening and sending */
void Mac1609_4::changeServiceChannel(int cN)
{
	ASSERT(useSCH);
	if (cN != Channels::SCH1 && cN != Channels::SCH2 && cN != Channels::SCH3 && cN != Channels::SCH4)
		throw cRuntimeError("This Service Channel doesnt exit: %d", cN);

	mySCH = cN;

	if (activeChannel == type_SCH)
	{
		// change to new chan immediately if we are in a SCH slot,
		// otherwise it will switch to the new SCH upon next channel switch
		phy11p->changeListeningFrequency(frequency[mySCH]);
	}
}

void Mac1609_4::setTxPower(double txPower_mW)
{
	txPower = txPower_mW;
}

void Mac1609_4::setMCS(enum PHY_MCS mcs)
{
	ASSERT2(mcs != MCS_DEFAULT, "invalid MCS selected");
	bitrate = getOfdmDatarate(mcs, BW_OFDM_10_MHZ);
	setParametersForBitrate(bitrate);
}

void Mac1609_4::setCCAThreshold(double ccaThreshold_dBm)
{
	phy11p->setCCAThreshold(ccaThreshold_dBm);
}

void Mac1609_4::handleSelfMsg(cMessage *msg)
{
	if (msg->getKind() == NEXT_MAC_EVT_MSG_KIND)
	{
		// we actually came to the point where we can send a packet
		channelBusySelf(true);
		WaveShortMessage* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

		lastAC = mapUserPriority(pktToSend->getPriority());
		lastWSM = pktToSend;

		DBG_MAC << "MacEvent received. Trying to send packet with priority " << lastAC << "\n";

		// send the packet
		Mac80211Pkt* mac = new Mac80211Pkt(pktToSend->getName(), pktToSend->getKind());
		mac->setDestAddr(pktToSend->getRecipientAddress());
		mac->setSrcAddr(myMacAddr);
		mac->encapsulate(pktToSend->dup());

		enum PHY_MCS mcs = MCS_DEFAULT;
		double txPower_mW = txPower;
		uint64_t datarate = bitrate;
		PhyControlMessage *controlInfo = dynamic_cast<PhyControlMessage*>(pktToSend->getControlInfo());
		if (controlInfo)
		{
			// if MCS is not specified, just use the default one
			mcs = (enum PHY_MCS)controlInfo->getMcs();
			datarate = mcs != MCS_DEFAULT ? getOfdmDatarate(mcs, BW_OFDM_10_MHZ) : bitrate;
			// apply the same principle to tx power
			txPower_mW = controlInfo->getTxPower_mW();
			if (txPower_mW < 0)
				txPower_mW = txPower;
		}

		simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength(), mcs);
		DBG_MAC << "Sending duration will be " << sendingDuration << "\n";
		if (!useSCH || timeLeftInSlot() > sendingDuration)
		{
			if (useSCH)
			{
				DBG_MAC << "Time in this slot left: " << timeLeftInSlot() << std::endl;
			}

			double freq = (activeChannel == type_CCH) ? frequency[Channels::CCH] : frequency[mySCH];

			DBG_MAC << "Sending a Packet. Frequency " << freq << std::endl;
			sendFrame(mac, RADIODELAY_11P, freq, datarate, txPower_mW);

			// schedule ack timeout for unicast packets
			if (pktToSend->getRecipientAddress() != LAddress::L2BROADCAST() && useAcks)
			{
				waitUntilAckRXorTimeout = true;
				// PHY-RXSTART.indication should be received within ackWaitTime
				// sifs + slot + rx_delay: see 802.11-2012 9.3.2.8 (32us + 13us + 49us = 94us)
				simtime_t ackWaitTime(94, SIMTIME_US);
				// update id in the retransmit timer
				myEDCA[activeChannel]->myQueues[lastAC].ackTimeOut->setWsmId(pktToSend->getTreeId());
				simtime_t timeOut = sendingDuration + ackWaitTime;
				scheduleAt(simTime() + timeOut, myEDCA[activeChannel]->myQueues[lastAC].ackTimeOut);
			}
		}
		else // not enough time left now
		{
			DBG_MAC << "Too little Time left. This packet cannot be send in this slot." << std::endl;
			++statsNumTooLittleTime;
			// revoke TXOP
			myEDCA[activeChannel]->revokeTxOPs();
			delete mac;
			channelIdle();
			// do nothing. contention will automatically start after channel switch
		}
	}
	else if (msg->getKind() == NEXT_CHANNEL_SWITCH_MSG_KIND)
	{
		ASSERT(useSCH);

		scheduleAt(simTime() + SWITCHING_INTERVAL_11P, nextChannelSwitch);

		switch (activeChannel)
		{
		case type_CCH:
			DBG_MAC << "CCH --> SCH" << std::endl;
			channelBusySelf(false);
			setActiveChannel(type_SCH);
			channelIdle(true);
			phy11p->changeListeningFrequency(frequency[mySCH]);
			break;
		case type_SCH:
			DBG_MAC << "SCH --> CCH" << std::endl;
			channelBusySelf(false);
			setActiveChannel(type_CCH);
			channelIdle(true);
			phy11p->changeListeningFrequency(frequency[Channels::CCH]);
			break;
		}
		// schedule next channel switch in 50ms
	}
	else if (msg->getKind() == STOP_IGNORE_CHANNEL_STATE_MSG_KIND)
	{
		ignoreChannelState = false;
	}
	else // note the kind of ackTimeOutMsg is one of enum t_access_category
	{
		AckTimeOutMessage *ackTimeOutMsg = dynamic_cast<AckTimeOutMessage*>(msg);
		ASSERT(ackTimeOutMsg);
		handleAckTimeOut(ackTimeOutMsg);
	}
}

void Mac1609_4::handleLowerMsg(cMessage *msg)
{
	Mac80211Pkt *macPkt = dynamic_cast<Mac80211Pkt*>(msg);
	LAddress::L2Type srcAddr = macPkt->getSrcAddr(), dstAddr = macPkt->getDestAddr();
	WaveShortMessage *wsm = nullptr;
	if (!macPkt->getIsAck())
		wsm = dynamic_cast<WaveShortMessage*>(macPkt->decapsulate());
	if (wsm != nullptr && strcmp(wsm->getName(), "beacon") == 0) // pass information about received frame to the upper layers
	{
		wsm->setRecipientAddress(srcAddr); // trick, there is no field indicates the source's MAC address
		wsm->setControlInfo(msg->removeControlInfo());
	}

	DBG_MAC << "Received frame name = " << macPkt->getName()
	        << ", src = " << srcAddr
	        << ", dst = " << dstAddr
	        << ", myAddr = " << myMacAddr << std::endl;

	if (dstAddr == myMacAddr)
	{
		if (!macPkt->getIsAck())
		{
			DBG_MAC << "Received a data packet addressed to me." << std::endl;
			if (useAcks)
				sendAck(srcAddr, wsm->getTreeId());
			++statsReceivedPackets;
			sendUp(wsm);
		}
		else
		{
			ASSERT(useAcks);
			handleAck(dynamic_cast<Mac80211Ack*>(macPkt));
		}
	}
	else if (dstAddr == LAddress::L2BROADCAST())
	{
		++statsReceivedBroadcasts;
		sendUp(wsm);
	}
	else
	{
		DBG_MAC << "Packet not for me, deleting..." << std::endl;
		delete wsm;
	}
	delete macPkt;

	if (rxStartIndication)
	{
		// We have handled/processed the incoming packet
		// Since we reached here, we were expecting an ack but we didn't get it, so retransmission should take place
		phy11p->notifyMacAboutRxStart(false);
		rxStartIndication = false;
		handleRetransmit(lastAC);
	}
}

void Mac1609_4::handleUpperMsg(cMessage *msg)
{
	WaveShortMessage *thisMsg = dynamic_cast<WaveShortMessage*>(msg);
	if (thisMsg == nullptr)
		error("WaveMac only accepts WaveShortMessages");

	t_access_category ac = mapUserPriority(thisMsg->getPriority());
	unsigned int channelIdentifier = thisMsg->getChannelNumber().value.uval;

	DBG_MAC << "Received a message from upper layer for channel " << channelIdentifier << ", Access Category (Priority): " << ac << "\n";

	t_channel chan = type_CCH;

	if (channelIdentifier != Channels::CCH)   // rewrite SCH channel to actual SCH the Mac1609_4 is set to
	{
		ASSERT(useSCH);
		WAVEInformationElement channelNumber(15, 1, static_cast<unsigned char>(mySCH));
		thisMsg->setChannelNumber(channelNumber);
		chan = type_SCH;
	}

	int num = myEDCA[chan]->queuePacket(ac, thisMsg);

	// packet was dropped in Mac
	if (num == -1)
	{
		statsDroppedPackets++;
		return;
	}

	// if this packet is not at the front of a new queue we dont have to reevaluate times
	DBG_MAC << "sorted packet into queue of EDCA " << chan << ", this packet is now at position: " << num << "\n";

	if (chan == activeChannel)
	{
		DBG_MAC << "this packet is for the currently active channel" << std::endl;
	}
	else
	{
		DBG_MAC << "this packet is NOT for the currently active channel" << std::endl;
	}

	if (num == 1 && chan == activeChannel)
	{
		if (idleChannel)
		{
			simtime_t nextEvent = myEDCA[chan]->startContent(lastIdle, guardActive());

			if (nextEvent != -1)
			{
				if (!useSCH || nextEvent <= nextChannelSwitch->getArrivalTime())
				{
					if (nextMacEvent->isScheduled())
						cancelEvent(nextMacEvent);
					scheduleAt(nextEvent, nextMacEvent);
					DBG_MAC << "Updated nextMacEvent: " << nextMacEvent->getArrivalTime().raw() << std::endl;
				}
				else
				{
					DBG_MAC << "Too little time in this interval. Will not schedule nextMacEvent" << std::endl;
					// it is possible that this queue has an txop. we have to revoke it
					myEDCA[activeChannel]->revokeTxOPs();
					++statsNumTooLittleTime;
				}
			}
			else
				cancelEvent(nextMacEvent);
		}
		else if (myEDCA[chan]->myQueues[ac].currentBackoff == 0)
			myEDCA[chan]->backoff(ac);
	}
}

void Mac1609_4::handleLowerControl(cMessage *msg)
{
	switch (msg->getKind())
	{
	case MacToPhyInterface::TX_OVER:
	{
		DBG_MAC << "Successfully transmitted a packet on " << lastAC << std::endl;

		phy->setRadioState(Radio::RX);

		// message was sent
		// update EDCA queue. go into post-transmit backoff and set cwCur to cwMin
		if (!lastMac->getIsAck())
			myEDCA[activeChannel]->postTransmit(lastAC, lastWSM, useAcks);
		// channel just turned idle.
		// don't set the chan to idle. the PHY layer decides, not us.

		if (guardActive())
			throw cRuntimeError("We should not have sent a packet in guard!");
		break;
	}
	case MacToPhyInterface::PHY_RX_START:
	{
		DBG_MAC << "PhyLayer said RX progress started." << std::endl;

		rxStartIndication = true;
		break;
	}
	case MacToPhyInterface::PHY_RX_END_WITH_SUCCESS:
	{
		// PHY_RX_END_WITH_SUCCESS will get packet soon! Nothing to do here
		break;
	}
	case MacToPhyInterface::PHY_RX_END_WITH_FAILURE:
	{
		DBG_MAC << "PhyLayer said RX progress end with failure." << std::endl;
		// RX failed at phy. Time to retransmit
		phy11p->notifyMacAboutRxStart(false);
		rxStartIndication = false;
		handleRetransmit(lastAC);
		break;
	}
	case Mac80211pToPhy11pInterface::CHANNEL_BUSY:
	{
		channelBusy();
		break;
	}
	case Mac80211pToPhy11pInterface::CHANNEL_IDLE:
	{
		// Decider80211p::processSignalEnd() sends up the received packet to MAC followed by control message CHANNEL_IDLE in the same timestamp.
		// If we received a unicast frame (first event scheduled by Decider), MAC immediately schedules an ACK message and wants to switch the radio to TX mode.
		// So, the notification for channel idle from phy is undesirable and we skip it here.
		// After ACK TX is over, PHY will inform the channel status again.
		if (!ignoreChannelState)
			channelIdle();
		// else: Skipping channelidle because we are about to send an ack regardless of the channel state
		break;
	}
	case Decider80211p::COLLISION:
	{
		emit(sigCollision, true);
		++statsCollisionLostPackets;
		DBG_MAC << "A packet was not received due to collision." << std::endl;
		break;
	}
	case Decider80211p::BITERROR:
	{
		++statsSINRLostPackets;
		DBG_MAC << "A packet was not received due to biterrors." << std::endl;
		break;
	}
	case Decider80211p::RECVWHILESEND:
	{
		++statsTXRXLostPackets;
		DBG_MAC << "A packet was not received because we were sending while receiving." << std::endl;
		break;
	}
	case Decider80211p::NOT_SYNCHRONIZED:
	{
		++statsSyncLostPackets;
		DBG_MAC << "A packet was not received due to synchronized failure." << std::endl;
		break;
	}
	case MacToPhyInterface::RADIO_SWITCHING_OVER:
	{
		DBG_MAC << "Phylayer said radio switching was done." << std::endl;
		break;
	}
	case BaseDecider::PACKET_DROPPED:
	{
		phy->setRadioState(Radio::RX);
		DBG_MAC << "Phylayer said packet was dropped." << std::endl;
		break;
	}
	default:
		throw cRuntimeError("Invalid control message type (type=NOTHING) : name=%s, modulesrc=%s.", msg->getName(), msg->getSenderModule()->getFullPath().c_str());
	}

	delete msg;
	msg = nullptr;
}

void Mac1609_4::handleUpperControl(cMessage* msg)
{
	throw cRuntimeError("Mac layer does not accept upper layer control message.");
}

uint32_t Mac1609_4::getEDCAQueueRoom(t_channel chan, int priority, bool total)
{
	t_access_category ac = mapUserPriority(priority);
	EDCA *edca = myEDCA[chan];
	if (total)
		return edca->maxQueueSize > 0 ? edca->maxQueueSize : 64;
	uint32_t curSize = edca->myQueues[ac].queue.size();
	return edca->maxQueueSize > 0 ? edca->maxQueueSize - curSize : 64;
}

void Mac1609_4::setActiveChannel(t_channel state)
{
	activeChannel = state;
	ASSERT(state == type_CCH || (useSCH && state == type_SCH));
}

void Mac1609_4::sendFrame(Mac80211Pkt *frame, simtime_t delay, double frequency, uint64_t datarate, double txPower_mW)
{
	phy->setRadioState(Radio::TX); // give time for the radio to be in Tx state before transmitting

	delay = std::max(delay, RADIODELAY_11P); // wait at least for the radio to switch

	attachSignal(frame, simTime() + delay, frequency, datarate, txPower_mW);
	// check_and_cast<MacToPhyControlInfo*>(frame->getControlInfo());

	delete lastMac;
	lastMac = frame->dup();
	sendDelayed(frame, delay, lowerLayerOut);

	if (frame->getIsAck())
		++statsSentAcks;
	else
		++statsSentPackets;
}

void Mac1609_4::attachSignal(Mac80211Pkt* mac, simtime_t startTime, double frequency, uint64_t datarate, double txPower_mW)
{
	simtime_t duration = getFrameDuration(mac->getBitLength());

	Signal *s = createSignal(startTime, duration, txPower_mW, datarate, frequency);
	MacToPhyControlInfo *cinfo = new MacToPhyControlInfo(s);

	mac->setControlInfo(cinfo);
}

Signal* Mac1609_4::createSignal(simtime_t start, simtime_t length, double power, uint64_t bitrate, double frequency)
{
	simtime_t end = start + length;
	// create signal with start at current simtime and passed length
	Signal *s = new Signal(start, length);

	// create and set tx power mapping
	ConstMapping *txPowerMapping = createSingleFrequencyMapping(start, end, frequency, 5.0e6, power);
	s->setTransmissionPower(txPowerMapping);

	Mapping *bitrateMapping = MappingUtils::createMapping(DimensionSet::timeDomain(), Mapping::STEPS);

	Argument pos(start);
	bitrateMapping->setValue(pos, bitrate);

	pos.setTime(phyHeaderLength / bitrate);
	bitrateMapping->setValue(pos, bitrate);

	s->setBitrate(bitrateMapping);

	return s;
}

/* checks if guard is active */
bool Mac1609_4::guardActive() const
{
	if (!useSCH) return false;
	return simTime() - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P;
}

/* returns the time until the guard is over */
simtime_t Mac1609_4::timeLeftTillGuardOver() const
{
	ASSERT(useSCH);
	simtime_t sTime = simTime();
	if (sTime - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P)
		return GUARD_INTERVAL_11P - (sTime - nextChannelSwitch->getSendingTime());
	else
		return 0;
}

/* returns the time left in this channel window */
simtime_t Mac1609_4::timeLeftInSlot() const
{
	ASSERT(useSCH);
	return nextChannelSwitch->getArrivalTime() - simTime();
}

Mac1609_4::t_access_category Mac1609_4::mapUserPriority(int prio)
{
	// Map user priority to access category, based on IEEE Std 802.11-2012, Table 9-1
	switch (prio)
	{
	case 1:
	case 2:
		return AC_BK;
	case 0:
	case 3:
		return AC_BE;
	case 4:
	case 5:
		return AC_VI;
	case 6:
	case 7:
		return AC_VO;
	default:
		throw cRuntimeError("MacLayer received a packet with unknown priority");
	}
	return AC_VO;
}

void Mac1609_4::channelBusySelf(bool generateTxOp)
{
	// the channel turned busy because we're sending. we don't want our queues to go into backoff
	// internal contention is already handled in initiateTransmission
	if (!idleChannel)
		return;
	idleChannel = false;
	DBG_MAC << "Channel turned busy: Switch or Self-Send" << std::endl;

	lastBusy = simTime();

	// channel turned busy
	if (nextMacEvent->isScheduled() == true)
		cancelEvent(nextMacEvent);
	// else: the edca subsystem was not doing anything anyway.

	myEDCA[activeChannel]->stopContent(false, generateTxOp);

	emit(sigChannelBusy, true);
}

void Mac1609_4::channelBusy()
{
	if (!idleChannel)
		return;
	// the channel turned busy because someone else is sending
	idleChannel = false;
	DBG_MAC << "Channel turned busy: External sender" << std::endl;

	lastBusy = simTime();

	// channel turned busy
	if (nextMacEvent->isScheduled() == true)
		cancelEvent(nextMacEvent);
	// else: the edca subsystem was not doing anything anyway.

	myEDCA[activeChannel]->stopContent(true, false);

	emit(sigChannelBusy, true);
}

void Mac1609_4::channelIdle(bool afterSwitch)
{
	DBG_MAC << "Channel turned idle: Switch: " << afterSwitch << "\n";

	if (waitUntilAckRXorTimeout)
		return;

	if (nextMacEvent->isScheduled() == true)
	{
		// this rare case can happen when another node's time has such a big offset that the node sent a packet although we already changed the channel
		// the workaround is not trivial and requires a lot of changes to the phy and decider
		return;
		// throw cRuntimeError("channel turned idle but contention timer was scheduled!");
	}

	idleChannel = true;

	simtime_t delay = 0;

	// account for 1609.4 guards
	if (afterSwitch)
	{
		// delay = GUARD_INTERVAL_11P;
	}
	if (useSCH)
		delay += timeLeftTillGuardOver();

	// channel turned idle! lets start contention!
	lastIdle = delay + simTime();
	statsTotalBusyTime += simTime() - lastBusy;

	// get next Event from current EDCA subsystem
	simtime_t nextEvent = myEDCA[activeChannel]->startContent(lastIdle, guardActive());
	if (nextEvent != -1)
	{
		if (!useSCH || nextEvent < nextChannelSwitch->getArrivalTime())
		{
			scheduleAt(nextEvent, nextMacEvent);
			DBG_MAC << "next Event is at " << nextMacEvent->getArrivalTime().raw() << std::endl;
		}
		else
		{
			DBG_MAC << "Too little time in this interval. will not schedule macEvent" << std::endl;
			++statsNumTooLittleTime;
			myEDCA[activeChannel]->revokeTxOPs();
		}
	}
	else
	{
		DBG_MAC << "I don't have any new events in this EDCA sub system" << std::endl;
	}

	emit(sigChannelBusy, false);
}

void Mac1609_4::setParametersForBitrate(uint64_t bitrate)
{
	for (unsigned int i = 0; i < NUM_BITRATES_80211P; ++i)
	{
		if (bitrate == BITRATES_80211P[i])
		{
			n_dbps = N_DBPS_80211P[i];
			return;
		}
	}
	throw cRuntimeError("Chosen Bitrate is not valid for 802.11p: Valid rates are: 3Mbps, 4.5Mbps, 6Mbps, 9Mbps, 12Mbps, 18Mbps, 24Mbps and 27Mbps. Please adjust your omnetpp.ini file accordingly.");
}

simtime_t Mac1609_4::getFrameDuration(int payloadLengthBits, enum PHY_MCS mcs) const
{
	simtime_t duration;
	if (mcs == MCS_DEFAULT)
	{
		// calculate frame duration according to Equation (17-29) of the IEEE 802.11-2007 standard
		duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(n_dbps) );
	}
	else
	{
		uint32_t ndbps = getNDBPS(mcs);
		duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(ndbps) );
	}

	return duration;
}

//////////////////////////////    Unicast    //////////////////////////////
#define FOREACH_EDCAQUEUE    for (std::map<t_access_category, EDCA::EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); ++iter)

void Mac1609_4::sendAck(LAddress::L2Type recpAddress, unsigned long wsmId)
{
	ASSERT(useAcks);
	// 802.11-2012 9.3.2.8
	// send an ACK after SIFS without regard of busy/ idle state of channel
	ignoreChannelState = true;
	channelBusySelf(true);

	// send the packet
	Mac80211Ack *macAck = new Mac80211Ack("ACK");
	macAck->setBitLength(ackLength);
	macAck->setDestAddr(recpAddress);
	macAck->setSrcAddr(myMacAddr);
	macAck->setIsAck(true);
	macAck->setMessageId(wsmId);

	enum PHY_MCS mcs = MCS_DEFAULT;
	uint64_t datarate = getOfdmDatarate(mcs, BW_OFDM_10_MHZ);

	simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(macAck->getBitLength(), mcs);
	DBG_MAC << "Ack sending duration will be " << sendingDuration << "\n";

	// TODO: check ack procedure when channel switching is allowed
	// double freq = (activeChannel == type_CCH) ? frequency[Channels::CCH] : frequency[mySCH];
	double freq = frequency[Channels::CCH];

	DBG_MAC << "Sending an ack. Frequency " << freq << " at time: " << simTime() + SIFS_11P << std::endl;
	sendFrame(macAck, SIFS_11P, freq, datarate, txPower);
	scheduleAt(simTime() + SIFS_11P, stopIgnoreChannelStateMsg);
}

void Mac1609_4::handleAck(Mac80211Ack *macAck)
{
	ASSERT2(rxStartIndication, "Not expecting ack when RX progress not started");
	phy11p->notifyMacAboutRxStart(false);
	rxStartIndication = false;

	bool queueUnblocked = false;
	std::map<t_access_category, EDCA::EDCAQueue> &myQueues = myEDCA[type_CCH]->myQueues;
	FOREACH_EDCAQUEUE
	{
		EDCA::EDCAQueue &EDCAQ = iter->second;
		if (!EDCAQ.queue.empty() && EDCAQ.waitForAck && EDCAQ.waitOnUnicastID == macAck->getMessageId())
		{
			EDCAQ.pop_front();
			myEDCA[type_CCH]->backoff(iter->first);
			if (EDCAQ.ackTimeOut->isScheduled())
				cancelEvent(EDCAQ.ackTimeOut);
			queueUnblocked = true;
		}
	}
	if (queueUnblocked)
		waitUntilAckRXorTimeout = false;
	else
		throw cRuntimeError("Could not find WSM in EDCA queues with WSM ID received in ACK");
}

void Mac1609_4::handleAckTimeOut(AckTimeOutMessage *ackTimeOutMsg)
{
	if (rxStartIndication)
	{
		// Rx is already in process. Wait for it to complete.
		// In case it is not an ack, we will retransmit
		// This assigning might be redundant as it was set already in handleSelfMsg but no harm in reassigning here.
		lastAC = (t_access_category)(ackTimeOutMsg->getKind());
		return;
	}
	// We did not start receiving any packet.
	// stop receiving notification for rx start as we will retransmit
	phy11p->notifyMacAboutRxStart(false);
	// back off and try retransmission again
	handleRetransmit((t_access_category)(ackTimeOutMsg->getKind()));
	// Phy was requested not to send channel idle status on TX_OVER
	// So request the channel status now. For the case when we receive ACK, decider updates channel status itself after ACK RX
	phy11p->requestChannelStatusIfIdle();
}

void Mac1609_4::handleRetransmit(t_access_category ac)
{
	EDCA::EDCAQueue &EDCAQ = myEDCA[type_CCH]->myQueues[ac];
	// cancel the acktime out
	if (EDCAQ.ackTimeOut->isScheduled())
	{
		// This case is possible if we received PHY_RX_END_WITH_SUCCESS or FAILURE even before ack timeout
		cancelEvent(EDCAQ.ackTimeOut);
	}
	if (EDCAQ.queue.empty())
		throw cRuntimeError("Trying retransmission on empty queue...");

	bool contend = false;
	bool retriesExceeded = false;
	WaveShortMessage *wsm = EDCAQ.queue.front();
	// page 879 of IEEE 802.11-2012
	if (wsm->getBitLength() <= dot11RTSThreshold)
	{
		EDCAQ.ssrc++;
		retriesExceeded = EDCAQ.ssrc > dot11ShortRetryLimit;
	}
	else
	{
		EDCAQ.slrc++;
		retriesExceeded = EDCAQ.slrc > dot11LongRetryLimit;
	}
	if (!retriesExceeded) // try again!
	{
		EDCAQ.cwCur = std::min(EDCAQ.cwMax, (EDCAQ.cwCur * 2) + 1);
		myEDCA[type_CCH]->backoff(ac);
		contend = true;
		// no need to reset wait on id here as we are still retransmitting same packet
		EDCAQ.waitForAck = false;
	}
	else // enough tries!
	{
		sendControlUp(wsm->dup()); // notifies upper layer that link layer may be broken
		EDCAQ.pop_front();
		contend = !EDCAQ.queue.empty(); // start contention only if there are more packets in the queue
		myEDCA[type_CCH]->backoff(ac);
	}
	waitUntilAckRXorTimeout = false;
	if (contend && idleChannel && !ignoreChannelState)
	{
		// reevaluate times -- if channel is not idle, then contention would start automatically
		cancelEvent(nextMacEvent);
		simtime_t nextEvent = myEDCA[type_CCH]->startContent(lastIdle, guardActive());
		scheduleAt(nextEvent, nextMacEvent);
	}
}

//////////////////////////////    Mac1609_4::EDCA    //////////////////////////////
void Mac1609_4::EDCA::createQueue(int aifsn, int cwMin, int cwMax, t_access_category ac)
{
	if (myQueues.find(ac) != myQueues.end())
		throw cRuntimeError("You can only add one queue per Access Category per EDCA subsystem");

	EDCAQueue newQueue(aifsn, cwMin, cwMax, ac);
	myQueues[ac] = newQueue;
	myQueues[ac].ackTimeOut = new AckTimeOutMessage("AckTimeOut", ac);
}

int Mac1609_4::EDCA::queuePacket(t_access_category ac, WaveShortMessage* msg)
{
	if (maxQueueSize && myQueues[ac].queue.size() >= maxQueueSize)
	{
		delete msg;
		return -1;
	}
	myQueues[ac].queue.push(msg);
	return myQueues[ac].queue.size();
}

WaveShortMessage* Mac1609_4::EDCA::initiateTransmit(simtime_t lastIdle)
{
	// iterate through the queues to return the packet we want to send
	WaveShortMessage *pktToSend = nullptr;

	simtime_t idleTime = simTime() - lastIdle;

	DBG_MAC << "Initiating transmit at " << simTime() << ". I've been idle since " << idleTime << std::endl;

	// As t_access_category is sorted by priority, we iterate back to front.
	// This realizes the behavior documented in IEEE Std 802.11-2012 Section 9.2.4.2; that is, "data frames from the higher priority AC" win an internal collision.
	// The phrase "EDCAF of higher UP" of IEEE Std 802.11-2012 Section 9.19.2.3 is assumed to be meaningless.
	for (std::map<t_access_category, EDCAQueue>::reverse_iterator riter = myQueues.rbegin(); riter != myQueues.rend(); ++riter)
	{
		EDCAQueue &EDCAQ = riter->second;
		if (!EDCAQ.queue.empty() && !EDCAQ.waitForAck)
		{
			if (idleTime >= EDCAQ.aifsn* SLOTLENGTH_11P + SIFS_11P && EDCAQ.txOP)
			{
				DBG_MAC << "Queue " << riter->first << " is ready to send!" << std::endl;

				EDCAQ.txOP = false;
				//this queue is ready to send
				if (pktToSend == nullptr)
					pktToSend = EDCAQ.queue.front();
				else
				{
					// there was already another packet ready. we have to go increase cw and go into backoff. It's called internal contention and its wonderful
					++statsNumInternalContention;
					EDCAQ.cwCur = std::min(EDCAQ.cwMax, (EDCAQ.cwCur+1)*2 - 1);
					EDCAQ.currentBackoff = owner->intuniform(0, EDCAQ.cwCur);
					DBG_MAC << "Internal contention for queue " << riter->first << " : "<< EDCAQ.currentBackoff << ". Increase cwCur to " << EDCAQ.cwCur << std::endl;
				}
			}
		}
	}

	if (pktToSend == nullptr)
		throw cRuntimeError("No packet was ready");
	return pktToSend;
}

simtime_t Mac1609_4::EDCA::startContent(simtime_t idleSince, bool guardActive)
{
	DBG_MAC << "Restarting contention.\n";

	simtime_t nextEvent = -1;

	simtime_t idleTime = SimTime().setRaw(std::max((int64_t)0, (simTime() - idleSince).raw()));;

	lastStart = idleSince;

	DBG_MAC << "Channel is already idle for: " << idleTime << " since " << idleSince << "\n";

	// this returns the nearest possible event in this EDCA subsystem after a busy channel
	FOREACH_EDCAQUEUE
	{
		EDCAQueue &EDCAQ = iter->second;
		if (!EDCAQ.queue.empty() && !EDCAQ.waitForAck)
		{
			/* 1609_4 says that when attempting to send (backoff == 0) when guard is active, a random backoff is invoked */
			if (guardActive == true && EDCAQ.currentBackoff == 0)
			{
				// cw is not increased
				EDCAQ.currentBackoff = owner->intuniform(0, EDCAQ.cwCur);
				++statsNumBackoff;
			}

			simtime_t DIFS = EDCAQ.aifsn * SLOTLENGTH_11P + SIFS_11P;

			// the next possible time to send can be in the past if the channel was idle for a long time, meaning we COULD have sent earlier if we had a packet
			simtime_t possibleNextEvent = DIFS + EDCAQ.currentBackoff * SLOTLENGTH_11P;

			DBG_MAC << "Waiting Time for Queue " << iter->first <<  ": " << possibleNextEvent << " = " << EDCAQ.aifsn << "*"  << SLOTLENGTH_11P << " + "
			        << SIFS_11P << " + " << EDCAQ.currentBackoff << "*" << SLOTLENGTH_11P << std::endl;

			if (idleTime > possibleNextEvent)
			{
				DBG_MAC << "Could have already send if we had it earlier" << std::endl;
				// we could have already sent. round up to next boundary
				simtime_t base = idleSince + DIFS;
				possibleNextEvent = simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;
			}
			else
			{
				// we are gonna send in the future
				DBG_MAC << "Sending in the future" << std::endl;
				possibleNextEvent = idleSince + possibleNextEvent;
			}
			nextEvent = nextEvent == -1 ? possibleNextEvent : std::min(nextEvent, possibleNextEvent);
		}
	}
	return nextEvent;
}

void Mac1609_4::EDCA::stopContent(bool allowBackoff, bool generateTxOp)
{
	// update all Queues
	DBG_MAC << "Stopping Contention at " << simTime().raw() << std::endl;

	simtime_t passedTime = simTime() - lastStart;

	DBG_MAC << "Channel was idle for " << passedTime << std::endl;

	lastStart = -1; // indicate that there was no last start

	FOREACH_EDCAQUEUE
	{
		EDCAQueue &EDCAQ = iter->second;
		if ((EDCAQ.currentBackoff != 0 || !EDCAQ.queue.empty()) && !EDCAQ.waitForAck)
		{
			// check how many slots we already waited until the chan became busy
			int oldBackoff = EDCAQ.currentBackoff;

			std::string info;
			if (passedTime < EDCAQ.aifsn * SLOTLENGTH_11P + SIFS_11P)
			{
				// we didnt even make it one DIFS :(
				info.append(" No DIFS");
			}
			else
			{
				// decrease the backoff by one because we made it longer than one DIFS
				EDCAQ.currentBackoff--;

				// check how many slots we waited after the first DIFS
				int passedSlots = (int)((passedTime - SimTime(EDCAQ.aifsn * SLOTLENGTH_11P + SIFS_11P)) / SLOTLENGTH_11P);

				DBG_MAC << "Passed slots after DIFS: " << passedSlots << std::endl;

				if (EDCAQ.queue.empty())
				{
					// this can be below 0 because of post transmit backoff -> backoff on empty queues will not generate macevents,
					// we dont want to generate a txOP for empty queues
					EDCAQ.currentBackoff -= std::min(EDCAQ.currentBackoff, passedSlots);
					info.append(" PostCommit Over");
				}
				else
				{
					EDCAQ.currentBackoff -= passedSlots;
					if (EDCAQ.currentBackoff <= -1)
					{
						if (generateTxOp)
						{
							EDCAQ.txOP = true;
							info.append(" TXOP");
						}
						// else: this packet couldnt be sent because there was too little time. we could have generated a txop, but the channel switched
						EDCAQ.currentBackoff = 0;
					}
				}
			}
			DBG_MAC << "Updating backoff for Queue " << iter->first << ": " << oldBackoff << " -> " << EDCAQ.currentBackoff << info <<std::endl;
		}
	}
}

void Mac1609_4::EDCA::backoff(t_access_category ac)
{
	myQueues[ac].currentBackoff = owner->intuniform(0, myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	++statsNumBackoff;
	DBG_MAC << "Going into Backoff because channel was busy when new packet arrived from upperLayer" << std::endl;
}

void Mac1609_4::EDCA::postTransmit(t_access_category ac, WaveShortMessage *wsm, bool useAcks)
{
	EDCAQueue &EDCAQ = myQueues[ac];
	if (wsm->getRecipientAddress() == LAddress::L2BROADCAST() || !useAcks)
	{
		EDCAQ.waitForAck = false;
		delete EDCAQ.queue.front();
		EDCAQ.queue.pop();
		EDCAQ.cwCur = myQueues[ac].cwMin;
		// post transmit backoff
		EDCAQ.currentBackoff = owner->intuniform(0, EDCAQ.cwCur);
		statsSlotsBackoff += EDCAQ.currentBackoff;
		++statsNumBackoff;
		DBG_MAC << "Queue " << ac << " will go into post-transmit backoff for " << EDCAQ.currentBackoff << " slots" << std::endl;
	}
	else
	{
		// mac->waitUntilAckRXorTimeout = true; // set in handleselfmsg()
		// Head of line blocking, wait until ack timeout
		EDCAQ.waitForAck = true;
		EDCAQ.waitOnUnicastID = wsm->getTreeId();
		owner->phy11p->notifyMacAboutRxStart(true);
	}
}

void Mac1609_4::EDCA::cleanUp()
{
	FOREACH_EDCAQUEUE
	owner->cancelAndDelete(iter->second.ackTimeOut); // we clear member 'queue' in the destructor of EDCAQueue
	myQueues.clear();
}

void Mac1609_4::EDCA::revokeTxOPs()
{
	FOREACH_EDCAQUEUE
	{
		if (iter->second.txOP == true)
		{
			iter->second.txOP = false;
			iter->second.currentBackoff = 0;
		}
	}
}
