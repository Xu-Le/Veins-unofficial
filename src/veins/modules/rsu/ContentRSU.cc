//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/rsu/ContentRSU.h"
#if !DEBUG_STAG
#include <fstream>
#endif

Define_Module(ContentRSU);

void ContentRSU::initialize(int stage)
{
	BaseRSU::initialize(stage);

	if (stage == 0)
	{
		nodeNum = linkNum = downloaderNum = 0;
		slotNum = par("slotNum").longValue();
		slotSpan = par("slotSpan").longValue();

		distributeVLinkBytesOnce = 100 * (headerLength + dataLengthBits) / 8; // distributeVLinkPacketsOnce == 1000
		distributeRLinkBytesOnce = 1000 * 1500; // MTU is 1500 bytes, distributeRLinkPacketsOnce == 1000
		distributeVApplBytesOnce = 100 * dataLengthBits / 8;
		distributeRApplBytesOnce = 1000 * 1472; // UDP/IP header length is 28 bytes
		distributeVPeriod = SimTime::ZERO;
		distributeRPeriod = SimTime(8 * distributeRLinkBytesOnce * 10, SIMTIME_NS); // 100Mbps wired channel, 10 is obtained by 1e9 / 100 / 1e6

		distributeVEvt = new cMessage("distribute v evt", ContentRSUMsgKinds::DISTRIBUTE_V_EVT);
		distributeREvt = new cMessage("distribute r evt", ContentRSUMsgKinds::DISTRIBUTE_R_EVT);
		distributeVEvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at vehicle
		distributeREvt->setSchedulingPriority(1); // ensure when handle with self message to send the next packet, the previous packet has arrived at another RSU
		schemeSwitchEvt = new cMessage("scheme switch evt", ContentRSUMsgKinds::SCHEME_SWITCH_EVT);
	}
}

void ContentRSU::finish()
{
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL)
	{
		delete itDL->second;
	}
	downloaders.clear();

	if (distributeVEvt->isScheduled())
		cancelAndDelete(distributeVEvt);
	else
		delete distributeVEvt;
	if (distributeREvt->isScheduled())
		cancelAndDelete(distributeREvt);
	else
		delete distributeREvt;
	if (schemeSwitchEvt->isScheduled())
		cancelAndDelete(schemeSwitchEvt);
	else
		delete schemeSwitchEvt;

	BaseRSU::finish();
}

void ContentRSU::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind())
	{
	case ContentRSUMsgKinds::SCHEME_SWITCH_EVT:
	{
		EV << "switched to the scheme in slot " << itRSL->slot << ", distributed offset starts at " << itRSL->offset.begin << " bytes.\n";
		downloaders[itRSL->downloader]->distributedOffset = itRSL->offset.begin;
		prevSlotStartTime = simTime();
	}
	case ContentRSUMsgKinds::DISTRIBUTE_V_EVT:
	{
		WaveShortMessage *wsm = prepareWSM("data", dataLengthBits, dataOnSch ? t_channel::type_SCH : t_channel::type_CCH, dataPriority, -1);
		ASSERT( wsm != nullptr );
		DataMessage *dataMsg = dynamic_cast<DataMessage*>(wsm);

		dataMsg->setReceiver(itRSL->receiver);
		dataMsg->setDownloader(itRSL->downloader);
		DownloaderInfo *downloaderInfo = downloaders[itRSL->downloader];
		// estimate transmission rate between self and downloader only taking relative distance into consideration
		int distance = RoutingUtils::_length(curPosition, vehicles[itRSL->downloader]->pos);
		ASSERT( distance < 250 );
		int estimatedRate = ContentUtils::rateTable[distance/10];
		EV << "estimated rate between myself and downloader [" << itRSL->downloader << "] is " << 128*estimatedRate << " bytes per second.\n";
		if (itRSL->amount > distributeVApplBytesOnce)
		{
			dataMsg->setIsLast(false);
			dataMsg->setBytesNum(distributeVApplBytesOnce);
			// dataMsg->addBitLength(8*distributeVLinkBytesOnce);
			dataMsg->addBitLength(headerLength);
			distributeVPeriod = SimTime(distributeVLinkBytesOnce*1024/estimatedRate*8, SIMTIME_US);
			scheduleAt(simTime() + distributeVPeriod, distributeVEvt);
			itRSL->amount -= distributeVApplBytesOnce;
			downloaderInfo->distributedOffset += distributeVApplBytesOnce;
			downloaderInfo->distributedAt = simTime();
			EV << "downloader [" << itRSL->downloader << "]'s distributed offset updated to " << downloaderInfo->distributedOffset << std::endl;
		}
		else
		{
			dataMsg->setIsLast(true);
			dataMsg->setBytesNum(itRSL->amount);
			int completePacketNum = itRSL->amount / (dataLengthBits/8);
			int lastPacketLength = 0;
			if ((lastPacketLength = itRSL->amount % (dataLengthBits/8)) != 0)
				lastPacketLength += headerLength/8; // plus header length
			int totalLinkBytes = (headerLength+dataLengthBits)/8 * completePacketNum + lastPacketLength;
			// dataMsg->addBitLength(8 * totalLinkBytes);
			dataMsg->addBitLength(headerLength);
			distributeVPeriod = SimTime(totalLinkBytes*1024/estimatedRate*8, SIMTIME_US);
			downloaderInfo->distributedOffset += itRSL->amount;
			EV << "downloader [" << itRSL->downloader << "]'s distributed offset updated to " << downloaderInfo->distributedOffset << std::endl;

			int prevSlot = itRSL->slot;
			if (++itRSL != rsuSchemeList.end()) // prepare to schedule self message schemeSwitchEvt
			{
				SimTime dispatchedSlotInterval(slotSpan*(itRSL->slot - prevSlot), SIMTIME_MS);
				schemeSwitchInterval = dispatchedSlotInterval - (simTime() - prevSlotStartTime);
				EV << "prepare to switch to the scheme in slot " << itRSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
				scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
			}
		}
		dataMsg->setCurOffset(downloaderInfo->distributedOffset);
		sendDelayedDown(dataMsg, 4*distributeVPeriod/5);
		break;
	}
	case ContentRSUMsgKinds::DISTRIBUTE_R_EVT:
	{
		break;
	}
	default:
		BaseRSU::handleSelfMsg(msg);
	}
}

void ContentRSU::handleWiredMsg(WiredMessage *wiredMsg)
{
    ASSERT( wiredMsg->getControlCode() == WiredMsgCC::NORMAL_DATA_PACKET || wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET );

	DownloaderInfo *downloaderInfo = downloaders[wiredMsg->getDownloader()];
	downloaderInfo->cacheEndOffset = wiredMsg->getCurOffset();
	EV << "downloader [" << wiredMsg->getDownloader() << "]'s cache end offset updates to " << downloaderInfo->cacheEndOffset << " bytes.\n";

	if (wiredMsg->getControlCode() == WiredMsgCC::LAST_DATA_PACKET) // it is the last data packet, indicating prefetch progress has finished
	{
		EV << "its content prefetch progress has finished, now begin to distribute transmission scheme to relays.\n";
		if (!schemeItemList.empty()) // it is necessary to distribute transmission scheme items to vehicles
		{
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
			if (wsm == nullptr) return;
			ContentMessage *distributionMsg = dynamic_cast<ContentMessage*>(wsm);

			distributionMsg->setControlCode(ContentMsgCC::SCHEME_DISTRIBUTION);
			distributionMsg->setDownloader(wiredMsg->getDownloader());
			distributionMsg->setScheme(schemeItemList);
			sendWSM(distributionMsg);

			// clear scheme item list
			for (SchemeItems::iterator itSI = schemeItemList.begin(); itSI != schemeItemList.end(); ++itSI)
				itSI->second.clear();
			schemeItemList.clear();
		}
		if (!rsuSchemeList.empty()) // handle with self transmission scheme
		{
			itRSL = rsuSchemeList.begin();
			schemeSwitchInterval = SimTime(slotSpan*(itRSL->slot-1), SIMTIME_MS);
			EV << "prepare to switch to the scheme in slot " << itRSL->slot << ", wait duration " << schemeSwitchInterval.dbl() << "s.\n";
			scheduleAt(simTime() + schemeSwitchInterval, schemeSwitchEvt);
		}
	}
	delete wiredMsg;
	wiredMsg = nullptr;
}

void ContentRSU::decorateWSM(WaveShortMessage *wsm)
{
	BaseRSU::decorateWSM(wsm);
}

void ContentRSU::onBeacon(BeaconMessage *beaconMsg)
{
	// LAddress::L3Type sender = beaconMsg->getSenderAddress(); // alias
	// bool isOld = vehicles.find(sender) != vehicles.end();

	BaseRSU::onBeacon(beaconMsg);
}

void ContentRSU::onRouting(RoutingMessage *routingMsg)
{
	EV << "ContentRSUs don't react to content messages since they don't help routings.\n";
}

void ContentRSU::onContent(ContentMessage* contentMsg)
{
	EV << logName() << ": " << "onContent!\n";

	// LAddress::L3Type sender = contentMsg->getSenderAddress(); // alias
	LAddress::L3Type downloader = contentMsg->getDownloader(); // alias
	DownloaderInfo *downloaderInfo = nullptr;
	switch (contentMsg->getControlCode())
	{
	case ContentMsgCC::CONTENT_REQUEST: // it is a request message from a downloader
	{
		if ( downloaders.find(downloader) != downloaders.end() ) // update old record
		{
			EV << "    downloader: node[" << downloader << "] is an old downloader, update its info.\n";
			downloaders[downloader]->acknowledgedOffset = contentMsg->getReceivedOffset();
			downloaders[downloader]->remainingDataAmount = contentMsg->getReceivedOffset() - contentMsg->getConsumedOffset();
		}
		else
		{
			// insert new record
			EV << "    downloader: node[" << downloader << "] is a new downloader, insert its info.\n";
			downloaderInfo = new DownloaderInfo(contentMsg->getContentSize(), contentMsg->getConsumingRate());
			downloaders.insert(std::pair<LAddress::L3Type, DownloaderInfo*>(downloader, downloaderInfo));

			// response to the request vehicle
			WaveShortMessage *wsm = prepareWSM("content", contentLengthBits, t_channel::type_CCH, contentPriority, -1);
			ASSERT( wsm != nullptr );
			ContentMessage *responseMsg = dynamic_cast<ContentMessage*>(wsm);

			responseMsg->setControlCode(ContentMsgCC::CONTENT_RESPONSE);
			responseMsg->setDownloader(downloader);
			sendWSM(responseMsg);

			// use STAG approach to obtain the optimal cooperative transmission scheme
			SimTime calculatingTime = obtainCooperativeScheme();
			ackMsgNum = 0;

			// send prefetch data request to content server
			WiredMessage *wiredMsg = new WiredMessage("prefetch");
			wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
			wiredMsg->setDownloader(downloader);
			wiredMsg->setContentSize(contentMsg->getContentSize());
			wiredMsg->setStartOffset(0);
			wiredMsg->setEndOffset(downloaderInfo->prefetchDataAmount);
			EV << "prefetching " << downloaderInfo->prefetchDataAmount << " bytes from content server for it.\n";
			wiredMsg->addBitLength(wiredHeaderLength + 160); // 5*sizeof(int) * 8
			sendDelayed(wiredMsg, calculatingTime, wiredOut);
		}
		break;
	}
	case ContentMsgCC::CONTENT_RESPONSE: // it is a response message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::SCHEME_DISTRIBUTION: // it is a transmission scheme message distributed by RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::ACKNOWLEDGEMENT:
	{
		if ( downloaders.find(downloader) != downloaders.end() )
		{
			EV << "downloader [" << downloader << "]'s acknowledged offset updates to " << contentMsg->getReceivedOffset() << std::endl;
			downloaderInfo = downloaders[downloader];
			downloaderInfo->acknowledgedOffset = contentMsg->getReceivedOffset();
			if (++ackMsgNum == slotNum)
			{
				// use STAG approach to obtain the optimal cooperative transmission scheme again
				SimTime calculatingTime = obtainCooperativeScheme();
				ackMsgNum = 0;

				// send prefetch data request to content server
				WiredMessage *wiredMsg = new WiredMessage("prefetch");
				wiredMsg->setControlCode(WiredMsgCC::START_TRANSMISSION);
				wiredMsg->setDownloader(downloader);
				wiredMsg->setContentSize(downloaderInfo->totalContentSize);
				wiredMsg->setStartOffset(downloaderInfo->cacheEndOffset);
				wiredMsg->setEndOffset(downloaderInfo->acknowledgedOffset + downloaderInfo->prefetchDataAmount);
				downloaderInfo->prefetchDataAmount -= downloaderInfo->cacheEndOffset - downloaderInfo->acknowledgedOffset;
				EV << "prefetching " << downloaderInfo->prefetchDataAmount << " bytes from content server for it.\n";
				wiredMsg->addBitLength(wiredHeaderLength + 160); // 5*sizeof(int) * 8
				sendDelayed(wiredMsg, calculatingTime, wiredOut);
			}
		}
		break;
	}
	case ContentMsgCC::CARRIER_SELECTION: // it is a carrier chosen notification message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::DOWNLOADING_COMPLETED: // it is a downloading completed notification message from vehicle
	{
	    break;
	}
	case ContentMsgCC::LINK_BREAK_DIRECT: // it is a link break notification - the communication link between downloader and RSU will break soon
	{
		if (contentMsg->getNeighborInfo().empty()) // this downloader has no neighbors, only transfer the rest data to the next RSU
		{
			// TODO: transfer operations write here
		}
		break;
	}
	case ContentMsgCC::LINK_BREAK_DR: // it is a link break notification - the communication link between downloader and relay will break soon
	{
		// transfer the rest data as well as downloader's neighbors info to the next RSU
		break;
	}
	case ContentMsgCC::LINK_BREAK_RR: // it is a link break notification - the communication link between relay and RSU will break soon
	{
		break;
	}
	case ContentMsgCC::RELAY_DISCOVERY: // it is a relay discovery message from RSU
	{
		EV_WARN << "RSU should not receive a message sent by itself.\n";
		break;
	}
	case ContentMsgCC::DISCOVERY_RESPONSE: // it is a response to relay discovery message from relay vehicle
	{
		break;
	}
	default:
		EV_WARN << "Unknown control code " << contentMsg->getControlCode() << ", ignore it." << std::endl;
	}
}

void ContentRSU::onData(DataMessage* dataMsg)
{
	EV << logName() << ": " << "onData!\n";

	findHost()->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(dataMsg->getSenderPos(), curPosition, "blue"));
}

SimTime ContentRSU::obtainCooperativeScheme()
{
	Timer algorithmTimer("Algorithm Time Cost: ");

	predictLinkBandwidth();
	printLinkBandwidth();

	STAG graph(nodeNum, linkNum, downloaderNum, slotNum, downloaderArray, downloaderTable, remainingTable, playTable);

	graph.construct(linkTuples);
	graph.undirectify(linkTuples);

	graph.maximumFlow();

	// maintain node ID map from node IDs used by STAG to real vehicle IDs
	std::vector<int> nodeIdMap(vehicles.size()+1, 0);
	size_t k = 0;
	int nodeId = 1;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
		nodeIdMap[nodeId++] = itV->first;
	// get prefetch data amount list from STAG
	for (int d = 0; d < downloaderNum; ++d)
		downloaders[nodeIdMap[downloaderArray[d]]]->prefetchDataAmount = graph.prefetchAmountTable[downloaderArray[d]];
	// get RSU self scheme list from STAG
	for (k = 0; k < graph.fluxSchemeList[0].size(); ++k)
	{
		STAG::FluxScheme &fluxScheme = graph.fluxSchemeList[0][k]; // alias
		rsuSchemeList.push_back(SchemeTuple(fluxScheme.slot, nodeIdMap[fluxScheme.dest], nodeIdMap[fluxScheme.downloader], fluxScheme.flow));
		// calculate offset of distributed data in each slot
		rsuSchemeList.back().offset.assign(&fluxScheme.segment);
		rsuSchemeList.back().offset.begin += downloaders[nodeIdMap[fluxScheme.downloader]]->cacheStartOffset;
		rsuSchemeList.back().offset.end += downloaders[nodeIdMap[fluxScheme.downloader]]->cacheStartOffset;
	}
	// get vehicles' scheme item list from STAG
	for (nodeId = 1; nodeId < nodeNum; ++nodeId)
	{
		if (graph.fluxSchemeList[nodeId].empty() == false)
		{
			schemeItemList.insert(std::pair<LAddress::L3Type, std::list<SchemeTuple> >(nodeIdMap[nodeId], std::list<SchemeTuple>()));
			for (k = 0; k < graph.fluxSchemeList[nodeId].size(); ++k)
			{
				STAG::FluxScheme &fluxScheme = graph.fluxSchemeList[nodeId][k]; // alias
				std::list<SchemeTuple> &schemeItem = schemeItemList[nodeIdMap[nodeId]]; // alias
				schemeItem.push_back(SchemeTuple(fluxScheme.slot, nodeIdMap[fluxScheme.dest], nodeIdMap[fluxScheme.downloader], fluxScheme.flow));
				// calculate offset of distributed data in each slot
				schemeItem.back().offset.assign(&fluxScheme.segment);
				schemeItem.back().offset.begin += downloaders[nodeIdMap[fluxScheme.downloader]]->cacheStartOffset;
				schemeItem.back().offset.end += downloaders[nodeIdMap[fluxScheme.downloader]]->cacheStartOffset;
			}
		}
	}

	graph.destruct();
	linkTuples.clear();
	downloaderArray.clear();
	downloaderTable.clear();
	remainingTable.clear();
	playTable.clear();
	return SimTime(algorithmTimer.elapsed(), SIMTIME_MS);
}

void ContentRSU::predictVehicleMobility(std::vector<std::vector<Coord> >& vehicleSpeed, std::vector<std::vector<Coord> >& vehiclePos)
{
	for (int j = 1; j < slotNum; ++j)
		for (int nodeId = 1; nodeId < nodeNum; ++nodeId) // update vehicles position, assuming they have uniform motion
			vehiclePos[nodeId][j] = vehiclePos[nodeId][j-1] + vehicleSpeed[nodeId][0];
}

void ContentRSU::predictLinkBandwidth()
{
	nodeNum = static_cast<int>(vehicles.size()) + 1;
	linkNum = nodeNum * (nodeNum + 1) / 2;
	downloaderNum = static_cast<int>(downloaders.size());

	std::vector<int> nodeIdMap(vehicles.size()+1, 0); // maintain node ID map from node IDs used by STAG to real vehicle IDs
	std::map<int, int> nodeIdRevMap; // maintain node ID map from real vehicle IDs to node IDs used by STAG
	int nodeId = 1, j = 0;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		nodeIdMap[nodeId] = itV->first;
		nodeIdRevMap.insert(std::pair<int, int>(itV->first, nodeId++));
	}

	std::vector<Coord> vecPadding(slotNum, Coord::ZERO);
	std::vector<std::vector<Coord> > nodeSpeed;
	std::vector<std::vector<Coord> > nodePos;
	for (nodeId = 0; nodeId < nodeNum; ++nodeId)
	{
		nodeSpeed.push_back(vecPadding);
		nodePos.push_back(vecPadding);
	}
	for (j = 0; j < slotNum; ++j)
	{
		nodeSpeed[0][j] = Coord::ZERO; // RSU has no speed since it cannot move
		nodePos[0][j] = curPosition;   // node 0 is RSU
	}
	for (nodeId = 1; nodeId < nodeNum; ++nodeId) // current position of vehicles
	{
		nodeSpeed[nodeId][0] = vehicles[nodeIdMap[nodeId]]->speed;
		nodePos[nodeId][0] = vehicles[nodeIdMap[nodeId]]->pos;
	}
	predictVehicleMobility(nodeSpeed, nodePos);

	int srcNode = 0, dstNode = 0;
	for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
		for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
			linkTuples.push_back(LinkTuple(srcNode, dstNode, slotNum));

	std::list<LinkTuple>::iterator itLT = linkTuples.begin();
	for (j = 0; j < slotNum; ++j)
	{
		itLT = linkTuples.begin();
		for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
		{
			for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
			{
				int dist = RoutingUtils::_length(nodePos[srcNode][j], nodePos[dstNode][j]);
				if (dist >= 250)
					itLT->bandwidth[j] = 0;
				else
					itLT->bandwidth[j] = 128*ContentUtils::rateTable[dist/10];
				++itLT;
			}
		}
	}

	int downloaderIdx = 0;
	for (itDL = downloaders.begin(); itDL != downloaders.end(); ++itDL, ++downloaderIdx)
	{
		int downloaderID = nodeIdRevMap[itDL->first]; // downloader's corresponding nodeId used by STAG
		downloaderArray.push_back(downloaderID);
		downloaderTable.insert(std::pair<int, int>(downloaderID, downloaderIdx));
		remainingTable.insert(std::pair<int, int>(downloaderID, itDL->second->remainingDataAmount));
		playTable.insert(std::pair<int, int>(downloaderID, itDL->second->consumingRate));
	}

#if !DEBUG_STAG
	std::ofstream fout("STAG_input.txt", std::ios_base::out | std::ios_base::trunc);
	if ( !fout.is_open() )
	{
		error("cannot open file STAG_input.txt!");
	}
	else
	{
		fout << nodeNum << " " << linkNum << " " << downloaderNum << " " << slotNum << "\n\n";
		for (itLT = linkTuples.begin(); itLT != linkTuples.end(); ++itLT)
		{
			fout << itLT->src << " " << itLT->dst << " |";
			for (int j = 0; j < slotNum; ++j)
				fout << " " << itLT->bandwidth[j];
			fout << "\n";
		}
		fout << std::endl;

		for (nodeId = 0; nodeId < nodeNum; ++nodeId)
		{
			fout << nodeId;
			for (int d = 0; d < downloaderIdx; ++d)
			{
				if (nodeId == downloaderArray[d])
					fout << " " << downloaderArray[d] << " " << remainingTable[nodeId];
				else
					fout << " " << downloaderArray[d] << " 0";
			}
			fout << "\n";
		}
		fout << std::endl;

		for (std::map<int, int>::iterator itPT = playTable.begin(); itPT != playTable.end(); ++itPT)
			fout << itPT->first << " " << itPT->second << "\n";
		fout << std::endl;
	}
	fout.close();
#endif
}

void ContentRSU::printLinkBandwidth()
{
	EV << nodeNum << " " << linkNum << " " << downloaderNum << " " << slotNum << "\n\n";
	for (std::list<LinkTuple>::iterator itLT = linkTuples.begin(); itLT != linkTuples.end(); ++itLT)
	{
		EV << itLT->src << " " << itLT->dst << " |";
		for (int j = 0; j < slotNum; ++j)
			EV << " " << itLT->bandwidth[j];
		EV << "\n";
	}
	EV << std::endl;

	for (std::map<int, int>::iterator itPT = playTable.begin(); itPT != playTable.end(); ++itPT)
		EV << itPT->first << " " << itPT->second << "\n";
}

