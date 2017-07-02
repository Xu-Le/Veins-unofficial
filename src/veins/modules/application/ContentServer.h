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

#ifndef __CONTENTSERVER_H__
#define __CONTENTSERVER_H__

#include <queue>
#include <omnetpp/csimplemodule.h>
#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/messages/WiredMessage_m.h"

/**
 * @brief File Content Server.
 *
 * @author Xu Le
 */
class ContentServer : public ::omnetpp::cSimpleModule
{
public:
	/** @name constructor, destructor. */
	///@{
	ContentServer() : cSimpleModule() {}
	ContentServer(unsigned stacksize) : cSimpleModule(stacksize) {}
	virtual ~ContentServer() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief The self message kinds. */
	enum SelfMsgKinds {
		DISTRIBUTE_RSU_EVT,
		DISTRIBUTE_BS_EVT
	};

	/** @brief Called every time a message arrives. */
	void handleMessage(omnetpp::cMessage *msg) override;

	/** @brief Handle self messages. */
	void handleSelfMsg(omnetpp::cMessage *msg);
	/** @brief Handle wired incoming messages from cellular base station. */
	void handleLTEIncomingMsg(WiredMessage *lteMsg);
	/** @brief Handle wired incoming messages from road side unit. */
	void handleRSUIncomingMsg(WiredMessage *rsuMsg, int rsuIdx);

private:
	/** @brief The class to store downloader's information. */
	class DownloaderInfo
	{
	public:
		DownloaderInfo(int t) : totalContentSize(t), distributedOffset(0), requiredEndOffset(t), rsuIndex(-1), distributedAt(omnetpp::SimTime::ZERO) {}

		int totalContentSize;
		int distributedOffset;
		int requiredEndOffset;
		int rsuIndex;
		SimTime distributedAt;
	};

	int rsuNum; ///< number of RSUs in current scenario.
	/** @name gate IDs. */
	///@{
	int *rsuIn;      ///< receive packets from road side units.
	int *rsuOut;     ///< send packets to road side units.
	int cellularIn;  ///< receive packets from cellular base station.
	int cellularOut; ///< send packets to cellular base station.
	///@}
	int headerLength; ///< header length of the wired UDP/IP packet in bits.

	std::vector<std::queue<LAddress::L3Type, std::list<LAddress::L3Type> > > activeDownloaderQs; ///< RSUs' queue of downloaders which are currently fetching data.

	/** @name performance consideration. */
	///@{
	int distributeRSULinkBytesOnce; ///< how many bytes measured in link layer to distribute to RSU once in transmission.
	int distributeBSLinkBytesOnce;  ///< how many bytes measured in link layer to distribute to BS once in transmission.
	int distributeRSUApplBytesOnce; ///< how many bytes measured in application layer to distribute to RSU once in transmission.
	int distributeBSApplBytesOnce;  ///< how many bytes measured in application layer to distribute to BS once in transmission.
	SimTime distributeRSUPeriod; ///< period to handle self message prefetchEvt.
	SimTime distributeBSPeriod;  ///< period to handle self message distributeEvt.
	///@}

	cMessage *distributeRSUEvt;  ///< self message used to periodically distribute data to RSU.
	cMessage *distributeBSEvt;   ///< self message used to periodically distribute data to BS.

	std::map<LAddress::L3Type, DownloaderInfo*> downloaders; ///< a map from a downloader's identifier to all its related info.
	std::map<LAddress::L3Type, DownloaderInfo*>::iterator itDL; ///< a iterator used to traverse container downloaders.
};

#endif /* __CONTENTSERVER_H__ */
