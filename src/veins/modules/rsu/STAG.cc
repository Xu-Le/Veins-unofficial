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

#include "veins/modules/rsu/STAG.h"
#include <omnetpp/cexception.h>

using omnetpp::cRuntimeError;

int STAG::arcIDIncrement = 0;
int STAG::gDownloaderNum = 0;
int STAG::gSlotNum = 0;

STAG::Arc::Arc() : arcID(arcIDIncrement), srcID(-1), dstID(-1), nextArc(NULL)
{
	bandwidth = new int[gSlotNum];
	flow = new int[2*gSlotNum];       // index range [slotNum, 2*slotNum-1) is used for recording purpose
	downloader = new int[2*gSlotNum]; // index range [slotNum, 2*slotNum-1) is used for recording purpose
	idle = new bool[gSlotNum];
	memset(flow, 0, 2*gSlotNum*sizeof(int));
	memset(downloader, -1, 2*gSlotNum*sizeof(int));
	memset(idle, true, gSlotNum*sizeof(bool));
	++arcIDIncrement;
}

STAG::Arc::~Arc()
{
	delete []bandwidth;
	delete []flow;
	delete []downloader;
	delete []idle;
}

STAG::Node::Node() : firstArc(NULL), lastArc(NULL)
{
	_S = new struct Data[(gSlotNum+1)*gDownloaderNum];
	S = new struct Data*[gSlotNum+1];
	for (int i = 0; i < gSlotNum+1; ++i)
		S[i] = _S + i*gDownloaderNum;
}

STAG::Node::~Node()
{
	delete []S;
	delete []_S;
}

/** constructor of class STAG. */
STAG::STAG(int _nodeNum, int _linkNum, int _downloaderNum, int _slotNum, std::vector<int>& _downloaderArray, std::map<int, int>& _downloaderTable, std::map<int, int>& _remainingTable, std::map<int, int>& _playTable, std::map<int, int>& _demandingAmountTable)
	: nodeNum(_nodeNum), linkNum(_linkNum), downloaderNum(_downloaderNum), slotNum(_slotNum), downloaderArray(_downloaderArray), downloaderTable(_downloaderTable), remainingTable(_remainingTable), playTable(_playTable), demandingAmountTable(_demandingAmountTable)
{
	arcNum = 2*linkNum + downloaderNum;
	arcIDIncrement = 0;
	gDownloaderNum = downloaderNum;
	gSlotNum = slotNum;
	NodeList = new Node[nodeNum + 1]; // reserve one node as the super sink
	arcTable = new Arc[arcNum];
}

/** destructor of class STAG. */
STAG::~STAG()
{
	delete []arcTable;
	delete []NodeList;
}

/** construct the graph in the structure of adjacency list. */
void STAG::construct(std::list<LinkTuple>& linkTuples)
{
	std::list<LinkTuple>::iterator iter1 = linkTuples.begin(), iter2;

	int curNode = 0, rank = 2;
	struct Arc *firstarc = NULL, *parc = NULL, *qarc = NULL;

	int arcID = 0;
	firstarc = &arcTable[arcID];
	firstarc->srcID = iter1->src;
	firstarc->dstID = iter1->dst;
	memcpy(firstarc->bandwidth, iter1->bandwidth, slotNum*sizeof(int));
	firstarc->nextArc = NULL;

	curNode = iter1->src; // node index start with 0
	NodeList[curNode].firstArc = firstarc;
	NodeList[curNode].lastArc = firstarc;
	while (true)
	{
		iter2 = iter1; // iter2 is always a forword element of iter1
		++iter2;
		if (iter2 == linkTuples.end())
			break;
		if (iter2->src == iter1->src)
		{
			if (rank % 2 == 0)
			{
				arcID += 2;
				parc = &arcTable[arcID];
				parc->srcID = iter2->src;
				parc->dstID = iter2->dst;
				memcpy(parc->bandwidth, iter2->bandwidth, slotNum*sizeof(int));
				parc->nextArc = NULL;

				if (rank == 2)
					firstarc->nextArc = parc;
				else
					qarc->nextArc = parc;
				NodeList[curNode].lastArc = parc;
			}
			else
			{
				arcID += 2;
				qarc = &arcTable[arcID];
				qarc->srcID = iter2->src;
				qarc->dstID = iter2->dst;
				memcpy(qarc->bandwidth, iter2->bandwidth, slotNum*sizeof(int));
				qarc->nextArc = NULL;

				parc->nextArc = qarc;
				NodeList[curNode].lastArc = qarc;
			}
			++rank;
		}
		else
		{
			rank = 2;

			arcID += 2;
			firstarc = &arcTable[arcID];
			firstarc->srcID = iter2->src;
			firstarc->dstID = iter2->dst;
			memcpy(firstarc->bandwidth, iter2->bandwidth, slotNum*sizeof(int));
			firstarc->nextArc = NULL;

			curNode = iter2->src;
			NodeList[curNode].firstArc = firstarc;
			NodeList[curNode].lastArc = firstarc;
		}
		++iter1;
	}

	_initNodeStorage();
}

/** internal function called by function construct(). */
void STAG::_initNodeStorage()
{
	for (int downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		int downloader = downloaderArray[downloaderIdx];
		for (int i = 1; i < nodeNum; ++i)
		{
			NodeList[i].S[0][downloaderIdx].downloader = downloader;
			NodeList[i].S[0][downloaderIdx].size = downloaderTable.find(i) != downloaderTable.end() ? remainingTable[i] : 0;
		}
		NodeList[0].S[0][downloaderIdx].downloader = downloader;
		NodeList[0].S[0][downloaderIdx].size = 1024*1024*1024; // 1GB
	}
}

/** tranform directed graph to be undirected graph. */
void STAG::undirectify(std::list<LinkTuple>& linkTuples)
{
	struct Arc *parc = NULL;

	int arcID = -1; // set arc x and x+1 be the same link with opposite direction, x is an even integer

	for (std::list<LinkTuple>::iterator iter = linkTuples.begin(); iter != linkTuples.end(); ++iter)
	{
		int curNode = iter->dst; // alias
		arcID += 2;
		// data will never transmit from vehicles to RSU, but maybe transmit from downloaders
		// to other vehicles when the downloader is a relay of another downloader at the same time
		if (iter->src == 0)
			continue;

		parc = &arcTable[arcID];
		parc->srcID = curNode;
		parc->dstID = iter->src;
		memcpy(parc->bandwidth, iter->bandwidth, slotNum*sizeof(int));
		parc->nextArc = NULL;

		if (NodeList[curNode].firstArc == NULL)
			NodeList[curNode].firstArc = parc;
		else
			NodeList[curNode].lastArc->nextArc = parc; // append to the last
		NodeList[curNode].lastArc = parc; // update lastArc
	}
}

/** destruct the graph and free memory. */
void STAG::destruct()
{
	for (int i = 0; i < nodeNum; ++i)
	{
		NodeList[i].firstArc = NULL;
		NodeList[i].lastArc = NULL;
	}
}

void STAG::maximumFlow()
{
	superSink = nodeNum;
	interruptTimeTable.resize(downloaderNum);
	receivedAmountTable.resize(downloaderNum);
	lackSlotsTable.resize(downloaderNum);

	_addSuperSink();

	_obtainArcPathList();

	_obtainInitialScheme();

	_calcNodeStorage();

	double totalInterruptTime = 0.0;
	int totalReceivedAmount = 0;
	_obtainSchemeIndicators(totalInterruptTime, totalReceivedAmount);

	double minTotalInterruptTime = totalInterruptTime;
	int maxTotalReceivedAmount = totalReceivedAmount;
	_recordBetterScheme();

	for (int loop = 0; loop < slotNum/2; ++loop)
	{
		EV << "===============    Loop " << loop + 1 << "    ===============\n";

		bool continueLoop = _seekBetterScheme();
		if (!continueLoop)
			break;

		_clearNodeStorage();

		_calcNodeStorage();

		_obtainSchemeIndicators(totalInterruptTime, totalReceivedAmount);

		// if indeed get a better scheme, then record this better scheme
		if (minTotalInterruptTime > totalInterruptTime || (minTotalInterruptTime == totalInterruptTime && maxTotalReceivedAmount < totalReceivedAmount))
		{
			minTotalInterruptTime = totalInterruptTime;
			maxTotalReceivedAmount = totalReceivedAmount;
			_recordBetterScheme();
		}
	}

	EV << "Final total interrupt time is " << minTotalInterruptTime << ", total received data amount is " << maxTotalReceivedAmount << ".\n";

	_revertBestScheme();

	_recordFluxScheme();

	_clearNodeStorage();

	_delSuperSink();
}

void STAG::_obtainArcPathList()
{
	std::vector<int> arcPath;

	struct Arc *parc = NULL, *qarc = NULL;
	for (parc = NodeList[0].firstArc; parc != NULL; parc = parc->nextArc)
	{
		arcPath.push_back(parc->arcID);

		if (downloaderTable.find(parc->dstID) != downloaderTable.end())
		{
			arcPath.push_back(parc->dstID);
			arcPathList.push_back(arcPath);
			arcPath.pop_back();
		}

		for (qarc = NodeList[parc->dstID].firstArc; qarc != NULL; qarc = qarc->nextArc)
		{
			if (downloaderTable.find(qarc->dstID) != downloaderTable.end())
			{
				arcPath.push_back(qarc->arcID);
				arcPath.push_back(qarc->dstID);
				arcPathList.push_back(arcPath);
				arcPath.pop_back();
				arcPath.pop_back();
			}
		}

		arcPath.pop_back();
	}

	std::sort(arcPathList.begin(), arcPathList.end(), [](const std::vector<int>& lhs, const std::vector<int>& rhs) {
		if (lhs.back() == rhs.back())
			return lhs.size() < rhs.size();
		return lhs.back() < rhs.back();
	});

	EV << "===== Arc path list =====\n";
	for (itAPL = arcPathList.begin(); itAPL != arcPathList.end(); ++itAPL)
	{
		for (size_t k = 0; k < itAPL->size(); ++k)
			EV << itAPL->at(k) << " ";
		EV << std::endl;
	}

	int arcPathListSize = static_cast<int>(arcPathList.size());
	int downloaderIdx = 0;
	arcPathRange.push_back(std::pair<int, int>(0, arcPathListSize));
	for (int i = 1; i < arcPathListSize; ++i)
	{
		if (arcPathList[i].back() != arcPathList[i-1].back())
		{
			arcPathRange[downloaderIdx++].second = i;
			arcPathRange.push_back(std::pair<int, int>(i, arcPathListSize));
		}
	}

#if INFO_STAG
	EV << "===== Arc path range =====\n";
	for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
		EV << "downloader " << downloaderArray[downloaderIdx] << ": [" << arcPathRange[downloaderIdx].first << "," << arcPathRange[downloaderIdx].second << ")\n";
#endif
}

void STAG::_obtainInitialScheme()
{
	size_t k = 0;
	std::vector<int> initialChosenLink;
	std::vector<int> initialChosenPath;
	std::vector<int> initialChosenSlot;
	initialChosenLink.reserve(4);
	initialChosenPath.reserve(4);
	initialChosenLink.push_back(-1); // default fill
	initialChosenPath.push_back(-1); // default fill
	initialChosenSlot.reserve(slotNum+1); // the last element indicates where separating position is
	for (k = 0; k < arcPathList.size(); ++k)
		chosenSlots.push_back(initialChosenSlot);
	struct Arc *parc = NULL, *qarc = NULL, *tarc = NULL, *rarc = NULL;
	for (int j = 0; j < slotNum; ++j)
	{
		int maxBandwidth = 0;
		int maxArcID = 0;
		size_t maxArcPathIdx = 0;
		for (k = 0; k < arcPathList.size(); ++k)
		{
			if (arcPathList[k].size() == 2) // direct download path
			{
				parc = &arcTable[arcPathList[k].front()];
				if (maxBandwidth < parc->bandwidth[j])
				{
					maxBandwidth = parc->bandwidth[j];
					maxArcID = arcPathList[k].front();
					maxArcPathIdx = k;
				}
			}
		}
		parc = &arcTable[maxArcID];
		parc->flow[j] = parc->bandwidth[j];
		parc->downloader[j] = parc->dstID;
		initialChosenLink[0] = maxArcID;
		initialChosenPath[0] = maxArcPathIdx;
		chosenLinks.push_back(initialChosenLink);
		chosenPaths.push_back(initialChosenPath);
		chosenSlots[maxArcPathIdx].push_back(j);

		// handle with transmiter's channel status
		for (qarc = NodeList[parc->srcID].firstArc; qarc != NULL; qarc = qarc->nextArc)
		{
			qarc->idle[j] = false;
			rarc = oppositeArc(qarc->arcID);
			rarc->idle[j] = false;
			if (qarc->bandwidth[j] > 0) // handle with transmiter neighbor's channel status
			{
				for (tarc = NodeList[qarc->dstID].firstArc; tarc != NULL; tarc = tarc->nextArc)
				{
					tarc->idle[j] = false; // a vehicle and its neighbor cannot transmit at the same time;
					rarc = oppositeArc(tarc->arcID);
					rarc->idle[j] = false; // when a vehicle is transmiting, its neighbor cannot receive from another vehicle.
				}
			}
		}
		// handle with receiver's channel status
		for (qarc = NodeList[parc->dstID].firstArc; qarc != NULL; qarc = qarc->nextArc)
		{
			qarc->idle[j] = false;
			rarc = oppositeArc(qarc->arcID);
			rarc->idle[j] = false;
			if (qarc->bandwidth[j] > 0) // handle with receiver neighbor's channel status
				for (tarc = NodeList[qarc->dstID].firstArc; tarc != NULL; tarc = tarc->nextArc)
					tarc->idle[j] = false; // when a vehicle is receiving, its neighbor(except for its opposite) cannot transmit at the same time but they can receive from a non-neighbor of its opposite.
		}
	}
	for (k = 0; k < arcPathList.size(); ++k)
	{
		int indicator = chosenSlots[k].empty() ? 0 : -1;
		chosenSlots[k].push_back(indicator);
	}
}

bool STAG::_seekBetterScheme()
{
	int downloaderIdx = 0;
	double maxInterruptTime = 0.0;
	int maxInterruptIdx = 0;
	for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		if (maxInterruptTime < interruptTimeTable[downloaderIdx])
		{
			maxInterruptTime = interruptTimeTable[downloaderIdx];
			maxInterruptIdx = downloaderIdx;
		}
	}
	if (maxInterruptTime > 1e-6) // video play progress exists interruption
	{
		return _optimizeInterruptTime(maxInterruptIdx);
	}
	else
	{
		int minReceivedAmount = INT32_MAX;
		int minReceivedIdx = 0;
		for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
		{
			if (minReceivedAmount > receivedAmountTable[downloaderIdx])
			{
				minReceivedAmount = receivedAmountTable[downloaderIdx];
				minReceivedIdx = downloaderIdx;
			}
		}
		return _optimizeReceivedAmount(minReceivedIdx);
	}
}

bool STAG::_optimizeInterruptTime(const int curDownloaderIdx)
{
	int latestLackSlot = lackSlotsTable[curDownloaderIdx].back();
	if (latestLackSlot < 1)
		return false;

	bool willOptimize = false;
	int downloaderIdx = 0;
	for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		int directArcPathIdx = arcPathRange[downloaderIdx].first;
		if (chosenSlots[directArcPathIdx].size() > 2)
		{
			willOptimize = true;
			break;
		}
	}
	/*
	for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		int directArcPathIdx = arcPathRange[downloaderIdx].first;
		if (chosenSlots[directArcPathIdx].size() == 1)
		{
			willOptimize = false;
			break;
		}
	}
	*/

	if (willOptimize)
	{
		int i = arcPathRange[curDownloaderIdx].first, j = 0; // i is the direct download arc path index
		int storeI = 0, storeM = 0, storeN = 0, m = 0, n = 0;
		int maxAlternativeFlow = INT32_MIN;
		size_t k = 0;
		struct Arc *marc = NULL, *narc = NULL, *collisionArc = NULL;
		// obtain currently optimal alternative relay path according to the maximize flow rule
		for (i = arcPathRange[curDownloaderIdx].first+1; i < arcPathRange[curDownloaderIdx].second; ++i)
		{
			marc = &arcTable[arcPathList[i][0]];
			narc = &arcTable[arcPathList[i][1]];
			for (m = 0; m <= latestLackSlot-1; ++m)
			{
				for (n = m+1; n <= latestLackSlot; ++n)
				{
					int alternativeFlow = __calcAlternativeFlow(i, m, n, false); // calculate alternative flow on the same arc path
					for (k = 0; k < chosenLinks[m].size(); ++k) // calculate alternative flow on the other arc paths
					{
						if (chosenPaths[m][k] == i)
							continue;
						collisionArc = &arcTable[chosenLinks[m][k]];
						alternativeFlow -= collisionArc->flow[m];
						if (collisionArc->srcID != 0) // second relay arc in the arc path
						{
							for (j = m+1; j < slotNum; ++j)
								if (collisionArc->downloader[j] == collisionArc->downloader[m])
									alternativeFlow += collisionArc->bandwidth[j] - collisionArc->flow[j];
						}
						else // first relay arc in the arc path
						{
							for (j = 0; j < m; ++j)
								if (collisionArc->downloader[j] == collisionArc->downloader[m])
									alternativeFlow += collisionArc->bandwidth[j] - collisionArc->flow[j];
						}
					}
					for (k = 0; k < chosenLinks[n].size(); ++k) // calculate alternative flow on the other arc paths
					{
						if (chosenPaths[n][k] == i)
							continue;
						collisionArc = &arcTable[chosenLinks[n][k]];
						alternativeFlow -= collisionArc->flow[n];
						if (collisionArc->srcID != 0) // second relay arc in the arc path
						{
							for (j = n+1; j < slotNum; ++j)
								if (collisionArc->downloader[j] == collisionArc->downloader[n])
									alternativeFlow += collisionArc->bandwidth[j] - collisionArc->flow[j];
						}
						else // first relay arc in the arc path
						{
							for (j = 0; j < n; ++j)
								if (collisionArc->downloader[j] == collisionArc->downloader[n])
									alternativeFlow += collisionArc->bandwidth[j] - collisionArc->flow[j];
						}
					}
					if (maxAlternativeFlow < alternativeFlow)
					{
						maxAlternativeFlow = alternativeFlow;
						storeM = m;
						storeN = n;
						storeI = i;
					}
				}
			}
		}

		i = storeI;
		m = storeM;
		n = storeN;
		marc = &arcTable[arcPathList[i][0]];
		narc = &arcTable[arcPathList[i][1]];
		EV << "path " << i << ", m " << m << ", n " << n << std::endl;

		// update chosenLinks, chosenPaths, chosenSlots
		std::vector<int> tmpChosenLink;
		std::vector<int> tmpChosenPath;
		for (k = 0; k < chosenLinks[m].size(); ++k)
		{
			int chosenPath = chosenPaths[m][k];
			collisionArc = &arcTable[chosenLinks[m][k]];
			if (!__isCollision(marc, collisionArc, m))
			{
				tmpChosenLink.push_back(chosenLinks[m][k]);
				tmpChosenPath.push_back(chosenPath);
			}
			else if (chosenPath != i)
			{
				__eraseChosenSlot(chosenSlots[chosenPath], m);
				_checkChosenSlots(collisionArc, m, chosenPath);
				collisionArc->flow[m] = 0;
				collisionArc->downloader[m] = -1;
			}
		}
		chosenLinks[m] = tmpChosenLink;
		chosenPaths[m] = tmpChosenPath;
		chosenLinks[m].push_back(marc->arcID);
		chosenPaths[m].push_back(i);
		tmpChosenLink.clear();
		tmpChosenPath.clear();
		for (k = 0; k < chosenLinks[n].size(); ++k)
		{
			int chosenPath = chosenPaths[n][k];
			collisionArc = &arcTable[chosenLinks[n][k]];
			if (!__isCollision(narc, collisionArc, n))
			{
				tmpChosenLink.push_back(chosenLinks[n][k]);
				tmpChosenPath.push_back(chosenPath);
			}
			else if (chosenPath != i)
			{
				__eraseChosenSlot(chosenSlots[chosenPath], n);
				_checkChosenSlots(collisionArc, n, chosenPath);
				collisionArc->flow[n] = 0;
				collisionArc->downloader[n] = -1;
			}
		}
		chosenLinks[n] = tmpChosenLink;
		chosenPaths[n] = tmpChosenPath;
		chosenLinks[n].push_back(narc->arcID);
		chosenPaths[n].push_back(i);

		// update flow
		maxAlternativeFlow = __calcAlternativeFlow(i, m, n, true); // flow adjustment inside, chosenSlots[chosenPath] adjustment also inside
		marc->downloader[m] = downloaderArray[curDownloaderIdx];
		narc->downloader[n] = downloaderArray[curDownloaderIdx];

		__printChosenSlots();
	}

	return willOptimize;
}

bool STAG::_optimizeReceivedAmount(const int curDownloaderIdx)
{
	ASSERT( lackSlotsTable[curDownloaderIdx].empty() );

	lackSlotsTable[curDownloaderIdx].push_back(slotNum-1);

	return _optimizeInterruptTime(curDownloaderIdx);
}

void STAG::_checkChosenSlots(struct Arc *collisionArc, const int collisionSlot, const int chosenPath)
{
	int j = 0, decreasedFlow = collisionArc->flow[collisionSlot];
	struct Arc *checkingArc = NULL;
	if (arcPathList[chosenPath].size() > 2)
	{
		if (collisionArc->srcID == 0) // scan second relay arc, checking whether need to erase slot
		{
			checkingArc = &arcTable[arcPathList[chosenPath][1]];
			for (j = slotNum-1; j > collisionSlot; --j)
			{
				if (checkingArc->flow[j] == 0)
					continue;
				if (decreasedFlow >= checkingArc->flow[j]) // need to erase this slot
				{
					decreasedFlow -= checkingArc->flow[j];
					checkingArc->flow[j] = 0;
					checkingArc->downloader[j] = -1;
					ContentUtils::vectorRemove(chosenLinks[j], checkingArc->arcID);
					ContentUtils::vectorRemove(chosenPaths[j], chosenPath);
					__eraseChosenSlot(chosenSlots[chosenPath], j);
				}
				else // needn't to erase this slot, but need to adjust flow
				{
					checkingArc->flow[j] -= decreasedFlow;
					decreasedFlow = 0;
				}
				if (decreasedFlow == 0)
					break;
			}
		}
		else // scan first relay arc, checking whether need to erase slot
		{
			checkingArc = &arcTable[arcPathList[chosenPath][0]];
			for (j = 0; j < collisionSlot; ++j)
			{
				if (checkingArc->flow[j] == 0)
					continue;
				if (decreasedFlow >= checkingArc->flow[j]) // need to erase this slot
				{
					decreasedFlow -= checkingArc->flow[j];
					checkingArc->flow[j] = 0;
					checkingArc->downloader[j] = -1;
					ContentUtils::vectorRemove(chosenLinks[j], checkingArc->arcID);
					ContentUtils::vectorRemove(chosenPaths[j], chosenPath);
					__eraseChosenSlot(chosenSlots[chosenPath], j);
				}
				else // needn't to erase this slot, but need to adjust flow
				{
					checkingArc->flow[j] -= decreasedFlow;
					decreasedFlow = 0;
				}
				if (decreasedFlow == 0)
					break;
			}
		}
	}
}

void STAG::_calcNodeStorage()
{
#if DEBUG_STAG
	EV << "===== chosen links =====\n";
#endif
	int j = 0, downloaderIdx = 0;
	for (j = 0; j < slotNum; ++j)
	{
		std::vector<int> &chosenLink = chosenLinks[j];
#if DEBUG_STAG
			EV << "slot " << j+1 << ":";
			for (size_t l = 0; l < chosenLink.size(); ++l)
				EV << " " << chosenLink[l] << "(" << arcTable[chosenLink[l]].srcID << "->" << arcTable[chosenLink[l]].dstID << ")";
			EV << std::endl;
		}
#endif

		// handle with data storage process
		for (int i = 0; i <= nodeNum; ++i)
			for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
				NodeList[i].S[j+1][downloaderIdx] = NodeList[i].S[j][downloaderIdx];

		// handle with data transmition process
		for (size_t k = 0; k < chosenLink.size(); ++k)
		{
			struct Arc *parc = &arcTable[chosenLink[k]];
			downloaderIdx = downloaderTable[parc->downloader[j]];
			parc->flow[j] = std::min(parc->flow[j], NodeList[parc->srcID].S[j][downloaderIdx].size);
			NodeList[parc->srcID].S[j+1][downloaderIdx].size -= parc->flow[j];
			NodeList[parc->dstID].S[j+1][downloaderIdx].size += parc->flow[j];
		}

		// handle with video play process of downloaders
		for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
		{
			int downloader = downloaderArray[downloaderIdx];
			int playRate = playTable[downloader];
			if (NodeList[downloader].S[j+1][downloaderIdx].size >= playRate)
			{
				NodeList[superSink].S[j+1][downloaderIdx].size += playRate;
				NodeList[downloader].S[j+1][downloaderIdx].size -= playRate;
			}
			else
			{
				NodeList[superSink].S[j+1][downloaderIdx].size += NodeList[downloader].S[j+1][downloaderIdx].size;
				NodeList[downloader].S[j+1][downloaderIdx].size = 0;
				lackSlotsTable[downloaderIdx].push_back(j);
			}
		}
	}

	for (j = 1; j <= slotNum; ++j)
		for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
			NodeList[0].S[j][downloaderIdx].size -= NodeList[0].S[slotNum][downloaderIdx].size;

	// __printChannelStatus();

	__printNodeStorage();
}

void STAG::_clearNodeStorage()
{
	for (int i = 0; i <= nodeNum; ++i)
		for (int j = 1; j <= slotNum; ++j)
			for (int d = 0; d < downloaderNum; ++d)
				NodeList[i].S[j][d].size = 0;
	for (int downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
		lackSlotsTable[downloaderIdx].clear();
}

void STAG::_obtainSchemeIndicators(double& totalInterruptTime, int& totalReceivedAmount)
{
	totalInterruptTime = 0.0;
	totalReceivedAmount = 0;

	for (int downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		int downloader = downloaderArray[downloaderIdx];
		int playRate = playTable[downloader];
		int receivedAmount  = NodeList[downloader].S[slotNum][downloaderIdx].size - NodeList[downloader].S[0][downloaderIdx].size + NodeList[superSink].S[slotNum][downloaderIdx].size;
		int lack = slotNum*playRate - NodeList[superSink].S[slotNum][downloaderIdx].size;
		double interruptTime = static_cast<double>(lack) / playRate;
		interruptTimeTable[downloaderIdx] = interruptTime;
		receivedAmountTable[downloaderIdx] = receivedAmount;
		totalInterruptTime += interruptTime;
		totalReceivedAmount += receivedAmount;
#if INFO_STAG
		EV << "Interrupt time of downloader [" << downloader << "] is " << interruptTime << " delta, received data amount of it is " << receivedAmount << ".\n";
#endif
	}
#if INFO_STAG
	EV << "Current total interrupt time is " << totalInterruptTime << ", total received data amount is " << totalReceivedAmount << ".\n";
#endif
}

void STAG::_recordFluxScheme()
{
	__eraseUnchosenArcPath(); // execute this for efficiency of the following operations

	std::vector<FluxScheme> vecPadding;
	vecPadding.reserve(slotNum);
	for (int l = 0; l < nodeNum; ++l)
		fluxSchemeList.push_back(vecPadding);

	size_t k = 0;
	int pathIdx = 0;
	std::vector<std::pair<int /* slot */, int /* flow */> > sortReceivedSlot;
	std::vector<std::pair<int /* slot */, int /* path index */> > sortReceivedPath;
	sortReceivedSlot.reserve(slotNum);
	sortReceivedPath.reserve(slotNum);
	for (int downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		std::vector<Segment> segmentOffsets(slotNum);
		std::vector<Segment> dupSegmentOffsets(slotNum); // avoid modifying segmentOffsets in operations
		for (pathIdx = arcPathRange[downloaderIdx].first; pathIdx < arcPathRange[downloaderIdx].second; ++pathIdx)
		{
			for (k = 0; k < arcPathList[pathIdx].size()-1; ++k)
			{
				struct Arc *parc = &arcTable[arcPathList[pathIdx][k]];
				for (int j = 0; j < slotNum; ++j)
				{
					if (parc->flow[j] > 0 && parc->dstID == parc->downloader[j]) // && parc->downloader[j] == arcPathList[pathIdx].back()
					{
						sortReceivedSlot.push_back(std::pair<int, int>(j, parc->flow[j]));
						sortReceivedPath.push_back(std::pair<int, int>(j, pathIdx));
					}
				}
			}
		}
		std::sort(sortReceivedSlot.begin(), sortReceivedSlot.end());
		int accumulatedOffset = 0;
#if INFO_STAG
		EV << "arrival (slot,amount)[offset]:";
#endif
		for (k = 0; k < sortReceivedSlot.size(); ++k)
		{
			segmentOffsets[sortReceivedSlot[k].first].begin = accumulatedOffset;
			dupSegmentOffsets[sortReceivedSlot[k].first].begin = accumulatedOffset;
			accumulatedOffset += sortReceivedSlot[k].second;
			segmentOffsets[sortReceivedSlot[k].first].end = accumulatedOffset;
			dupSegmentOffsets[sortReceivedSlot[k].first].end = accumulatedOffset;
#if INFO_STAG
			EV << " (" << sortReceivedSlot[k].first+1 << ',' << sortReceivedSlot[k].second << ")[" << segmentOffsets[sortReceivedSlot[k].first].begin << ',' << segmentOffsets[sortReceivedSlot[k].first].end << ']';
		}
		EV << "\n";
#else
		}
#endif
		prefetchAmountTable.insert(std::pair<int, int>(downloaderArray[downloaderIdx], accumulatedOffset));

		// modify prefetching amount if necessary in order to avoid exceeding total content size
#if INFO_STAG
		EV << "prefetch amount is " << accumulatedOffset << " bytes, demanding amount is " << demandingAmountTable[downloaderArray[downloaderIdx]] << " bytes.\n";
#endif
		int decreasingAmount = prefetchAmountTable[downloaderArray[downloaderIdx]] - demandingAmountTable[downloaderArray[downloaderIdx]]; // alias
		if (decreasingAmount > 0)
		{
			prefetchAmountTable[downloaderArray[downloaderIdx]] = demandingAmountTable[downloaderArray[downloaderIdx]];
			// in order to modify flux scheme list, thus remove slot scheme in segmentOffsets reversely by slot index
			std::sort(sortReceivedPath.begin(), sortReceivedPath.end());
			int sIdx = static_cast<int>(sortReceivedPath.size()) - 1;
			while (decreasingAmount > 0)
			{
				int slot_ = sortReceivedPath[sIdx].first, path_ = sortReceivedPath[sIdx].second; // alias
				int segmentLength = sortReceivedSlot[sIdx].second; // alias
				if (segmentLength <= decreasingAmount)
				{
					segmentOffsets[slot_].begin = segmentOffsets[slot_].end = -1;
					dupSegmentOffsets[slot_].begin = dupSegmentOffsets[slot_].end = -1;
					decreasingAmount -= segmentLength;
					if (arcPathList[path_].size() == 2) // direct download arc path
						arcTable[arcPathList[path_][0]].flow[slot_] = 0;
					else // relay arc path
					{
						struct Arc *marc = &arcTable[arcPathList[path_][0]];
						arcTable[arcPathList[path_][1]].flow[slot_] = 0;
						for (int j = slotNum-1; j >= 0; --j)
						{
							if (marc->flow[j] == 0)
								continue;
							if (marc->flow[j] < segmentLength)
							{
								segmentLength -= marc->flow[j];
								marc->flow[j] = 0;
							}
							else
							{
								marc->flow[j] -= segmentLength;
								segmentLength = 0;
								break;
							}
						}
					}
					--sIdx;
				}
				else
				{
					segmentOffsets[slot_].end -= decreasingAmount;
					dupSegmentOffsets[slot_].end -= decreasingAmount;
					if (arcPathList[path_].size() == 2) // direct download arc path
					{
						arcTable[arcPathList[path_][0]].flow[slot_] -= decreasingAmount;
						decreasingAmount = 0;
					}
					else // relay arc path
					{
						struct Arc *marc = &arcTable[arcPathList[path_][0]];
						arcTable[arcPathList[path_][1]].flow[slot_] -= decreasingAmount;
						for (int j = slotNum-1; j >= 0; --j)
						{
							if (marc->flow[j] == 0)
								continue;
							if (marc->flow[j] < decreasingAmount)
							{
								decreasingAmount -= marc->flow[j];
								marc->flow[j] = 0;
							}
							else
							{
								marc->flow[j] -= decreasingAmount;
								decreasingAmount = 0;
								break;
							}
						}
					}
				} // if (segmentLength <= decreasingAmount)
			}
		}

		pathIdx = arcPathRange[downloaderIdx].first; // handle with the direct download arc path
		for (k = 0; k < arcPathList[pathIdx].size()-1; ++k)
		{
			struct Arc *parc = &arcTable[arcPathList[pathIdx][k]];
			for (int j = 0; j < slotNum; ++j)
			{
				if (parc->flow[j] > 0)
				{
					fluxSchemeList[parc->srcID].push_back(FluxScheme(j+1, parc->dstID, parc->downloader[j], parc->flow[j]));
					fluxSchemeList[parc->srcID].back().segment.assign(&segmentOffsets[j]);
					fluxSchemeList[parc->srcID].back().segment.print();
				}
			}
		}
		for (pathIdx = arcPathRange[downloaderIdx].first+1; pathIdx < arcPathRange[downloaderIdx].second; ++pathIdx) // handle with the relay arc path
		{
			struct Arc *marc = &arcTable[arcPathList[pathIdx][0]], *narc = &arcTable[arcPathList[pathIdx][1]];
			std::vector<std::pair<int /* slot */, int /* flow */> > marcSlotFlow, narcSlotFlow;
			for (int j = 0; j < slotNum; ++j)
			{
				if (marc->flow[j] > 0)
					marcSlotFlow.push_back(std::pair<int, int>(j, marc->flow[j]));
				if (narc->flow[j] > 0)
					narcSlotFlow.push_back(std::pair<int, int>(j, narc->flow[j]));
			}
			bool uncontinuousAppend = true;
			Segment *mSeg = &segmentOffsets[marcSlotFlow[0].first], *nSeg = &dupSegmentOffsets[narcSlotFlow[0].first];
			for (size_t m = 0, n = 0; m < marcSlotFlow.size() && n < narcSlotFlow.size();)
			{
				int mflow = marcSlotFlow[m].second, nflow = narcSlotFlow[n].second; // alias
				if (mflow < nflow)
				{
					if (uncontinuousAppend)
						mSeg->end = mSeg->begin = nSeg->begin;
					mSeg->end += mflow;
					nSeg->begin += mflow;
					narcSlotFlow[n].second -= mflow;
					mSeg = &segmentOffsets[marcSlotFlow[++m].first];
					uncontinuousAppend = true;
				}
				else if (mflow > nflow)
				{
					if (uncontinuousAppend)
						mSeg->end = mSeg->begin = nSeg->begin;
					mSeg->end += nflow;
					marcSlotFlow[m].second -= nflow;
					nSeg = &dupSegmentOffsets[narcSlotFlow[++n].first];
					if (mSeg->end < nSeg->begin) // uncontinuous segment, append to next field
					{
						mSeg->next = new Segment;
						mSeg = mSeg->next;
						uncontinuousAppend = true;
					}
					else // (mSeg->end == nSeg->begin), concat continuous segment to the existing segment
						uncontinuousAppend = false;
				}
				else // (mflow == nflow)
				{
					if (uncontinuousAppend)
						mSeg->begin = nSeg->begin;
					mSeg->end = nSeg->end;
					if (++m < marcSlotFlow.size() && ++n < narcSlotFlow.size())
					{
						mSeg = &segmentOffsets[marcSlotFlow[m].first];
						nSeg = &dupSegmentOffsets[narcSlotFlow[n].first];
						uncontinuousAppend = true;
					}
				}
			}
			for (int j = 0; j < slotNum; ++j)
			{
				if (marc->flow[j] > 0)
				{
					fluxSchemeList[marc->srcID].push_back(FluxScheme(j+1, marc->dstID, marc->downloader[j], marc->flow[j]));
					fluxSchemeList[marc->srcID].back().segment.assign(&segmentOffsets[j]);
					fluxSchemeList[marc->srcID].back().segment.print();
				}
				if (narc->flow[j] > 0)
				{
					fluxSchemeList[narc->srcID].push_back(FluxScheme(j+1, narc->dstID, narc->downloader[j], narc->flow[j]));
					fluxSchemeList[narc->srcID].back().segment.assign(&segmentOffsets[j]);
					fluxSchemeList[narc->srcID].back().segment.print();
				}
			}
		}

		sortReceivedSlot.clear();
		sortReceivedPath.clear();
	}

	for (int i = 0; i < nodeNum; ++i)
	{
		std::sort(fluxSchemeList[i].begin(), fluxSchemeList[i].end(), FluxSchemeCmp());
#if INFO_STAG
		EV << i << " |";
		for (k = 0; k < fluxSchemeList[i].size(); ++k)
			EV << " " << fluxSchemeList[i][k].slot << " " << fluxSchemeList[i][k].dest << " " << fluxSchemeList[i][k].downloader << " " << fluxSchemeList[i][k].flow;
		EV << "\n";
#endif
	}
}

void STAG::_recordBetterScheme()
{
	for (itAPL = arcPathList.begin(); itAPL != arcPathList.end(); ++itAPL)
	{
		std::vector<int> &arcPath = *itAPL; // alias
		for (size_t i = 0; i < arcPath.size()-1; ++i)
		{
			struct Arc *parc = &arcTable[arcPath[i]];
			for (int j = 0; j < slotNum; ++j)
			{
				parc->flow[j+slotNum] = parc->flow[j];
				parc->downloader[j+slotNum] = parc->downloader[j];
			}
		}
	}
}

void STAG::_revertBestScheme()
{
	for (itAPL = arcPathList.begin(); itAPL != arcPathList.end(); ++itAPL)
	{
		std::vector<int> &arcPath = *itAPL; // alias
		for (size_t i = 0; i < arcPath.size()-1; ++i)
		{
			struct Arc *parc = &arcTable[arcPath[i]];
			for (int j = 0; j < slotNum; ++j)
			{
				parc->flow[j] = parc->flow[j+slotNum];
				parc->downloader[j] = parc->downloader[j+slotNum];
			}
		}
	}
}

/** add the super sink node, this function is called by maximumFlow(). */
void STAG::_addSuperSink()
{
	struct Arc *parc = NULL;

	int arcID = 2*linkNum;

	for (std::map<int, int>::iterator itPT = playTable.begin(); itPT != playTable.end(); ++itPT)
	{
		int curNode = itPT->first; // downloader node ID

		parc = &arcTable[arcID++];
		parc->srcID = curNode;
		parc->dstID = superSink; // super sink node
		for (int i = 0; i < slotNum; ++i)
			parc->bandwidth[i] = itPT->second;
		parc->nextArc = NULL;

		// if (NodeList[curNode].firstArc == NULL)
		//  NodeList[curNode].firstArc = parc;
		// else
		//  NodeList[curNode].lastArc->nextArc = parc; // append to the last
		// NodeList[curNode].lastArc = parc;  // update lastArc too
	}

	for (int j = 0; j < downloaderNum; ++j)
	{
		NodeList[superSink].S[0][j].downloader = NodeList[0].S[0][j].downloader;
		NodeList[superSink].S[0][j].size = 0;
	}
}

/** delete the super sink node, this function is called by maximumFlow(). */
void STAG::_delSuperSink()
{
	/*
	for (std::map<int, int>::iterator itPT = playTable.begin(); itPT != playTable.end(); ++itPT)
	{
		struct Arc *freearc = NodeList[itPT->first].firstArc, *prevarc = NULL;

		while (freearc->nextArc != NULL)
		{
			prevarc = freearc;
			freearc = freearc->nextArc;
		}

		if (NodeList[itPT->first].firstArc->nextArc == NULL)
		{
			NodeList[itPT->first].firstArc = NULL;
			NodeList[itPT->first].lastArc = NULL;
		}
		else
		{
			prevarc->nextArc = NULL;
			NodeList[itPT->first].lastArc = prevarc;
		}
	}
	*/
}

int STAG::__calcAlternativeFlow(const int curPath, const int m, const int n, const bool setFlow)
{
	struct Arc *marc = &arcTable[arcPathList[curPath][0]], *narc = &arcTable[arcPathList[curPath][1]];
	std::vector<int> chosenSlot = chosenSlots[curPath]; // value copy
	int i = 0, preFlow = 0, postFlow = 0, firstArcFlow = 0, secondArcFlow = 0;
	int indicator = chosenSlot.back(), vecSize = static_cast<int>(chosenSlot.size());
	bool collisionM = false, collisionN = false, erasureM = false, erasureN = false;
	for (i = 0; i < indicator; ++i)
	{
		firstArcFlow += marc->flow[chosenSlot[i]];
		if (chosenSlot[i] == m)
			collisionM = true;
		else if (chosenSlot[i] == n)
			erasureN = true;
	}
	for (i = indicator; i < vecSize-1; ++i)
	{
		secondArcFlow += narc->flow[chosenSlot[i]];
		if (chosenSlot[i] == n)
			collisionN = true;
		else if (chosenSlot[i] == m)
			erasureM = true;
	}

	ASSERT(firstArcFlow == secondArcFlow);
	preFlow = firstArcFlow;

	if (erasureM)
		__eraseChosenSlot(chosenSlot, m);
	if (erasureN)
		__eraseChosenSlot(chosenSlot, n);
	if (!collisionM)
		__insertChosenSlot(chosenSlot, m, true);
	if (!collisionN)
		__insertChosenSlot(chosenSlot, n, false);

#if DEBUG_STAG
	__printChosenSlot(chosenSlot);
#endif

	postFlow = __calcChosenPathPostFlow(chosenSlot, marc, narc, setFlow);
	if (setFlow)
		chosenSlots[curPath] = chosenSlot; // assign back

	return postFlow - preFlow;
}

int STAG::__calcChosenPathPostFlow(std::vector<int>& chosenSlot, struct Arc *&marc, struct Arc *&narc, const bool setFlow)
{
	int maxFlow = 0;
	int indicator = chosenSlot.back(), vecSize = static_cast<int>(chosenSlot.size());
	int i = 0, j = 0, firstArcBW = 0, secondArcBW = 0, firstArcRestCap = 0, secondArcRestCap = 0;
	int *marcBandwidth = new int[slotNum];
	int *narcBandwidth = new int[slotNum];
	memcpy(marcBandwidth, marc->bandwidth, slotNum*sizeof(int));
	memcpy(narcBandwidth, narc->bandwidth, slotNum*sizeof(int));

	for (i = 0; i < indicator; ++i)
		firstArcBW += marcBandwidth[chosenSlot[i]];
	for (j = indicator; j < vecSize-1; ++j)
		secondArcBW += narcBandwidth[chosenSlot[j]];

	for (i = 0, j = indicator; i < indicator && j < vecSize-1;)
	{
		while (chosenSlot[i] > chosenSlot[j] && j < vecSize-1)
			++j;
		if (j == vecSize-1)
			break;
		if (marcBandwidth[chosenSlot[i]] < narcBandwidth[chosenSlot[j]])
		{
			narcBandwidth[chosenSlot[j]] -= marcBandwidth[chosenSlot[i]];
			marcBandwidth[chosenSlot[i]] = 0;
			++i;
		}
		else if (marcBandwidth[chosenSlot[i]] > narcBandwidth[chosenSlot[j]])
		{
			marcBandwidth[chosenSlot[i]] -= narcBandwidth[chosenSlot[j]];
			narcBandwidth[chosenSlot[j]] = 0;
			++j;
		}
		else
		{
			marcBandwidth[chosenSlot[i]] = narcBandwidth[chosenSlot[j]] = 0;
			++i;
			++j;
		}
	}

	for (i = 0; i < indicator; ++i)
		firstArcRestCap += marcBandwidth[chosenSlot[i]];
	for (j = indicator; j < vecSize-1; ++j)
		secondArcRestCap += narcBandwidth[chosenSlot[j]];

	ASSERT(firstArcBW - firstArcRestCap == secondArcBW - secondArcRestCap);
	maxFlow = firstArcBW - firstArcRestCap;

	if (setFlow)
	{
        memset(marc->flow, 0, slotNum*sizeof(int));
        memset(narc->flow, 0, slotNum*sizeof(int));

		for (i = 0; i < indicator; ++i)
			marc->flow[chosenSlot[i]] = marc->bandwidth[chosenSlot[i]] - marcBandwidth[chosenSlot[i]];

		for (j = indicator; j < vecSize-1; ++j)
			narc->flow[chosenSlot[j]] = narc->bandwidth[chosenSlot[j]] - narcBandwidth[chosenSlot[j]];
	}

	delete []marcBandwidth;
	delete []narcBandwidth;
	return maxFlow;
}

void STAG::__printNodeStorage()
{
#if DEBUG_STAG
	for (int i = 0; i <= nodeNum; ++i)
	{
		EV << "node [" << i << "]:  " << downloaderArray[0] << "$(" << NodeList[i].S[0][0].size;
		int j = 0;
		for (j = 1; j <= slotNum; ++j)
			EV << "," << NodeList[i].S[j][0].size;
		for (int d = 1; d < downloaderNum; ++d)
		{
			EV << ")  |  " << downloaderArray[d] << "$(" << NodeList[i].S[0][d].size;
			for (j = 1; j <= slotNum; ++j)
				EV << "," << NodeList[i].S[j][d].size;
		}
		EV << ")\n";
	}
#endif
}

void STAG::__printChannelStatus()
{
#if DEBUG_STAG
	for (int i = 0; i < nodeNum; i++)
	{
		EV << "node [" << i << "]";
		for (struct Arc *darc = NodeList[i].firstArc; darc != NULL; darc = darc->nextArc)
		{
			EV << " -> " << darc->dstID << "(";
			for (int j = 0; j < slotNum-1; ++j)
				EV << darc->idle[j] << ",";
			EV << darc->idle[slotNum-1] << ")";
		}
		EV << std::endl;
	}
#endif
}

void STAG::__printChosenSlots()
{
#if INFO_STAG
	EV << "===== chosen slots =====\n";
	for (size_t k = 0; k < chosenSlots.size(); ++k)
	{
		EV << "path " << k << ":";
		__printChosenSlot(chosenSlots[k]);
	}
#endif
}

void STAG::__printChosenSlot(std::vector<int>& vec)
{
	ASSERT(!vec.empty());

#if INFO_STAG
	int i = 0, indicator = vec.back(), vecSize = static_cast<int>(vec.size());
	if (indicator != -1) // relay arc path
	{
		for (i = 0; i < indicator; ++i)
		{
			EV << " " << vec[i]+1;
			if (i == indicator-1)
				EV << " |";
		}
		for (i = indicator; i < vecSize-1; ++i)
		{
			EV << " " << vec[i]+1;
			if (i == vecSize-2)
				EV << " |";
		}
		EV << " " << indicator;
	}
	else // direct arc path
	{
		for (i = 0; i < vecSize-1; ++i)
		{
			EV << " " << vec[i]+1;
			if (i == vecSize-2)
                EV << " |";
		}
		EV << " " << indicator;
	}
	EV << std::endl;
#endif
}

void STAG::__eraseChosenSlot(std::vector<int>& vec, const int slot)
{
	int i = 0, pos = -1, vecSize = static_cast<int>(vec.size());
	int &indicator = vec.back();
	for (i = 0; i < indicator; ++i)
	{
		if (vec[i] == slot)
		{
			pos = i;
			--indicator;
			break;
		}
	}
	if (pos == -1)
	{
		int startIdx = indicator == -1 ? 0 : indicator;
		for (i = startIdx; i < vecSize-1; ++i)
		{
			if (vec[i] == slot)
			{
				pos = i;
				break;
			}
		}
	}
	if (pos != -1)
	{
		for (i = pos; i < vecSize-1; ++i)
			vec[i] = vec[i+1];
		vec.pop_back();
	}
	else
	{
		EV_ERROR << "erase the slot that hasn't been chosen!\n";
		ASSERT(false);
	}
}

void STAG::__insertChosenSlot(std::vector<int>& vec, const int slot, bool frontArc)
{
	int i = 0, pos = -1, vecSize = static_cast<int>(vec.size());
	for (i = 0; i < vecSize-1; ++i)
	{
		if (vec[i] == slot)
		{
			EV_ERROR << "insert the same slot as the slot that has been chosen!\n";
			ASSERT(false);
		}
	}

	int &indicator = vec.back();
	if (frontArc)
	{
		for (i = 0; i < indicator; ++i)
		{
			if (slot < vec[i])
			{
				pos = i;
				break;
			}
		}
		if (pos == -1)
			pos = indicator;
		++indicator;
	}
	else
	{
		int startIdx = indicator == -1 ? 0 : indicator;
		for (i = startIdx; i < vecSize-1; ++i)
		{
			if (slot < vec[i])
			{
				pos = i;
				break;
			}
		}
		if (pos == -1)
			pos = vecSize - 1;
	}
	vec.push_back(slot);
	++vecSize;
	for (i = vecSize-1; i > pos; --i)
		vec[i] = vec[i-1];
	vec[pos] = slot;
}

void STAG::__eraseUnchosenArcPath()
{
	std::vector<size_t> chosenPathIndices, unChosenPathIndices;
	chosenPathIndices.reserve(arcPathList.size());
	unChosenPathIndices.reserve(arcPathList.size());
	int downloaderIdx = 0;
	for (size_t pathIdx = 0; pathIdx < arcPathList.size(); ++pathIdx)
	{
		if (downloaderIdx < downloaderNum && static_cast<int>(pathIdx) == arcPathRange[downloaderIdx].first) // don't erase direct download link even if it is not chosen
		{
			++downloaderIdx;
			chosenPathIndices.push_back(pathIdx);
			continue;
		}

		bool isChosen = false;
		for (size_t k = 0; k < arcPathList[pathIdx].size()-1; ++k)
		{
			struct Arc *parc = &arcTable[arcPathList[pathIdx][k]];
			for (int j = 0; j < slotNum; ++j)
			{
				if (parc->flow[j] > 0 && parc->downloader[j] == arcPathList[pathIdx].back())
				{
					isChosen = true;
					break;
				}
			}
			if (isChosen)
				break;
		}
		if (isChosen)
			chosenPathIndices.push_back(pathIdx);
		else
			unChosenPathIndices.push_back(pathIdx);
	}

	int cPathSize = static_cast<int>(chosenPathIndices.size()), ucPathSize = static_cast<int>(unChosenPathIndices.size());
	int cPathIdx = cPathSize - 1, ucPathIdx = 0;
	std::vector<size_t> reverseIndices; // used to correct the order after swap
	while (cPathIdx >= 0 && ucPathIdx < ucPathSize)
	{
		size_t pathIdxC = chosenPathIndices[cPathIdx];
		size_t pathIdxUC = unChosenPathIndices[ucPathIdx];
		if (pathIdxUC < pathIdxC)
		{
			arcPathList[pathIdxUC].swap(arcPathList[pathIdxC]);
			reverseIndices.push_back(pathIdxUC); // after swap operation, path indices at original unchosen positions must in the reverse order
			++ucPathIdx;
			--cPathIdx;
		}
		else
			break;
	}
	while (ucPathSize-- > 0) // remove unchosen paths as they all have been swapped to the tail of vector
		arcPathList.pop_back();

	int reverseSize = static_cast<int>(reverseIndices.size());
	for (int m = 0, n = reverseSize - 1; m < n && m < reverseSize && n >= 0; ++m, --n) // adopt swap idiom to correct the reverse order
		arcPathList[reverseIndices[m]].swap(arcPathList[reverseIndices[n]]);

	EV << "===== chosen arc path list =====\n";
	for (itAPL = arcPathList.begin(); itAPL != arcPathList.end(); ++itAPL)
	{
		for (size_t k = 0; k < itAPL->size(); ++k)
			EV << itAPL->at(k) << " ";
		EV << std::endl;
	}

	// update arc path range for each downloader
	int arcPathListSize = static_cast<int>(arcPathList.size());
	downloaderIdx = 0;
	for (int i = 1; i < arcPathListSize; ++i)
	{
		if (arcPathList[i].back() != arcPathList[i-1].back())
		{
			arcPathRange[downloaderIdx++].second = i;
			arcPathRange[downloaderIdx].first = i;
		}
	}
	arcPathRange[downloaderIdx].second = arcPathListSize;

#if INFO_STAG
	EV << "===== chosen arc path range =====\n";
	for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
		EV << "downloader " << downloaderArray[downloaderIdx] << ": [" << arcPathRange[downloaderIdx].first << "," << arcPathRange[downloaderIdx].second << ")\n";
#endif
}

bool STAG::__isCollision(struct Arc *parc, struct Arc *qarc, const int slot)
{
	return true;
}

