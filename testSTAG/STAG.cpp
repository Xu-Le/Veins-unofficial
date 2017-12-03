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

#include "STAG.h"

extern int log_level;

int nodeNum = 0;
int linkNum = 0;
int downloaderNum = 0;
int slotNum = 0;

/** maintaining the map from node ID to its initial storage. */
std::map<int, std::string> storageTable;
/** maintaining the map from downloader node ID to its index. */
std::map<int, int> downloaderTable;
/** reverse table of downloaderTable, cost only O(1). */
std::vector<int> downloaderArray;

/** automatically increase ID for all arcs. */
static int arcIDIncrement = 0;

Arc::Arc() : arcID(arcIDIncrement), srcID(-1), dstID(-1), nextArc(NULL)
{
	bandwidth = new int[slotNum];
	flow = new int[2*slotNum];       // index range [slotNum, 2*slotNum-1) is used for recording purpose
	downloader = new int[2*slotNum]; // index range [slotNum, 2*slotNum-1) is used for recording purpose
	idle = new bool[slotNum];
	memset(flow, 0, 2*slotNum*sizeof(int));
	memset(downloader, -1, 2*slotNum*sizeof(int));
	memset(idle, true, slotNum*sizeof(bool));
	++arcIDIncrement;
}

Arc::~Arc()
{
	delete []bandwidth;
	delete []flow;
	delete []downloader;
	delete []idle;
}

Node::Node() : firstArc(NULL), lastArc(NULL)
{
	_S = new struct Data[(slotNum+1)*downloaderNum];
	S = new struct Data*[slotNum+1];
	for (int i = 0; i < slotNum+1; ++i)
		S[i] = _S + i*downloaderNum;
}

Node::~Node()
{
	delete []S;
	delete []_S;
}

/** constructor of class STAG. */
STAG::STAG()
{
	NodeList = new Node[nodeNum];
	arcTable = new Arc[2*linkNum];
	arcRecord = new Arc*[nodeNum];
	dist = new int[nodeNum];
	f = new int[nodeNum];
	pi = new int[nodeNum];
}

/** destructor of class STAG. */
STAG::~STAG()
{
	delete []pi;
	delete []dist;
	delete []f;
	delete []arcRecord;
	delete []arcTable;
	delete []NodeList;
}

/** construct the graph in the structure of adjacency list. */
void STAG::construct(std::list<LinkTuple>& linkTuples)
{
	std::list<struct LinkTuple>::iterator iter1 = linkTuples.begin(), iter2;

	int curNode = 0, rank = 2;
	struct Arc *firstarc = NULL, *parc = NULL, *qarc = NULL;

	int arcID = 0;
	firstarc = &arcTable[arcID];
	firstarc->srcID = iter1->src;
	firstarc->dstID = iter1->dst;
	memcpy(firstarc->bandwidth, iter1->bandwidth, slotNum*sizeof(int));
	firstarc->nextArc = NULL;

	curNode = iter1->src; // node index start with 0
	_initNodeStorage(curNode);
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
			_initNodeStorage(curNode);
			NodeList[curNode].firstArc = firstarc;
			NodeList[curNode].lastArc = firstarc;
		}
		++iter1;
	}

	for (int i = 0; i < nodeNum; ++i)
		if (NodeList[i].firstArc == NULL)
			_initNodeStorage(i);
}

/** internal function called by function construct(). */
void STAG::_initNodeStorage(int curNode)
{
	std::string &str = storageTable[curNode];
	std::string subStr;
	size_t old = 0, cur = 0;
	int idx = 0;
	while (cur < str.length())
	{
		if (str[cur] == ' ')
		{
			subStr = str.substr(old, cur-old);
			if (idx % 2 == 0)
				NodeList[curNode].S[0][idx/2].downloader = atoi(subStr.c_str());
			else
				NodeList[curNode].S[0][idx/2].size = atoi(subStr.c_str());
			++idx;
			old = ++cur;
		}
		else
			++cur;
	}
	if (!str.empty())
	{
		subStr = str.substr(old, cur-old);
		NodeList[curNode].S[0][idx/2].size = atoi(subStr.c_str());
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

/** display the graph whose structure is adjacency list. */
void STAG::display()
{
	if (log_level < DEBUG_LEVEL)
		return;

	printf("STAG's adjacency list displays: \n");
	for (int i = 0; i < nodeNum; i++)
	{
		printf("node [%d]", i);
		for (struct Arc *darc = NodeList[i].firstArc; darc != NULL; darc = darc->nextArc)
		{
			printf(" -> %d(", darc->dstID);
			for (int j = 0; j < slotNum-1; ++j)
				printf("%d,", darc->bandwidth[j]);
			printf("%d)", darc->bandwidth[slotNum-1]);
		}
		printf("\n");
	}
}

int STAG::maximumFlow()
{
	receivedAmountTable.resize(downloaderNum);

	_obtainArcPathList();

	_obtainInitialScheme();

	_calcNodeStorage();

	int totalReceivedAmount = _obtainSchemeIndicators();
	int maxTotalReceivedAmount = totalReceivedAmount;

	if (nodeNum > 2)
	{
		_recordBetterScheme();

		for (int loop = 0; loop < downloaderNum*slotNum/2; ++loop)
		{
			info_log("===============    Loop %d    ===============\n", loop + 1);

			bool continueLoop = _seekBetterScheme();
			if (!continueLoop)
				break;

			_clearNodeStorage();

			_calcNodeStorage();

			totalReceivedAmount = _obtainSchemeIndicators();

			// if indeed get a better scheme, then record this better scheme
			if (maxTotalReceivedAmount < totalReceivedAmount)
			{
				maxTotalReceivedAmount = totalReceivedAmount;
				_recordBetterScheme();
			}
		}

		_revertBestScheme();
	}

#if VERIFY_SCHEME
	_recordFluxPath();
#endif

	_recordFluxScheme();

	_clearNodeStorage();

	return maxTotalReceivedAmount;
}

void STAG::test()
{
	// test __calcChosenPathPostFlow()
#if 1
	std::vector<int> cslots({0, 1, 3, 4, 2, 5, 6, 7, 4});

	struct Arc *marc = new struct Arc;
	struct Arc *narc = new struct Arc;
	marc->bandwidth[0] = 5; marc->bandwidth[1] = 11; marc->bandwidth[2] = 0; marc->bandwidth[3] = 25;
	marc->bandwidth[4] = 10; marc->bandwidth[5] = 0; marc->bandwidth[6] = 0; marc->bandwidth[7] = 0;
	narc->bandwidth[0] = 0; narc->bandwidth[1] = 0; narc->bandwidth[2] = 18; narc->bandwidth[3] = 0;
	narc->bandwidth[4] = 0; narc->bandwidth[5] = 11; narc->bandwidth[6] = 11; narc->bandwidth[7] = 12;
	for (int j = 0; j < slotNum; ++j)
		marc->flow[j] = narc->flow[j] = 1;
	marc->downloader[5] = narc->downloader[3] = 2;
	__calcChosenPathPostFlow(cslots, marc, narc, true);

	debug_log("marc->flow:");
	int i = 0;
	for (i = 0; i < slotNum; ++i)
		printf(" %d", marc->flow[i]);
	printf("\n");
	debug_log("narc->flow:");
	for (i = 0; i < slotNum; ++i)
		printf(" %d", narc->flow[i]);
	printf("\n");
#endif

#if 0
	std::vector<int> cslots({0, 1, 2, 3, 4, 5, 6, 4});

	struct Arc *marc = new struct Arc;
	struct Arc *narc = new struct Arc;
	marc->bandwidth[0] = 601600; marc->bandwidth[1] = 524800; marc->bandwidth[2] = 332800; marc->bandwidth[3] = 204800;
	marc->bandwidth[4] = 89600; marc->bandwidth[5] = 51200; marc->bandwidth[6] = 38400; marc->bandwidth[7] = 38400;
	narc->bandwidth[0] = 665600; narc->bandwidth[1] = 665600; narc->bandwidth[2] = 665600; narc->bandwidth[3] = 665600;
	narc->bandwidth[4] = 665600; narc->bandwidth[5] = 665600; narc->bandwidth[6] = 665600; narc->bandwidth[7] = 665600;
	for (int j = 0; j < slotNum; ++j)
		marc->flow[j] = narc->flow[j] = 1;
	__calcChosenPathPostFlow(cslots, marc, narc, true);

	debug_log("marc->flow:");
	int i = 0;
	for (i = 0; i < slotNum; ++i)
		printf(" %d", marc->flow[i]);
	printf("\n");
	debug_log("narc->flow:");
	for (i = 0; i < slotNum; ++i)
		printf(" %d", narc->flow[i]);
	printf("\n");
#endif

	// test calculate segment offset
#if 0
	delete []arcTable;
	arcIDIncrement = 0;
	arcTable = new Arc[3];
	nodeNum = 3;

	struct Arc *parc = &arcTable[0];
	parc->srcID = 0; parc->dstID = 2; parc->flow[0] = 2; parc->flow[4] = 4; parc->flow[7] = 3; parc->downloader[0] = 2; parc->downloader[4] = 2; parc->downloader[7] = 2;
	std::vector<int> arcPath;
	arcPath.push_back(parc->arcID);
	arcPath.push_back(2);
	arcPathList.push_back(arcPath);
	arcPath.clear();

	parc = &arcTable[1];
	parc->srcID = 0; parc->dstID = 1; parc->flow[1] = 3; parc->flow[2] = 4; parc->flow[5] = 4; parc->downloader[1] = 2; parc->downloader[2] = 2; parc->downloader[5] = 2;
	arcPath.push_back(parc->arcID);
	parc = &arcTable[2];
	parc->srcID = 1; parc->dstID = 2; parc->flow[3] = 5; parc->flow[6] = 6; parc->downloader[3] = 2; parc->downloader[6] = 2;
	arcPath.push_back(parc->arcID);
	arcPath.push_back(2);
	arcPathList.push_back(arcPath);
	arcPath.clear();

	arcPathRange.push_back(std::pair<int, int>(0, 2));
	downloaderNum = 1;
	downloaderArray[0] = 2;
	_recordFluxScheme();
#endif

	// test class Segment
#if 0
	FluxScheme schemes[3];
	schemes[0].slot = 3;
	struct Segment *pSeg = &schemes[0].segment;
	pSeg->begin = 0;
	pSeg->end = 3;
	pSeg->next = new struct Segment;
	pSeg = pSeg->next;
	pSeg->begin = 6;
	pSeg->end = 8;
	pSeg->next = new struct Segment;
	pSeg = pSeg->next;
	pSeg->begin = 11;
	pSeg->end = 14;
	schemes[1].slot = 5;
	struct Segment *qSeg = &schemes[1].segment;
	qSeg->begin = 2;
	qSeg->end = 5;
	qSeg->next = new struct Segment;
	qSeg = qSeg->next;
	qSeg->begin = 7;
	qSeg->end = 9;
	schemes[2].slot = 4;
	struct Segment *rSeg = &schemes[2].segment;
	rSeg->begin = 1;
	rSeg->end = 4;
	rSeg->next = new struct Segment;
	rSeg = rSeg->next;
	rSeg->begin = 5;
	rSeg->end = 9;
	info_log("before swap\n");
	schemes[0].segment.print();
	schemes[1].segment.print();
	schemes[2].segment.print();
	info_log("after swap\n");
	schemes[2] = schemes[0];
	schemes[0] = schemes[1];
	schemes[1] = schemes[2];
	schemes[0].segment.print();
	schemes[1].segment.print();
	schemes[2].segment.print();

	std::vector<FluxScheme> FluxSchemeList;
	FluxSchemeList.push_back(schemes[0]);
	FluxSchemeList.push_back(schemes[1]);
	FluxSchemeList.push_back(schemes[2]);
	std::sort(FluxSchemeList.begin(), FluxSchemeList.end(), FluxSchemeCmp());
	info_log("after std::sort\n");
	FluxSchemeList[0].segment.print();
	FluxSchemeList[1].segment.print();
	FluxSchemeList[2].segment.print();
#endif
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

	info_log("===== Arc path list =====\n");
	for (itAPL = arcPathList.begin(); itAPL != arcPathList.end(); ++itAPL)
		printVector(*itAPL, "", INFO_LEVEL);

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

	debug_log("===== Arc path range =====\n");
	if (log_level >= DEBUG_LEVEL)
		for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
			printf("downloader %d: [%d,%d)\n", downloaderArray[downloaderIdx], arcPathRange[downloaderIdx].first, arcPathRange[downloaderIdx].second);
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
		int maxBandwidth = -1;
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
	int minReceivedAmount = INT32_MAX;
	int minReceivedIdx = 0;
	for (int downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		if (minReceivedAmount > receivedAmountTable[downloaderIdx])
		{
			minReceivedAmount = receivedAmountTable[downloaderIdx];
			minReceivedIdx = downloaderIdx;
		}
	}
	return _optimizeReceivedAmount(minReceivedIdx);
}

bool STAG::_optimizeReceivedAmount(const int curDownloaderIdx)
{
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

	if (willOptimize)
	{
		int i = arcPathRange[curDownloaderIdx].first; // i is the direct download arc path index
		int storeI = 0, storeM = 0, storeN = 0, m = 0, n = 0;
		int maxAlternativeFlow = INT32_MIN;
		size_t k = 0;
		struct Arc *marc = NULL, *narc = NULL, *collisionArc = NULL;
		// obtain currently optimal alternative relay path according to the maximize flow rule
		for (i = arcPathRange[curDownloaderIdx].first+1; i < arcPathRange[curDownloaderIdx].second; ++i)
		{
			marc = &arcTable[arcPathList[i][0]];
			narc = &arcTable[arcPathList[i][1]];
			for (m = 0; m < slotNum-1; ++m)
			{
				for (n = m+1; n < slotNum; ++n)
				{
					int alternativeFlow = __calcAlternativeFlow(i, m, n, false); // calculate alternative flow on the same arc path
					for (k = 0; k < chosenLinks[m].size(); ++k) // calculate alternative flow on the other arc paths
					{
						int chosenPath = chosenPaths[m][k];
						if (chosenPath == i)
							continue;
						collisionArc = &arcTable[chosenLinks[m][k]];
						if (arcPathList[chosenPath].size() == 2) // the direct arc path
							alternativeFlow -= collisionArc->flow[m];
						else // the relay arc path
						{
							if (vectorFind(chosenPaths[n], chosenPath))
								alternativeFlow += __calcDecreasedFlow(chosenPath, m, n);
							else if (collisionArc->srcID == 0) // first arc in the relay arc path
								alternativeFlow += __calcDecreasedFlow(chosenPath, m, -1);
							else // second arc in the relay arc path
								alternativeFlow += __calcDecreasedFlow(chosenPath, -1, m);
						}
					}
					for (k = 0; k < chosenLinks[n].size(); ++k) // calculate alternative flow on the other arc paths
					{
						int chosenPath = chosenPaths[n][k];
						if (chosenPath == i)
							continue;
						collisionArc = &arcTable[chosenLinks[n][k]];
						if (arcPathList[chosenPath].size() == 2) // the direct arc path
							alternativeFlow -= collisionArc->flow[n];
						else // the relay arc path
						{
							if (vectorFind(chosenPaths[m], chosenPath))
								; // avoid calculating twice
							else if (collisionArc->srcID == 0) // first arc in the relay arc path
								alternativeFlow += __calcDecreasedFlow(chosenPath, n, -1);
							else // second arc in the relay arc path
								alternativeFlow += __calcDecreasedFlow(chosenPath, -1, n);
						}
					}
					if (vectorFind(chosenPaths[m], i) && vectorFind(chosenPaths[n], i))
						alternativeFlow = -99999999;
					if (log_level >= DEBUG_LEVEL)
						printf(", alternativeFlow is %d\n", alternativeFlow);
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
		debug_log("path %d, marc %d, narc %d, m %d, n %d\n", i, marc->arcID, narc->arcID, m, n);

		// update chosenLinks, chosenPaths, chosenSlots
		std::vector<int> tmpChosenLink;
		std::vector<int> tmpChosenPath, storeChosenPath;
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
				if (vectorFind(chosenPaths[n], chosenPath))
					storeChosenPath = chosenPaths[m]; // reserve to be done when check slot n
				else
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
				if (arcPathList[chosenPath].size() > 2 && vectorFind(storeChosenPath, chosenPath))
				{
					struct Arc *farc = &arcTable[arcPathList[chosenPath][0]], *sarc = &arcTable[arcPathList[chosenPath][1]];
					__calcChosenPathPostFlow(chosenSlots[chosenPath], farc, sarc, true);
				}
				else
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

void STAG::_checkChosenSlots(struct Arc *collisionArc, const int collisionSlot, const int chosenPath)
{
	if (arcPathList[chosenPath].size() > 2)
	{
		int j = 0, decreasedFlow = collisionArc->flow[collisionSlot];
		struct Arc *checkingArc = NULL;
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
					vectorRemove(chosenLinks[j], checkingArc->arcID);
					vectorRemove(chosenPaths[j], chosenPath);
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
					vectorRemove(chosenLinks[j], checkingArc->arcID);
					vectorRemove(chosenPaths[j], chosenPath);
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
	debug_log("===== chosen links =====\n");
	int j = 0, downloaderIdx = 0;
	for (j = 0; j < slotNum; ++j)
	{
		std::vector<int> &chosenLink = chosenLinks[j];
		if (log_level >= DEBUG_LEVEL)
		{
			printf("slot %d:", j+1);
			for (size_t l = 0; l < chosenLink.size(); ++l)
				printf(" %d(%d->%d)[%d]", chosenLink[l], arcTable[chosenLink[l]].srcID, arcTable[chosenLink[l]].dstID, arcTable[chosenLink[l]].flow[j]);
			printf("\n");
		}

		// handle with data storage process
		for (int i = 0; i < nodeNum; ++i)
			for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
				NodeList[i].S[j+1][downloaderIdx] = NodeList[i].S[j][downloaderIdx];

		// handle with data transmition process
		for (size_t k = 0; k < chosenLink.size(); ++k)
		{
			struct Arc *parc = &arcTable[chosenLink[k]];
			if (parc->downloader[j] != -1)
			{
				downloaderIdx = downloaderTable[parc->downloader[j]];
				parc->flow[j] = std::min(parc->flow[j], NodeList[parc->srcID].S[j][downloaderIdx].size);
			}
			NodeList[parc->srcID].S[j+1][downloaderIdx].size -= parc->flow[j];
			NodeList[parc->dstID].S[j+1][downloaderIdx].size += parc->flow[j];
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
	for (int i = 0; i < nodeNum; ++i)
		for (int j = 1; j <= slotNum; ++j)
			for (int d = 0; d < downloaderNum; ++d)
				NodeList[i].S[j][d].size = 0;
}

int STAG::_obtainSchemeIndicators()
{
	int totalReceivedAmount = 0;

	for (int downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
	{
		int downloader = downloaderArray[downloaderIdx];
		int receivedAmount  = NodeList[downloader].S[slotNum][downloaderIdx].size - NodeList[downloader].S[0][downloaderIdx].size;
		debug_log("Received data amount of downloader [%d] is %d.\n", downloader, receivedAmount);
		receivedAmountTable[downloaderIdx] = receivedAmount;
		totalReceivedAmount += receivedAmount;
	}

	debug_log("Current total received data amount is %d.\n", totalReceivedAmount);

	return totalReceivedAmount;
}

#if VERIFY_SCHEME
void STAG::_recordFluxPath()
{
	std::vector<int> fluxPath;
	fluxPath.reserve(2 + 3*slotNum);
	for (itAPL = arcPathList.begin(); itAPL != arcPathList.end(); ++itAPL)
	{
		std::vector<int> &arcPath = *itAPL; // alias
		for (size_t i = 0; i < arcPath.size()-1; ++i)
		{
			struct Arc *parc = &arcTable[arcPath[i]];
			bool hasAddedNode = false;
			for (int j = 0; j < slotNum; ++j)
			{
				if (parc->flow[j] > 0 && parc->downloader[j] == arcPath.back())
				{
					if (!hasAddedNode)
					{
						hasAddedNode = true;
						fluxPath.push_back(parc->srcID);
						fluxPath.push_back(parc->dstID);
					}
					fluxPath.push_back(j+1);
					fluxPath.push_back(parc->downloader[j]);
					fluxPath.push_back(parc->flow[j]);
				}
			}
			if (!fluxPath.empty())
			{
				fluxPathList.push_back(fluxPath);
				fluxPath.clear();
			}
		}
	}
}
#endif

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
				int j = 0;
				for (j = 0; j < slotNum; ++j)
				{
					if (parc->flow[j] > 0 && parc->dstID == parc->downloader[j]) // && parc->downloader[j] == arcPathList[pathIdx].back())
					{
						sortReceivedSlot.push_back(std::pair<int, int>(j, parc->flow[j]));
						sortReceivedPath.push_back(std::pair<int, int>(j, pathIdx));
					}
				}
				info_log("flow:");
				if (log_level >= INFO_LEVEL)
				{
					for (j = 0; j < slotNum; ++j)
						printf(" (%d,%d)", j+1, parc->flow[j]);
					printf("\n");
				}
			}
		}
		std::sort(sortReceivedSlot.begin(), sortReceivedSlot.end());
		int accumulatedOffset = 0;
		info_log("arrival (slot,amount)[offset]:");
		for (k = 0; k < sortReceivedSlot.size(); ++k)
		{
			segmentOffsets[sortReceivedSlot[k].first].begin = accumulatedOffset;
			dupSegmentOffsets[sortReceivedSlot[k].first].begin = accumulatedOffset;
			accumulatedOffset += sortReceivedSlot[k].second;
			segmentOffsets[sortReceivedSlot[k].first].end = accumulatedOffset;
			dupSegmentOffsets[sortReceivedSlot[k].first].end = accumulatedOffset;
			if (log_level >= INFO_LEVEL)
				printf(" (%d,%d)[%d,%d]", sortReceivedSlot[k].first+1, sortReceivedSlot[k].second, segmentOffsets[sortReceivedSlot[k].first].begin, segmentOffsets[sortReceivedSlot[k].first].end);
		}
		if (log_level >= INFO_LEVEL)
			printf("\n");

#if 0
		// this #if ... #endif part is left for test, beacuse we need to modify prefetching amount in order to avoid exceeding total content size
		int prefetchAmount = 1164800, demandingAmount = 8338800; // set these two variables to test, note prefetchAmount >= demandingAmount
		int decreasingAmount = prefetchAmount - demandingAmount;
		if (decreasingAmount > 0)
		{
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
#endif

		pathIdx = arcPathRange[downloaderIdx].first; // handle with the direct download arc path
		for (k = 0; k < arcPathList[pathIdx].size()-1; ++k)
		{
			struct Arc *parc = &arcTable[arcPathList[pathIdx][k]];
			for (int j = 0; j < slotNum; ++j)
			{
				if (parc->flow[j] > 0 && parc->dstID == parc->downloader[j])
				{
					fluxSchemeList[parc->srcID].push_back(FluxScheme(j+1, parc->dstID, parc->downloader[j], parc->flow[j]));
					fluxSchemeList[parc->srcID].back().segment = segmentOffsets[j];
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
				if (marc->flow[j] > 0 && marc->downloader[j] == arcPathList[pathIdx].back())
					marcSlotFlow.push_back(std::pair<int, int>(j, marc->flow[j]));
				if (narc->flow[j] > 0)
					narcSlotFlow.push_back(std::pair<int, int>(j, narc->flow[j]));
			}
			if (marcSlotFlow.empty() || narcSlotFlow.empty())
				continue;
			bool uncontinuousAppend = true;
			struct Segment *mSeg = &segmentOffsets[marcSlotFlow[0].first], *nSeg = &dupSegmentOffsets[narcSlotFlow[0].first];
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
						mSeg->next = new struct Segment;
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
				if (marc->flow[j] > 0 && marc->downloader[j] == arcPathList[pathIdx].back())
				{
					FluxScheme fluxS(j+1, marc->dstID, marc->downloader[j], marc->flow[j]);
					fluxSchemeList[marc->srcID].push_back(fluxS);
					fluxSchemeList[marc->srcID].back().segment = segmentOffsets[j];
					fluxSchemeList[marc->srcID].back().segment.print();
				}
				if (narc->flow[j] > 0)
				{
					FluxScheme fluxS(j+1, narc->dstID, narc->downloader[j], narc->flow[j]);
					fluxSchemeList[narc->srcID].push_back(fluxS);
					fluxSchemeList[narc->srcID].back().segment = segmentOffsets[j];
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
		if (log_level >= INFO_LEVEL)
		{
			printf("%d |", i);
			for (k = 0; k < fluxSchemeList[i].size(); ++k)
				printf(" %d %d %d %d", fluxSchemeList[i][k].slot, fluxSchemeList[i][k].dest, fluxSchemeList[i][k].downloader, fluxSchemeList[i][k].flow);
			printf("\n");
		}
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
	if (firstArcFlow != secondArcFlow)
	{
		error_log("assert firstArcFlow == secondArcFlow failed.\n");
		exit(EXIT_FAILURE);
	}
	preFlow = firstArcFlow;

	if (erasureM)
	{
		__eraseChosenSlot(chosenSlot, m);
		if (setFlow)
		{
			narc->flow[m] = 0;
			narc->downloader[m] = -1;
		}
	}
	if (erasureN)
	{
		__eraseChosenSlot(chosenSlot, n);
		if (setFlow)
		{
			marc->flow[n] = 0;
			marc->downloader[n] = -1;
		}
	}
	if (!collisionM)
		__insertChosenSlot(chosenSlot, m, true);
	if (!collisionN)
		__insertChosenSlot(chosenSlot, n, false);

	debug_log("%s choose slot:", setFlow ? "so" : "if");
	__printChosenSlot(chosenSlot);

	postFlow = __calcChosenPathPostFlow(chosenSlot, marc, narc, setFlow);
	if (setFlow)
		chosenSlots[curPath] = chosenSlot; // assign back
	else
		debug_log("then preFlow is %d, postFlow is %d", preFlow, postFlow);

	return postFlow - preFlow;
}

int STAG::__calcDecreasedFlow(const int curPath, const int m, const int n)
{
	const bool firstCollision = m != -1, secondCollision = n != -1;
	struct Arc *marc = &arcTable[arcPathList[curPath][0]], *narc = &arcTable[arcPathList[curPath][1]];
	std::vector<int> chosenSlot = chosenSlots[curPath]; // value copy
	int i = 0, preFlow = 0, postFlow = 0, firstArcFlow = 0, secondArcFlow = 0;
	int indicator = chosenSlot.back(), vecSize = static_cast<int>(chosenSlot.size());
	for (i = 0; i < indicator; ++i)
		firstArcFlow += marc->flow[chosenSlot[i]];
	for (i = indicator; i < vecSize-1; ++i)
		secondArcFlow += narc->flow[chosenSlot[i]];
	if (firstArcFlow != secondArcFlow)
	{
		error_log("assert firstArcFlow == secondArcFlow failed.\n");
		exit(EXIT_FAILURE);
	}
	preFlow = firstArcFlow;

	if (firstCollision)
		__eraseChosenSlot(chosenSlot, m);
	if (secondCollision)
		__eraseChosenSlot(chosenSlot, n);

	postFlow = __calcChosenPathPostFlow(chosenSlot, marc, narc, false);

	return postFlow - preFlow;
}

int STAG::__calcChosenPathPostFlow(std::vector<int>& chosenSlot, struct Arc *&marc, struct Arc *&narc, const bool setFlow)
{
	int i = 0, j = 0, firstArcBW = 0, secondArcBW = 0, firstArcRestCap = 0, secondArcRestCap = 0;
	int indicator = chosenSlot.back(), vecSize = static_cast<int>(chosenSlot.size());
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

	if (firstArcBW - firstArcRestCap != secondArcBW - secondArcRestCap)
	{
		error_log("assert firstArcFlow == secondArcFlow failed.\n");
		exit(EXIT_FAILURE);
	}
	int maxFlow = firstArcBW - firstArcRestCap;

	if (setFlow)
	{
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
	if (log_level < DEBUG_LEVEL)
		return;

	for (int i = 0; i < nodeNum; ++i)
	{
		printf("node [%d]:  %d$(%d", i, downloaderArray[0], NodeList[i].S[0][0].size);
		int j = 0;
		for (j = 1; j <= slotNum; ++j)
			printf(",%d", NodeList[i].S[j][0].size);
		for (int d = 1; d < downloaderNum; ++d)
		{
			printf(")  |  %d$(%d", downloaderArray[d], NodeList[i].S[0][d].size);
			for (j = 1; j <= slotNum; ++j)
				printf(",%d", NodeList[i].S[j][d].size);
		}
		printf(")\n");
	}
}

void STAG::__printChannelStatus()
{
	if (log_level < DEBUG_LEVEL)
		return;

	for (int i = 0; i < nodeNum; i++)
	{
		printf("node [%d]", i);
		for (struct Arc *darc = NodeList[i].firstArc; darc != NULL; darc = darc->nextArc)
		{
			printf(" -> %d(", darc->dstID);
			for (int j = 0; j < slotNum-1; ++j)
				printf("%d,", darc->idle[j]);
			printf("%d)", darc->idle[slotNum-1]);
		}
		printf("\n");
	}
}

void STAG::__printChosenSlots()
{
	if (log_level < DEBUG_LEVEL)
		return;

	debug_log("===== chosen slots =====\n");
	for (size_t k = 0; k < chosenSlots.size(); ++k)
	{
		printf("path %lu:", k);
		__printChosenSlot(chosenSlots[k]);
	}
}

void STAG::__printChosenSlot(std::vector<int>& vec)
{
	if (vec.empty())
	{
		error_log("chosen slot is empty!\n");
		exit(EXIT_FAILURE);
	}

	if (log_level < DEBUG_LEVEL)
		return;

	int i = 0, indicator = vec.back(), vecSize = static_cast<int>(vec.size());
	if (indicator != -1) // relay arc path
	{
		for (i = 0; i < indicator; ++i)
		{
			printf(" %d", vec[i]+1);
			if (i == indicator-1)
				printf(" |");
		}
		for (i = indicator; i < vecSize-1; ++i)
		{
			printf(" %d", vec[i]+1);
			if (i == vecSize-2)
				printf(" |");
		}
		printf(" %d", indicator);
	}
	else // direct arc path
	{
		for (i = 0; i < vecSize-1; ++i)
		{
			printf(" %d", vec[i]+1);
			if (i == vecSize-2)
				printf(" |");
		}
		printf(" %d", indicator);
	}
	printf("\n");
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
		error_log("erase the slot that hasn't been chosen!\n");
		exit(EXIT_FAILURE);
	}
}

void STAG::__insertChosenSlot(std::vector<int>& vec, const int slot, bool firstArc)
{
	int i = 0, pos = -1, vecSize = static_cast<int>(vec.size());
	for (i = 0; i < vecSize-1; ++i)
	{
		if (vec[i] == slot)
		{
			error_log("insert the same slot as the slot that has been chosen!\n");
			exit(EXIT_FAILURE);
		}
	}

	int &indicator = vec.back();
	if (firstArc)
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
	if (ucPathSize > 0) // ensure there exists unchosen paths
	{
		int cPathIdxPos = 1;
		size_t pathIdxUC = unChosenPathIndices[0];
		while (cPathIdxPos < cPathSize)
		{
			size_t pathIdxC = chosenPathIndices[cPathIdxPos++];
			if (pathIdxUC < pathIdxC)
			{
				arcPathList[pathIdxUC].swap(arcPathList[pathIdxC]);
				++pathIdxUC;
			}
		}
		while (ucPathSize-- > 0) // remove unchosen paths as they all have been swapped to the tail of vector
			arcPathList.pop_back();
	}

	info_log("===== chosen arc path list =====\n");
	for (itAPL = arcPathList.begin(); itAPL != arcPathList.end(); ++itAPL)
		printVector(*itAPL, "", INFO_LEVEL);

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

	info_log("===== chosen arc path range =====\n");
	if (log_level >= INFO_LEVEL)
		for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
			printf("downloader %d: [%d,%d)\n", downloaderArray[downloaderIdx], arcPathRange[downloaderIdx].first, arcPathRange[downloaderIdx].second);
}

bool STAG::__isCollision(struct Arc *parc, struct Arc *qarc, const int slot)
{
	return true;
}


////////////////    scheme verification part    ////////////////
#if VERIFY_SCHEME
enum VState {
	I, ///< Idle
	T, ///< Transmit
	R, ///< Receive
	C  ///< Collision
};

inline const char VStateStr(VState state)
{
	if (state == VState::I)
		return 'I';
	else if (state == VState::T)
		return 'T';
	else if (state == VState::R)
		return 'R';
	else
		return 'C';
}

/** verification of the file segment transmition scheme. */
bool verifyScheme(STAG& graph, double& totalInterruptTime, int& totalReceivedAmount)
{
	uncond_log("\n====================    Enter scheme verification category    ====================\n");
	std::list<std::vector<int> > &transmitionList = graph.fluxPathList;
	std::list<std::vector<int> >::iterator itTL = transmitionList.begin();
	bool ok = true;
	int i = 0, j = 0;
	size_t k = 0;
	struct Arc *parc = NULL;

	// 1. ====================    check the validity of scheme    ====================
	VState *channelOccupied = new VState[nodeNum*slotNum];
	VState **occupied = new VState*[nodeNum];
	for (i = 0; i < nodeNum; ++i)
		occupied[i] = channelOccupied + i*slotNum;
	for (i = 0; i < nodeNum*slotNum; ++i)
		channelOccupied[i] = VState::I;
	int *oppositeEnd = new int[nodeNum*slotNum];
	int **opposite = new int*[nodeNum];
	for (i = 0; i < nodeNum; ++i)
		opposite[i] = oppositeEnd + i*slotNum;
	for (i = 0; i < nodeNum*slotNum; ++i)
		oppositeEnd[i] = 0;
	Data *dataInfo = new Data[nodeNum*slotNum];
	Data **info = new Data*[nodeNum];
	for (i = 0; i < nodeNum; ++i)
		info[i] = dataInfo + i*slotNum;

	// 1.1. ===========    collect channel occupiation statistics    ==========
	int from = 0, to = 0, slot = 0;
	if (log_level >= INFO_LEVEL)
		printf("===== transmition scheme =====\n");
	for (; itTL != transmitionList.end(); ++itTL)
	{
		std::vector<int> &transmition = *itTL; // alias
		if (log_level >= INFO_LEVEL)
		{
			for (k = 0; k < transmition.size(); ++k)
				printf("%d ", transmition[k]);
			printf("\n");
		}
		from = transmition[0];
		to = transmition[1];
		for (k = 2; k < transmition.size(); k += 3)
		{
			slot = transmition[k] - 1;
			if (occupied[from][slot] == VState::I)
				occupied[from][slot] = VState::T;
			else
				occupied[from][slot] = VState::C;
			opposite[from][slot] = to;
			info[from][slot].downloader = transmition[k+1];
			info[from][slot].size = transmition[k+2];
			if (occupied[to][slot] == VState::I)
				occupied[to][slot] = VState::R;
			else
				occupied[to][slot] = VState::C;
			opposite[to][slot] = from;
			info[to][slot].downloader = transmition[k+1];
			info[to][slot].size = transmition[k+2];
		}
	}
	// 1.2. ==========    check whether there exists the following situations    ==========
	//     1) a vehicle transmit and receive at the same time;
	//     2) a vehicle transmit to more than one vehicle at the same time;
	//     3) a vehicle receive from more than one vehicle at the same time.
	std::list<std::pair<int, int> > transmitionPairs;
	for (i = 0; i < nodeNum; ++i)
		for (j = 0; j < slotNum; ++j)
			if (occupied[i][j] == VState::C)
				transmitionPairs.push_back(std::pair<int, int>(i, j+1));

	if (log_level >= DEBUG_LEVEL)
	{
		printf("===== vehicle state =====\n");
		for (i = 0; i < nodeNum; ++i)
		{
			for (j = 0; j < slotNum; ++j)
				printf("%c ", VStateStr(occupied[i][j]));
			printf("\n");
		}
		printf("===== opposite end =====\n");
		for (i = 0; i < nodeNum; ++i)
		{
			for (j = 0; j < slotNum; ++j)
				printf("%d ", opposite[i][j]);
			printf("\n");
		}
		printf("===== transmitted =====\n");
		for (i = 0; i < nodeNum; ++i)
		{
			for (j = 0; j < slotNum; ++j)
				printf("%d ", info[i][j].size);
			printf("\n");
		}
	}
	while (!transmitionPairs.empty())
	{
		error_log("node [%d] in time slot [%d] break the channel contention rule.\n", transmitionPairs.front().first, transmitionPairs.front().second);
		transmitionPairs.pop_front();
		ok = false;
	}

	// 1.3. ==========    check whether there exists the following situations    ==========
	//     when a vehicle is transmiting, its neighbor or its receiver's neighbor is transmiting at the same time.
	if (ok)
	{
		for (j = 0; j < slotNum; ++j)
		{
			for (i = 0; i < nodeNum; ++i)
			{
				if (occupied[i][j] == VState::T)
				{
					int receiver = opposite[i][j];
					for (parc = graph.NodeList[i].firstArc; parc != NULL; parc = parc->nextArc)
					{
						if (parc->bandwidth[j] && occupied[parc->dstID][j] == VState::T)
						{
							transmitionPairs.push_back(std::pair<int, int>(i, j+1));
							ok = false;
						}
						if (parc->bandwidth[j] && occupied[parc->dstID][j] == VState::R && opposite[parc->dstID][j] != i)
						{
							transmitionPairs.push_back(std::pair<int, int>(i, j+1));
							ok = false;
						}
					}
					for (parc = graph.NodeList[receiver].firstArc; parc != NULL; parc = parc->nextArc)
					{
						if (parc->bandwidth[j] && occupied[parc->dstID][j] == VState::T && opposite[parc->dstID][j] != receiver)
						{
							transmitionPairs.push_back(std::pair<int, int>(i, j+1));
							ok = false;
						}
					}
				}
				if (!ok)
					break;
			}
			if (!ok)
				break;
		}
	}

	if (!ok)
	{
		while (!transmitionPairs.empty())
		{
			error_log("node [%d] in time slot [%d] break the channel contention rule.\n", transmitionPairs.front().first, transmitionPairs.front().second);
			transmitionPairs.pop_front();
		}

		delete []info;
		delete []dataInfo;
		delete []opposite;
		delete []oppositeEnd;
		delete []occupied;
		delete []channelOccupied;

		uncond_log("====================    Exit scheme verification category    ====================\n\n");
		return false;
	}

	// 2. ====================    summarize the indicators of scheme    ====================
	int downloaderIdx = 0, downloader = 0, playRate = 0;

	for (j = 0; j < slotNum; ++j)
	{
		for (i = 0; i < nodeNum; ++i)
		{
			if (occupied[i][j] == VState::I)
				for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
					graph.NodeList[i].S[j+1][downloaderIdx].size = graph.NodeList[i].S[j][downloaderIdx].size;
			else if (occupied[i][j] == VState::T)
			{
				to = opposite[i][j]; // alias
				for (parc = graph.NodeList[i].firstArc; parc != NULL; parc = parc->nextArc)
					if (parc->dstID == to)
						break;
				if (info[i][j].size <= parc->bandwidth[j])
				{
					downloaderIdx = downloaderTable[info[i][j].downloader]; // alias
					// debug_log("original: %d, transmitted: %d, downloader: %d\n", graph.NodeList[i].S[j][downloader].size, info[i][j].size, downloader);
					graph.NodeList[i].S[j+1][downloaderIdx].size = graph.NodeList[i].S[j][downloaderIdx].size - info[i][j].size;
					graph.NodeList[to].S[j+1][downloaderIdx].size = graph.NodeList[to].S[j][downloaderIdx].size + info[i][j].size;
					if (graph.NodeList[i].S[j+1][downloaderIdx].size < 0)
					{
						error_log("transmitted data size from [%d] to [%d] beyond transmitter's original storage in slot [%d]\n", i, to, j+1);
						ok = false;
					}
					for (int d = 0; d < downloaderNum; ++d)
					{
						if (d != downloaderIdx) // as for other downloaders' data storage, reserve to next slot
						{
							graph.NodeList[i].S[j+1][d].size = graph.NodeList[i].S[j][d].size;
							graph.NodeList[to].S[j+1][d].size = graph.NodeList[to].S[j][d].size;
						}
					}
				}
				else
				{
					error_log("transmitted data size from [%d] to [%d] beyond link bandwidth in slot [%d]\n", i, to, j+1);
					ok = false;
				}
			}
			if (!ok)
				break;
		}
		if (!ok)
			break;
	}

	if (ok)
	{
		for (j = 0; j <= slotNum; ++j)
			for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
				graph.NodeList[0].S[j][downloaderIdx].size -= graph.NodeList[0].S[slotNum][downloaderIdx].size;

		if (log_level >= DEBUG_LEVEL)
		{
			for (i = 0; i <= nodeNum; ++i)
			{
				printf("node [%d]:  %d$(%d", i, downloaderArray[0], graph.NodeList[i].S[0][0].size);
				for (j = 1; j <= slotNum; ++j)
					printf(",%d", graph.NodeList[i].S[j][0].size);
				for (int d = 1; d < downloaderNum; ++d)
				{
					printf(")  |  %d$(%d", downloaderArray[d], graph.NodeList[i].S[0][d].size);
					for (j = 1; j <= slotNum; ++j)
						printf(",%d", graph.NodeList[i].S[j][d].size);
				}
				printf(")\n");
			}
		}

		for (downloaderIdx = 0; downloaderIdx < downloaderNum; ++downloaderIdx)
		{
			downloader = downloaderArray[downloaderIdx];
			int receivedAmount  = graph.NodeList[downloader].S[slotNum][downloaderIdx].size
								- graph.NodeList[downloader].S[0][downloaderIdx].size;
			info_log("Received data amount of downloader [%d] is %d.\n", downloader, receivedAmount);
			totalReceivedAmount += receivedAmount;
		}
	}

	delete []info;
	delete []dataInfo;
	delete []opposite;
	delete []oppositeEnd;
	delete []occupied;
	delete []channelOccupied;

	uncond_log("====================    Exit scheme verification category    ====================\n\n");
	return ok;
}
#endif
