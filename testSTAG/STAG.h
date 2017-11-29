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

#ifndef __STAG_H__
#define __STAG_H__

#include "Utils.h"

#define VERIFY_SCHEME    0

/** Arc structure of graph. */
struct Arc
{
	Arc();
	~Arc();

	int arcID;       ///< ID of this arc.
	int srcID;       ///< tail of this arc.
	int dstID;       ///< head of this arc.
	int *bandwidth;  ///< max capacity of this arc.
	int *flow;       ///< current flow of this arc.
	int *downloader; ///< current flow data belong to which downloader.
	bool *idle;      ///< whether current slot of this arc is idle for transmition.

	Arc *nextArc;    ///< next out degree arc of the node this arc derived.
};

/** Node structure of graph. */
struct Node
{
	Node();
	~Node();

	Data *_S; ///< continuous memory allocation, client code should use S instead.
	Data **S; ///< convenient matrix form S[i][j], where i is slot dimension and j is downloader dimension.

	Arc *firstArc; ///< first out degree arc of this node.
	Arc *lastArc;  ///< last out degree arc of this node.
};

/** Adjacency list structure representing the graph. */
class STAG
{
public:
	STAG();
	~STAG();

	/** @name regular methods of STAG. */
	///@{
	void construct(std::list<LinkTuple>& linkTuples);
	void undirectify(std::list<LinkTuple>& linkTuples);
	void destruct();
	void display();
	///@}

	/**
	 * API function executing a shortest interrupt time first max flow algorithm of STAG.
	 * 
	 * @return a pair of indicators totalInterruptTime and totalReceivedAmount.
	 */
	std::pair<double, int> maximumFlow();

	void test();

	struct Arc* oppositeArc(int curID) { return &arcTable[curID ^ 1]; }

private:
	STAG(const STAG&);
	STAG& operator=(const STAG&);

	void _initNodeStorage(int curNode);

	/** @name macro internal functions used by maximum flow algorithm. */
	///@{
	void _obtainArcPathList();
	void _obtainInitialScheme();
	bool _seekBetterScheme();
	bool _optimizeInterruptTime(const int curDownloaderIdx);
	bool _optimizeReceivedAmount(const int curDownloaderIdx);
	void _calcNodeStorage();
	void _clearNodeStorage();
	void _checkChosenSlots(struct Arc *collisionArc, const int collisionSlot, const int chosenPath);
	void _obtainSchemeIndicators(double& totalInterruptTime, int& totalReceivedAmount);
#if VERIFY_SCHEME
	void _recordFluxPath();
#endif
	void _recordFluxScheme();
	void _recordBetterScheme();
	void _revertBestScheme();

	void _addSuperSink();
	void _delSuperSink();
	///@}

	/** @name micro internal functions used by maximum flow algorithm. */
	///@{
	int __calcAlternativeFlow(const int curPath, const int m, const int n, const bool setFlow = false);
	int __calcDecreasedFlow(const int curPath, const int m, const int n);
	int __calcChosenPathPostFlow(std::vector<int>& chosenSlot, struct Arc *&marc, struct Arc *&narc, const bool setFlow);
	void __printNodeStorage();
	void __printChannelStatus();
	void __printChosenSlots();
	void __printChosenSlot(std::vector<int>& vec);
	void __eraseChosenSlot(std::vector<int>& vec, const int slot);
	void __insertChosenSlot(std::vector<int>& vec, const int slot, bool firstArc = false);
	void __eraseUnchosenArcPath();
	bool __isCollision(struct Arc *parc, struct Arc *qarc, const int slot);
	///@}

public:
	/** Segment structure of data. */
	struct Segment
	{
		Segment() : begin(-1), end(-1), next(NULL) {}
		~Segment()
		{
			if (next != NULL) // delete recursively
				delete next;
		}

		Segment& operator=(const Segment& rhs)
		{
			if (this == &rhs)
				return *this;
			clear();
			assign(&rhs);
			return *this;
		}

		void clear()
		{
			if (next != NULL) // delete recursively
				delete next;
		}

		void assign(const Segment *rhs)
		{
			begin = rhs->begin;
			end = rhs->end;
			if (rhs->next != NULL) // assign recursively
			{
				next = new struct Segment;
				next->assign(rhs->next);
			}
			else // reach the last segment
				next = NULL;
		}

		void print()
		{
			printf("(%d-%d) ", begin, end);
			if (next != NULL) // print recursively
				next->print();
			else // reach the last segment
				printf("\n");
		}

		int begin;
		int end;
		struct Segment *next;
	};
	/** Result format required by ContentRSU. */
	class FluxScheme
	{
	public:
		FluxScheme() : slot(-1), dest(-1), downloader(-1), flow(-1), segment() {}
		FluxScheme(int s, int r, int d, int a) : slot(s), dest(r), downloader(d), flow(a), segment() {}
		FluxScheme(const FluxScheme& rhs) : slot(rhs.slot), dest(rhs.dest), downloader(rhs.downloader), flow(rhs.flow) { segment = rhs.segment; }

		FluxScheme& operator=(const FluxScheme& rhs)
		{
			if (this == &rhs)
				return *this;
			slot = rhs.slot;
			dest = rhs.dest;
			downloader = rhs.downloader;
			flow = rhs.flow;
			segment = rhs.segment;
			return *this;
		}

		int slot;
		int dest;
		int downloader;
		int flow;
		Segment segment;
	};
	/** Predictor of class FluxScheme. */
	struct FluxSchemeCmp : public std::binary_function<FluxScheme, FluxScheme, bool>
	{
		/** compare rule of class FluxScheme. */
		bool operator()(const FluxScheme& lhs, const FluxScheme& rhs) const
		{
			return lhs.slot < rhs.slot;
		}
	};

	struct Node *NodeList;  ///< storing all the nodes in the graph.
	struct Arc *arcTable;   /**< 1. range of links that between RSU and vehicles nodes or among vehicles nodes is [0, 2*linkNum-1];
                                 2. range of links that between downloaders node and super sink node is [2*linkNum, 2*linkNum+downloaderNum); */
	struct Arc **arcRecord; ///< maintaining relationship of arc's predecessor when to call recordArcPath method after BFS or dijkstra traverse is called.
	int *dist;              ///< recording the time when a node is discovered.
	int *f;                 ///< recording the time when a node's all out degrees has been visited.
	int *pi;                ///< maintaining relationship of node's predecessor when BFS, DFS or dijkstra traverse is called.

	/** @name data members used by maximum flow algorithm. */
	///@{
	int superSink;          ///< ID of super sink.
	std::map<int, int>::iterator itPT; ///< iterator of container playTable.
	std::vector<double> interruptTimeTable; ///< store video play interrupt time of each downloader.
	std::vector<int> receivedAmountTable;   ///< store received data amount of each downloader.
	std::vector<std::pair<int, int> > arcPathRange; ///< store arc path index range of each downloader.
	std::vector<std::vector<int> > lackSlotsTable;  ///< store video play interrupt slot of each downloader.
	std::vector<std::vector<int> > chosenLinks; ///< stores arc IDs chosen to transmition in each slot.
	std::vector<std::vector<int> > chosenPaths; ///< stores arc paths chosen to transmition in each slot.
	std::vector<std::vector<int> > chosenSlots; ///< stores slots chosen to transmition of each arc path.
	std::vector<std::vector<int> > arcPathList; ///< stores all possible arc paths from RSU to all downloaders.
#if VERIFY_SCHEME
	std::list<std::vector<int> > fluxPathList;  ///< stores all flux paths from super source to super sink, which is certainly the result of this program.
#endif
	std::vector<std::vector<FluxScheme> > fluxSchemeList; ///< stores all flux schemes between each node pair, which is certainly the result of this program.
	std::vector<std::vector<int> >::iterator itAPL; ///< iterator of container arcPathList.
	///@}
};

#if VERIFY_SCHEME
bool verifyScheme(STAG& graph, double& totalInterruptTime, int& totalReceivedAmount);
#endif

#endif /* __STAG_H__ */
