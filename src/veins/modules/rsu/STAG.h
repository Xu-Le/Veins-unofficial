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

#include <cstdint>
#include <cstring>
#include <utility>
#include <time.h>
#include "veins/modules/application/ContentUtils.h"

#define INFO_STAG     1
#define DEBUG_STAG    0

/** Convenient class to calculate time elapsing. */
class Timer
{
public:
	explicit Timer(const char *s) : _note(s) { _start_time = clock(); }
	~Timer() { EV << _note.c_str() << _elapsed() << "s.\n"; }

	inline clock_t elapsed() { return clock() - _start_time; }
	inline double _elapsed() { return static_cast<double>(clock() - _start_time) / CLOCKS_PER_SEC; }

private:
	Timer(const Timer&);
	Timer& operator=(const Timer&);

private:
	std::string _note;
	clock_t _start_time;
};

/** Input to construct STAG, it is obtained by ContentRSU. */
class LinkTuple
{
public:
	LinkTuple(int _src, int _dst, int _slot_num) : src(_src), dst(_dst), slotNum(_slot_num) { bandwidth = new int[slotNum]; }
	~LinkTuple() { delete []bandwidth; }

	LinkTuple(const LinkTuple& rhs)
	{
		src = rhs.src;
		dst = rhs.dst;
		slotNum = rhs.slotNum;
		bandwidth = new int[slotNum];
		memcpy(bandwidth, rhs.bandwidth, slotNum*sizeof(int));
	}

	int src;
	int dst;
	int slotNum;
	int *bandwidth;

private:
	LinkTuple& operator=(const LinkTuple&);
};

/** Adjacency list structure representing the graph. */
class STAG
{
public:
	/** File segment store in vehicles. */
	struct Data
	{
		Data() : downloader(-1), size(0) {}

		int downloader;
		int size;
	};

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

		struct Arc *nextArc;    ///< next out degree arc of the node this arc derived.
	};

	/** Node structure of graph. */
	struct Node
	{
		Node();
		~Node();

		Data *_S; ///< continuous memory allocation, client code should use S instead.
		Data **S; ///< convenient matrix form S[i][j], where i is slot dimension and j is downloader dimension.

		struct Arc *firstArc; ///< first out degree arc of this node.
		struct Arc *lastArc;  ///< last out degree arc of this node.
	};

	STAG(int _nodeNum, int _linkNum, int _downloaderNum, int _slotNum, std::vector<int>& _downloaderArray,
			std::map<int, int>& _downloaderTable, std::map<int, int>& _remainingTable, std::map<int, int>& _playTable, std::map<int, int>& _demandingAmountTable);
	~STAG();

	/** @name regular methods of STAG. */
	///@{
	void construct(std::list<LinkTuple>& linkTuples);
	void undirectify(std::list<LinkTuple>& linkTuples);
	void destruct();
	///@}

	/**
	 * API function executing a shortest interrupt time first max flow algorithm of STAG.
	 */
	void maximumFlow();

	struct Arc* oppositeArc(int curID) { return &arcTable[curID ^ 1]; }

private:
	STAG(const STAG&);
	STAG& operator=(const STAG&);

	void _initNodeStorage();

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
	void __insertChosenSlot(std::vector<int>& vec, const int slot, const bool firstArc);
	void __eraseUnchosenArcPath();
	bool __isCollision(struct Arc *parc, struct Arc *qarc, const int slot);
	///@}

public:
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

	int nodeNum;
	int arcNum;
	int linkNum;
	int downloaderNum;
	int slotNum;
	int superSink; ///< ID of super sink.
	struct Node *NodeList;  ///< storing all the nodes in the graph.
	struct Arc *arcTable;   /**< 1. range of links that between RSU and vehicles nodes or among vehicles nodes is [0, 2*linkNum-1];
								 2. range of links that between downloaders node and super sink node is [2*linkNum, 2*linkNum+downloaderNum); */

	std::vector<int>& downloaderArray;   ///< reverse table of downloaderTable, cost only O(1).
	std::map<int, int>& downloaderTable; ///< maintaining the map from downloader node ID to its index.
	std::map<int, int>& remainingTable;  ///< maintaining the map from downloader node ID to its remaining data amount.
	std::map<int, int>& playTable;       ///< maintaining the map from downloader node ID to its play rate.
	std::map<int, int>& demandingAmountTable; ///< stores data amount that still needed of each downloader.
	std::map<int, int> prefetchAmountTable;   ///< stores data amount that should be prefetched of each downloader.
	std::vector<std::vector<FluxScheme> > fluxSchemeList; ///< stores all flux schemes between each node pair, which is certainly the result of this program.

	static int arcIDIncrement; ///< automatically increase ID for all arcs.
	static int gDownloaderNum; ///< helper variable used to construct Node array and Arc array.
	static int gSlotNum; ///< helper variable used to construct Node array and Arc array.

private:
	/** @name data members used by maximum flow algorithm. */
	///@{
	std::vector<double> interruptTimeTable; ///< store video play interrupt time of each downloader.
	std::vector<int> receivedAmountTable;   ///< store received data amount of each downloader.
	std::vector<std::pair<int, int> > arcPathRange; ///< store arc path index range of each downloader.
	std::vector<std::vector<int> > lackSlotsTable;  ///< store video play interrupt slot of each downloader.
	std::vector<std::vector<int> > chosenLinks; ///< stores arc IDs chosen to transmition in each slot.
	std::vector<std::vector<int> > chosenPaths; ///< stores arc paths chosen to transmition in each slot.
	std::vector<std::vector<int> > chosenSlots; ///< stores slots chosen to transmition of each arc path.
	std::vector<std::vector<int> > arcPathList; ///< stores all possible arc paths from RSU to all downloaders.
	std::vector<std::vector<int> >::iterator itAPL; ///< iterator of container arcPathList.
	///@}
};

#endif /* __STAG_H__ */
