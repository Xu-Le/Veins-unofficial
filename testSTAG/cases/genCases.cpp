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

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>

#include "Coord.h"

const Coord Coord::ZERO = Coord(0.0, 0.0, 0.0);

int nodeNum = 0;
int downloaderNum = 1;
int slotNum = 8;

static int rateTable[50] = { 5600, 5600, 5500, 5500, 5500, 5500, 5400, 5400, 5300, 5300,
5300, 5300, 5200, 5200, 5200, 5200, 5200, 5200, 5200, 5200,
5100, 5100, 5100, 5100, 5000, 4900, 4700, 4400, 4100, 3750,
3400, 3000, 2600, 2100, 1600, 1150, 700, 550, 400, 400,
350, 350, 300, 300, 300, 250, 250, 250, 200, 200 };

static std::vector<std::vector<Coord> > nodePos;
static std::vector<std::vector<Coord> > nodeSpeed;

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

void parseInput(const char *trafficFile)
{
	std::ifstream fin(trafficFile, std::ios_base::in);
	if (!fin.is_open())
	{
		std::cerr << "cannot open file " << trafficFile << "!\n";
		exit(EXIT_FAILURE);
	}

	char inputLine[255];
	while (fin.getline(inputLine, 255) && inputLine[0] != '\0')
		++nodeNum;

	fin.clear();
	fin.seekg(0);

	std::vector<Coord> slotPadding(slotNum+1, Coord::ZERO);
	for (int nodeId = 0; nodeId < nodeNum; ++nodeId)
	{
		nodeSpeed.push_back(slotPadding);
		nodePos.push_back(slotPadding);
	}

	int id = 0;
	double pos_x, pos_y, speed_x, speed_y;
	for (int i = 0; i < nodeNum; ++i)
	{
		fin >> id >> pos_x >> pos_y >> speed_x >> speed_y;
		nodePos[i][0] = Coord(pos_x, pos_y);
		nodeSpeed[i][0] = Coord(speed_x, speed_y);
	}
	for (int j = 1; j <= slotNum; ++j)
	{
		nodePos[0][j] = nodePos[0][0]; // node 0 is RSU
		nodeSpeed[0][j] = Coord::ZERO; // RSU has no speed since it cannot move
	}

	fin.close();
}

void predictVehicleMobility()
{
	for (int j = 1; j <= slotNum; ++j)
		for (int i = 1; i < nodeNum; ++i)
			nodePos[i][j] = nodePos[i][j-1] + nodeSpeed[i][0];
}

void predictLinkBandwidth(const char *caseFile)
{
	int linkNum = nodeNum * (nodeNum - 1) / 2;

	predictVehicleMobility();

	std::vector<int> distPadding(nodeNum);
	std::vector<std::vector<int> > nodeDist;
	for (int nodeId = 0; nodeId < nodeNum; ++nodeId)
		nodeDist.push_back(distPadding);

	int srcNode = 0, dstNode = 0;
	std::list<LinkTuple> linkTuples;
	for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
	{
		for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
		{
			linkTuples.push_back(LinkTuple(srcNode, dstNode, slotNum));
			memset(linkTuples.back().bandwidth, 0, slotNum*sizeof(int));
			nodeDist[srcNode][dstNode] = Coord::length(nodePos[srcNode][0], nodePos[dstNode][0]);
		}
	}

	std::list<LinkTuple>::iterator itLT = linkTuples.begin();
	for (int j = 0; j < slotNum; ++j)
	{
		itLT = linkTuples.begin();
		for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
		{
			for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
			{
				int curDist = nodeDist[srcNode][dstNode];
				int nextDist = Coord::length(nodePos[srcNode][j+1], nodePos[dstNode][j+1]);
				nodeDist[srcNode][dstNode] = nextDist;
				if (curDist < 250 || nextDist < 250)
				{
					int minIdx = curDist/5, maxIdx = nextDist/5;
					if (minIdx > maxIdx)
						std::swap(minIdx, maxIdx);
					int maxIdx_ = std::min(49, maxIdx);
					for (int i = minIdx; i <= maxIdx_; ++i)
						itLT->bandwidth[j] += rateTable[i];
					itLT->bandwidth[j] /= maxIdx - minIdx + 1;
					itLT->bandwidth[j] /= 8;
					++itLT->bandwidth[j];
				}
				else
					itLT->bandwidth[j] = 0;
				++itLT;
			}
		}
	}

	std::ofstream fout(caseFile, std::ios_base::out | std::ios_base::trunc);
	if (!fout.is_open())
	{
		std::cerr << "cannot open file " << caseFile << "!\n";
		exit(EXIT_FAILURE);
	}

	fout << nodeNum << " " << linkNum << " " << downloaderNum << " " << slotNum << "\n\n";
	for (itLT = linkTuples.begin(); itLT != linkTuples.end(); ++itLT)
	{
		fout << itLT->src << " " << itLT->dst;
		for (int j = 0; j < slotNum; ++j)
			fout << " " << itLT->bandwidth[j];
		fout << "\n";
	}
	fout << std::endl;

	for (int nodeId = 0; nodeId < nodeNum; ++nodeId)
	{
		fout << nodeId;
		for (int d = 0; d < downloaderNum; ++d)
		{
			if (nodeId > 0)
				fout << " 1 0";
			else // RSU self
				fout << " 1 99999999";
		}
		fout << "\n";
	}
	fout << std::endl;

	fout.close();
}

static void printHelp()
{
	printf("Usage:\n    ./genCases [-s] trafficx.txt casex.txt\n");
	printf("Example:\n    ./genCases -s 8 traffic0.txt case0.txt\n");
}

int main(int argc, char *argv[])
{
	if (argc != 3 && argc != 5)
	{
		printHelp();
		exit(EXIT_FAILURE);
	}

	if (argc == 5)
		slotNum = atoi(argv[2]);

	parseInput(argv[argc-2]);
	predictLinkBandwidth(argv[argc-1]);
	return 0;
}
