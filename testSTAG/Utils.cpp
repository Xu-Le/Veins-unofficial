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

#include "Utils.h"

#define MAX_LINE_LENGTH    1000

extern int log_level;

extern int nodeNum;
extern int linkNum;
extern int downloaderNum;
extern int slotNum;

extern std::map<int, std::string> storageTable;
extern std::map<int, int> playTable;
extern std::map<int, int> downloaderTable;
extern std::vector<int> downloaderArray;

static void parseLink(char buf[], int& srcNode, int& dstNode, int *bandwidth)
{
	std::string str(buf);
	std::string subStr;
	size_t old = 0, cur = 0;
	int idx = -2;
	while (cur < str.length())
	{
		if (str[cur] == ' ')
		{
			subStr = str.substr(old, cur-old);
			if (idx >= 0)
				bandwidth[idx] = atoi(subStr.c_str());
			else if (idx == -1)
				dstNode = atoi(subStr.c_str());
			else // idx == -2
				srcNode = atoi(subStr.c_str());
			++idx;
			old = ++cur;
		}
		else
			++cur;
	}
	if (!str.empty())
	{
		subStr = str.substr(old, cur-old);
		bandwidth[idx] = atoi(subStr.c_str());
	}
}

static void parseVehicle(char buf[])
{
	std::string str(buf);
	std::string subStr;
	for (size_t cur = 0; cur < str.length(); ++cur)
	{
		if (str[cur] == ' ')
		{
			subStr = str.substr(0, cur);
			int vehicle = atoi(subStr.c_str());
			subStr = str.substr(cur+1, str.length()-cur-1);
			storageTable.insert(std::pair<int, std::string>(vehicle, subStr));
			break;
		}
	}
}

void parseInput(const char *filename, std::list<LinkTuple>& linkTuples)
{
	FILE *fd = fopen(filename, "r");
	if (fd == NULL)
	{
		error_log("Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	int curLine = 0;
	fscanf(fd, "%d %d %d %d\n", &nodeNum, &linkNum, &downloaderNum, &slotNum);
	downloaderArray.resize(downloaderNum);

	int srcNode = 0, dstNode = 0, *bandwidth = new int[slotNum];
	char lineBuf[MAX_LINE_LENGTH];
	for (curLine = 3; curLine < 3+linkNum; ++curLine)
	{
		fgets(lineBuf, MAX_LINE_LENGTH, fd);
		parseLink(lineBuf, srcNode, dstNode, bandwidth);
		linkTuples.emplace_back(srcNode, dstNode, slotNum);
		memcpy(linkTuples.back().bandwidth, bandwidth, slotNum*sizeof(int));
	}
	delete []bandwidth;

	fgets(lineBuf, MAX_LINE_LENGTH, fd);

	for (++curLine; curLine < 4+linkNum+nodeNum; ++curLine)
	{
		fgets(lineBuf, MAX_LINE_LENGTH, fd);
		parseVehicle(lineBuf);
	}

	int nodeID = -1, nodeIdx = 0, playRate = 0;
	for (++curLine; curLine < 5+linkNum+nodeNum+downloaderNum; ++curLine, ++nodeIdx)
	{
		fscanf(fd, "%d %d\n", &nodeID, &playRate);
		playTable.insert(std::pair<int, int>(nodeID, playRate));
		downloaderTable.insert(std::pair<int, int>(nodeID, nodeIdx));
		downloaderArray[nodeIdx] = nodeID;
	}

	fclose(fd);
}

void printInput(std::list<LinkTuple>& linkTuples)
{
	if (log_level < DEBUG_LEVEL)
		return;

	printf("nodeNum: %d, linkNum: %d, downloaderNum: %d, slotNum: %d\n\n", nodeNum, linkNum, downloaderNum, slotNum);
	for (std::list<LinkTuple>::iterator itLink = linkTuples.begin(); itLink != linkTuples.end(); ++itLink)
	{
		printf("%d %d |", itLink->src, itLink->dst);
		for (int k = 0; k < slotNum; ++k)
			printf(" %d", itLink->bandwidth[k]);
		printf("\n");
	}
	printf("\n");

	for (std::map<int, std::string>::iterator itST = storageTable.begin(); itST != storageTable.end(); ++itST)
		printf("%d %s", itST->first, itST->second.c_str());
	printf("\n");

	for (std::map<int, int>::iterator itPT = playTable.begin(); itPT != playTable.end(); ++itPT)
		printf("%d %d\n", itPT->first, itPT->second);
	printf("\n");
}

void outputToFile(const char *filename, std::list<std::vector<int> >& fluxPathList)
{
	FILE *fd = fopen(filename, "w");
	if (fd == NULL)
	{
		error_log("Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	for (std::list<std::vector<int> >::iterator iter = fluxPathList.begin(); iter != fluxPathList.end(); ++iter)
	{
		std::vector<int> &fluxPath = *iter; // alias
		for (size_t i = 0; i < fluxPath.size()-1; ++i)
			fprintf(fd, "%d ", fluxPath[i]);
		fprintf(fd, "%d\n", fluxPath.back());
	}

	fclose(fd);
}

bool binarySearch(const std::vector<int>& vec, const int key)
{
	int low = 0, high = static_cast<int>(vec.size())-1, mid = 0;
	while (low <= high)
	{
		mid = low + ((high - low) >> 1);
		if (key == vec[mid])
			return true;
		else if (key < vec[mid])
			high = mid - 1;
		else
			low = mid + 1;
	}
	return false;
}

bool vectorFind(std::vector<int>& vec, const int key)
{
	return std::find(vec.begin(), vec.end(), key) != vec.end();
}

void vectorRemove(std::vector<int>& vec, const int key)
{
	vec.erase(std::remove(vec.begin(), vec.end(), key), vec.end());
}


////////////////    convenient functions of std::vector and std::list    ////////////////
void printVector(std::vector<int>& _vector, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::vector<int>::iterator iter = _vector.begin(); iter != _vector.end(); ++iter)
			printf("%d ", *iter);
		printf("\n");
	}
}

void printVector(std::vector<double>& _vector, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::vector<double>::iterator iter = _vector.begin(); iter != _vector.end(); ++iter)
			printf("%f ", *iter);
		printf("\n");
	}
}

void printList(std::list<int>& _list, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::list<int>::iterator iter = _list.begin(); iter != _list.end(); ++iter)
			printf("%d ", *iter);
		printf("\n");
	}
}

void printList(std::list<double>& _list, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::list<double>::iterator iter = _list.begin(); iter != _list.end(); ++iter)
			printf("%f ", *iter);
		printf("\n");
	}
}
