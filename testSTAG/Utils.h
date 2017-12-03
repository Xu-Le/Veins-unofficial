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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <cstdint>
#include <cstring>
#include <vector>
#include <list>
#include <stack>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <utility>

#include "Timer.h"

/** File segment store in vehicles. */
struct Data
{
	Data() : downloader(-1), size(0) {}

	int downloader;
	int size;
};

class LinkTuple
{
public:
	LinkTuple(int _src, int _dst, int _slot_num) : src(_src), dst(_dst) { bandwidth = new int[_slot_num]; }
	~LinkTuple() { delete []bandwidth; }

	int src;
	int dst;
	int *bandwidth;

private:
	LinkTuple(const LinkTuple&);
	LinkTuple& operator=(const LinkTuple&);
};

void parseInput(const char *filename, std::list<LinkTuple>& linkTuples);

void printInput(std::list<LinkTuple>& linkTuples);

void outputToFile(const char *filename, std::list<std::vector<int> >& fluxPathList);

bool binarySearch(const std::vector<int>& vec, const int key);

bool vectorFind(std::vector<int>& vec, const int key);

void vectorRemove(std::vector<int>& vec, const int key);


////////////////    convenient functions of std::vector and std::list    ////////////////
void printVector(std::vector<int>& _vector, const char *_note, int _log_level = INFO_LEVEL);

void printVector(std::vector<double>& _vector, const char *_note, int _log_level = INFO_LEVEL);

void printList(std::list<int>& _list, const char *_note, int _log_level = INFO_LEVEL);

void printList(std::list<double>& _list, const char *_note, int _log_level = INFO_LEVEL);

#endif /* __UTILS_H__ */
