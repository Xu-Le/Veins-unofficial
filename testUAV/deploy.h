//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __DEPLOY_H__
#define __DEPLOY_H__

#include "uav.h"

class Solution
{
public:
	Solution();

	int initialize(int _num);
	int handleEvent(int who, int what, void *data);

	void schedule(int period, int who, int what, int priority=1, void *data=NULL);

	void attainVehiclesNearby(const Point& coord, std::list<int>& unservedList);
	void attainNeighbors(const Point& coord, int selfIndex, std::list<UAV*>& neighbors);

private:
	Solution(const Solution&);
	Solution& operator=(const Solution&);

public:
	int eventSequence;
	int uavNum;
	RSU rsu;
	std::vector<UAV> uavs;
	std::vector<std::list<int> > accessPaths;
	std::priority_queue<std::pair<int, EventInfo>, std::vector<std::pair<int, EventInfo> >, std::greater<std::pair<int, EventInfo> > > eventQ;
};

#endif /* __DEPLOY_H__ */
