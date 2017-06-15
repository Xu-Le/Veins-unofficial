//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/routing/MobilityObserver.h"

MobilityObserver* MobilityObserver::_instance = nullptr;

MobilityObserver* MobilityObserver::Instance()
{
	if (_instance == nullptr) // Lazy initialization
		_instance = new MobilityObserver;
	return _instance;
}

void MobilityObserver::insert(LAddress::L3Type addr, Coord pos, Coord speed)
{
	globalPosition.insert(std::pair<LAddress::L3Type, Coord>(addr, pos));
	globalSpeed.insert(std::pair<LAddress::L3Type, Coord>(addr, speed));
}

void MobilityObserver::update(LAddress::L3Type addr, Coord pos, Coord speed)
{
	globalPosition[addr] = pos;
	globalSpeed[addr] = speed;
}

void MobilityObserver::erase(LAddress::L3Type addr)
{
	globalPosition.erase(addr);
	globalSpeed.erase(addr);
}

MobilityObserver::~MobilityObserver()
{
	globalPosition.clear();
	globalSpeed.clear();
}
