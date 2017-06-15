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

#ifndef __MOBILITYOBSERVER_H__
#define __MOBILITYOBSERVER_H__

#include "veins/base/utils/Coord.h"
#include "veins/base/utils/SimpleAddress.h"

/**
 * @brief Observer class for recording the mobility of vehicles.
 *
 * @author Xu Le
 * @ingroup routingLayer
 */
class MobilityObserver
{
public:
	/** @brief Singleton method, return the only instance. */
	static MobilityObserver* Instance();

	~MobilityObserver();

	/** @name basic operations. */
	///@{
	void insert(LAddress::L3Type addr, Coord pos, Coord speed);
	void update(LAddress::L3Type addr, Coord pos, Coord speed);
	void erase(LAddress::L3Type addr);
	///@}

private:
	MobilityObserver() {}

	static MobilityObserver *_instance;

public:
	std::map<LAddress::L3Type, Coord> globalPosition;
	std::map<LAddress::L3Type, Coord> globalSpeed;
};

#endif /* __MOBILITYOBSERVER_H__ */
