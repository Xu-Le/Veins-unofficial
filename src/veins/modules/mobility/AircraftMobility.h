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

#ifndef __AIRCRAFTMOBILITY_H__
#define __AIRCRAFTMOBILITY_H__

#include "veins/base/modules/BaseMobility.h"
#include "veins/base/utils/FindModule.h"

class AircraftMobility : public BaseMobility
{
public:
	AircraftMobility() : BaseMobility() {}
	~AircraftMobility() {}

	virtual void initialize(int) override;
	virtual void finish() override;

private:
	void makeMove() override;
	void fixIfHostGetsOutside() override;

private:
	cOutVector currentPosXVec;  ///< vector plotting posx
	cOutVector currentPosYVec;  ///< vector plotting posy
	cOutVector currentPosZVec;  ///< vector plotting posz
	cOutVector currentSpeedVec; ///< vector plotting speed
};

#endif /* __AIRCRAFTMOBILITY_H__ */
