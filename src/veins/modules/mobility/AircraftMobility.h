//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
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

    virtual void nextPosition(const Coord& position, double speed, double angle);

private:
    void updateDisplayString();

    /**< called after each read to check for (and handle) invalid positions */
    void fixIfHostGetsOutside() override;

private:
    Coord curPos; ///< updated by updatePosition()

    double speed; ///< updated by updatePosition()
    double angle; ///< updated by updatePosition()

    cOutVector currentPosXVec;  ///< vector plotting posx
    cOutVector currentPosYVec;  ///< vector plotting posy
    cOutVector currentPosZVec;  ///< vector plotting posz
    cOutVector currentSpeedVec; ///< vector plotting speed
};

#endif /* __AIRCRAFTMOBILITY_H__ */
