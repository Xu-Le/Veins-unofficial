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

#include "veins/modules/mobility/AircraftMobility.h"

void AircraftMobility::initialize(int stage)
{
	if (stage == 0)
	{
		BaseMobility::initialize(stage);

		currentPosXVec.setName("posx");
		currentPosYVec.setName("posy");
		currentPosZVec.setName("posz");
		currentSpeedVec.setName("speed");
	}
}

void AircraftMobility::finish()
{
	BaseModule::finish();
}

void AircraftMobility::makeMove()
{
	// keep statistics (for current step)
	currentPosXVec.record(move.getStartPos().x);
	currentPosYVec.record(move.getStartPos().y);
	currentPosZVec.record(move.getStartPos().z);
	currentSpeedVec.record(move.getSpeed());

	move.setDirectionByVector(Coord(1.0, 0.0));
	move.setSpeed(uniform(5.0, 10.0));
	SimTime nextInstant = simTime() + updateInterval;
	Coord nextPosition = move.getPositionAt(nextInstant);
	move.setStart(nextPosition);

	fixIfHostGetsOutside();
}

void AircraftMobility::fixIfHostGetsOutside()
{
	Coord pos = move.getStartPos();

	bool outsideX = pos.x < 0 || pos.x >= playgroundSizeX();
	bool outsideY = pos.y < 0 || pos.y >= playgroundSizeY();
	bool outsideZ = !world->use2D() && (pos.z < 0 || pos.z >= playgroundSizeZ());
	if (outsideX || outsideY || outsideZ)
		error("Tried moving host to (%f, %f) which is outside the playground", pos.x, pos.y);

	Coord dummy = Coord::ZERO;
	double dummyAngle = -1.0;
	handleIfOutside(BorderPolicy::RAISEERROR, pos, dummy, dummy, dummyAngle);
}
