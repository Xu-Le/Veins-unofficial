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

		curPos.x = par("x").doubleValue();
		curPos.y = par("y").doubleValue();
		curPos.z = par("z").doubleValue();

		move.setStart(curPos);
		move.setDirectionByVector(Coord(cos(angle), -sin(angle)));
		move.setSpeed(0);

		WATCH(speed);
		WATCH(angle);
	}
}

void AircraftMobility::finish()
{
	BaseModule::finish();
}

void AircraftMobility::nextPosition(const Coord& position, double speed, double angle)
{
	curPos = position;
	this->speed = speed;
	this->angle = angle;

	// keep statistics (for current step)
	currentPosXVec.record(move.getStartPos().x);
	currentPosYVec.record(move.getStartPos().y);
	currentPosZVec.record(move.getStartPos().z);
	currentSpeedVec.record(speed);

	move.setStart(position);
	move.setDirectionByVector(Coord(cos(angle), -sin(angle)));
	move.setSpeed(speed);

	if (hasGUI()) updateDisplayString();
	fixIfHostGetsOutside();

	updatePosition();
}

void AircraftMobility::updateDisplayString()
{
	ASSERT(angle >= -M_PI && angle < M_PI);

	getParentModule()->getDisplayString().setTagArg("b", 2, "rect");
	getParentModule()->getDisplayString().setTagArg("b", 3, "red");
	getParentModule()->getDisplayString().setTagArg("b", 4, "red");
	getParentModule()->getDisplayString().setTagArg("b", 5, "0");

	if (angle < -M_PI + 0.5 * M_PI_4 * 1) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2190");
		getParentModule()->getDisplayString().setTagArg("b", 0, "4");
		getParentModule()->getDisplayString().setTagArg("b", 1, "2");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 3) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2199");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 5) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2193");
		getParentModule()->getDisplayString().setTagArg("b", 0, "2");
		getParentModule()->getDisplayString().setTagArg("b", 1, "4");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 7) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2198");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 9) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2192");
		getParentModule()->getDisplayString().setTagArg("b", 0, "4");
		getParentModule()->getDisplayString().setTagArg("b", 1, "2");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 11) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2197");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 13) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2191");
		getParentModule()->getDisplayString().setTagArg("b", 0, "2");
		getParentModule()->getDisplayString().setTagArg("b", 1, "4");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 15) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2196");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2190");
		getParentModule()->getDisplayString().setTagArg("b", 0, "4");
		getParentModule()->getDisplayString().setTagArg("b", 1, "2");
	}
}

void AircraftMobility::fixIfHostGetsOutside()
{
	Coord pos = move.getStartPos();
	Coord dummy = Coord::ZERO;
	double dum;

	bool outsideX = (pos.x < 0) || (pos.x >= playgroundSizeX());
	bool outsideY = (pos.y < 0) || (pos.y >= playgroundSizeY());
	bool outsideZ = (!world->use2D()) && ((pos.z < 0) || (pos.z >= playgroundSizeZ()));
	if (outsideX || outsideY || outsideZ)
		error("Tried moving host to (%f, %f) which is outside the playground", pos.x, pos.y);

	handleIfOutside(RAISEERROR, pos, dummy, dummy, dum);
}
