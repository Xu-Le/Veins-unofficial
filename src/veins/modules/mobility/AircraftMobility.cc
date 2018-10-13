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

Define_Module(AircraftMobility);

void AircraftMobility::initialize(int stage)
{
	BaseMobility::initialize(stage);

	if (stage == 0)
	{
		circularTrajectory = par("circularTrajectory").boolValue();
		staticTrajectory = circularTrajectory || par("staticTrajectory").boolValue();
		if (staticTrajectory)
		{
			if (circularTrajectory)
			{
				double trajectoryRadius = par("trajectoryRadius").doubleValue();
				double flyingSpeed = par("flyingSpeed").doubleValue();
				initializeCircularTrajectory(trajectoryRadius, flyingSpeed);
			}
			else
				initializeTrajectory(par("trajectory").xmlValue());
			itTraj2 = trajectory.begin();
			itTraj = itTraj2;
			if (++itTraj2 == trajectory.end()) // stationary trajectory
				updateInterval = SimTime::ZERO; // no need to update position
			else
			{
				move.setStart(itTraj->first);
				move.setSpeed(itTraj->second);
				move.setDirectionByTarget(itTraj2->first);
			}
		}
	}
}

void AircraftMobility::finish()
{
	if (staticTrajectory)
		trajectory.clear();

	BaseModule::finish();
}

void AircraftMobility::getMove(Coord& pos, Coord& speed)
{
	pos = move.getStartPos();
	speed = move.getDirection();
	speed *= move.getSpeed();
}

void AircraftMobility::setMove(Coord dir, double speed, bool keepCurPos, Coord pos)
{
	if (keepCurPos)
		pos = move.getPositionAt();
	move.setStart(pos);
	move.setDirectionByVector(dir);
	move.setSpeed(speed);

	fixIfHostGetsOutside();

	updatePosition();
}

void AircraftMobility::makeMove()
{
	Coord curPos = move.getPositionAt();
	double curSpeed = move.getSpeed();
	if (curPos.sqrdist(itTraj2->first) <= curSpeed*curSpeed) // have reached destination
	{
		itTraj = itTraj2;
		if (++itTraj2 == trajectory.end()) // circular list
			itTraj2 = trajectory.begin();
		move.setStart(itTraj->first);
		move.setSpeed(itTraj->second);
		move.setDirectionByTarget(itTraj2->first);
	}
	else
		move.setStart(curPos);

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

	// Coord dummy = Coord::ZERO;
	// double dummyAngle = -1.0;
	// handleIfOutside(BorderPolicy::RAISEERROR, pos, dummy, dummy, dummyAngle);
}

void AircraftMobility::initializeTrajectory(cXMLElement *xmlConfig)
{
	if (xmlConfig == nullptr)
		throw cRuntimeError("No trajectory plan configuration file specified.");

	cXMLElementList planList = xmlConfig->getElementsByTagName("TrajectoryPlan");

	if (planList.empty())
		throw cRuntimeError("No trajectory plan configuration items specified.");

	for (cXMLElementList::iterator iter = planList.begin(); iter != planList.end(); ++iter)
	{
		cXMLElement *trajectoryPlan = *iter;

		const char* name = trajectoryPlan->getAttribute("type");

		if (strcmp(name, getParentModule()->getFullName()) == 0)
		{
			cXMLElementList parameters = trajectoryPlan->getElementsByTagName("parameter");

			ASSERT( parameters.size() % 2 == 0 );

			EV << logName() << "'s trajectory plan list as follows:\n";

			for (size_t i = 0; i < parameters.size(); i += 2)
			{
				Coord pos;
				double speed = 0.0;

				const char *name = parameters[i]->getAttribute("name");
				const char *type = parameters[i]->getAttribute("type");
				const char *value = parameters[i]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value.");

				if (strcmp(name, "pos") == 0 && strcmp(type, "coord") == 0)
				{
					char *p = const_cast<char*>(value), *q = p;
					while (*p != ',')
						++p;
					*p = '\0';
					pos.x = atof(q);
					*p++ = ',';
					q = p;
					while (*p != ',')
						++p;
					*p = '\0';
					pos.y = atof(q);
					*p++ = ',';
					q = p;
					pos.z = atof(q);
				}
				else
					throw cRuntimeError("Invalid parameter, name or type undefined.");

				name = parameters[i+1]->getAttribute("name");
				type = parameters[i+1]->getAttribute("type");
				value = parameters[i+1]->getAttribute("value");
				if (name == 0 || type == 0 || value == 0)
					throw cRuntimeError("Invalid parameter, could not find name, type or value.");

				if (strcmp(name, "speed") == 0 && strcmp(type, "double") == 0)
					speed = atof(value);
				else
					throw cRuntimeError("Invalid parameter, name or type undefined.");

				EV << "    pos: " << pos.info() << ", speed: " << speed << std::endl;
				trajectory.push_back(std::pair<Coord, double>(pos, speed));
			}

			break;
		}
	}
}

void AircraftMobility::initializeCircularTrajectory(double R, double v)
{
	Coord O = move.getStartPos(); // center of circle
	const int sectors = 36; // number of sectors
	const double theta = 2*M_PI/sectors;
	EV << logName() << "'s trajectory plan list as follows:\n";
	for (int i = 0; i < sectors; ++i)
	{
		double phi = i * theta;
		Coord convexP(O.x + R*cos(phi), O.y + R*sin(phi));
		EV << "    pos: " << convexP.info() << ", speed: " << v << std::endl;
		trajectory.push_back(std::pair<Coord, double>(convexP, v));
	}
}
