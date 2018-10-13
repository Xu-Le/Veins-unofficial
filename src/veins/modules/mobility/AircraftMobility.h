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

	void getMove(Coord& pos, Coord& speed);
	void setMove(Coord dir, double speed, bool keepCurPos=true, Coord pos=Coord::ZERO);

private:
	void makeMove() override;
	void fixIfHostGetsOutside() override;

	/** @brief initialize trajectory through configured xmlfile. */
	void initializeTrajectory(cXMLElement *xmlConfig);
	/** @brief initialize circular trajectory. */
	void initializeCircularTrajectory(double R, double v);

private:
	bool staticTrajectory;   ///< use static trajectory specified by std::list<> trajectory.
	bool circularTrajectory; ///< use circular trajectory.
	std::list<std::pair<Coord, double> > trajectory; ///< trajectory of the aircraft.
	std::list<std::pair<Coord, double> >::iterator itTraj;  ///< iterator of std::list<> trajectory.
	std::list<std::pair<Coord, double> >::iterator itTraj2; ///< itTraj2 == ++itTraj.
};

#endif /* __AIRCRAFTMOBILITY_H__ */
