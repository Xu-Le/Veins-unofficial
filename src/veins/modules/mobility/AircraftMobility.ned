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

package org.car2x.veins.modules.mobility;

import org.car2x.veins.base.modules.BaseMobility;

simple AircraftMobility extends BaseMobility
{
	parameters:
		@class(AircraftMobility);
		@display("i=block/cogwheel");
		bool staticTrajectory = default(true);
		bool circularTrajectory = default(false);
		double trajectoryRadius = default(50m) @unit(m); // radius of the circle if using circular trajectory
		double flyingSpeed = default(5); // constant flying speed if using circular trajectory
		double updateInterval = default(1s) @unit(s);

		xml trajectory;
}
