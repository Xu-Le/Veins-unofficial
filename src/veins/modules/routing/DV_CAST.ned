//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

package org.car2x.veins.modules.routing;

import org.car2x.veins.modules.wave.BaseWaveApplLayer;

simple DV_CAST extends BaseWaveApplLayer
{
	@class(DV_CAST);
	@display("i=block/routing");
	int warningLengthBits = default(960bit) @unit(bit); // the length of a warning packet
	int warningPriority = default(0); // the access category of the warning messages
	double WAIT_TIME = default(0.01s) @unit(s);

	xml warningPlan;
}
