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

simple GPCR extends BaseWaveApplLayer
{
	@class(GPCR);
	@display("i=block/routing");
	int routingLengthBits = default(960bit) @unit(bit); // the length of a routing packet
	int routingPriority = default(2); // the access category of the routing messages
	int dataLengthBits = default(18272bit) @unit(bit); //the length of a data packet 8*(2312-20-8)
	int dataPriority = default(3); //the access category for data packets

	xml routingPlan;
}
