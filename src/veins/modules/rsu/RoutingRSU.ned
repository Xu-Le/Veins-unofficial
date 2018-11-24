//
// Copyright (C) 2016-2019 Xu Le <xmutongxinXuLe@163.com>
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

package org.car2x.veins.modules.rsu;

import org.car2x.veins.modules.rsu.BaseRSU;

simple RoutingRSU extends BaseRSU
{
	@class(RoutingRSU);
	@display("i=block/routing");

	int beaconLengthBits = default(1216bit) @unit(bit); // the length of a beacon packet
	int beaconPriority = default(0); // the access category of the beacon messages
	int routingLengthBits = default(960bit) @unit(bit); // the length of a routing packet
	int routingPriority = default(1); // the access category of the routing messages
	int dataLengthBits = default(18272bit) @unit(bit); // the length of a data packet 8 * (2312-20-8)
	int dataPriority = default(2); // the access category for data packets

	int totalUavNum = default(1); // the total number of UAV to be emitted per RSU
	int emitFirstUavAt = default(60); // when to emit the first UAV
	int theta = default(20); // sector granularity measured in degree
	int expand = default(3); // expand to how many adjacent sectors to smooth the density in average

	double beaconInterval = default(1s) @unit(s); // the intervall between 2 beacon messages
	double examineUavsInterval = default(2s) @unit(s); // the intervall between 2 examine UAV whether in communication range messages
	double emitUavInterval = default(120s) @unit(s); // the interval between 2 emit UAV events
	double attainDensityInterval = default(10s) @unit(s); // the interval between 2 attain density events
	double UavElapsed = default(3s) @unit(s); // if exceeds this time, assuming has lose connectivity with the UAV
}
