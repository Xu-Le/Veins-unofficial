//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

package org.car2x.veins.modules.uav;

import org.car2x.veins.modules.uav.BaseUAV;

simple RoutingUAV extends BaseUAV
{
	@class(RoutingUAV);
	@display("i=block/routing");

	int routingLengthBits = default(960bit) @unit(bit); // the length of a routing packet
	int routingPriority = default(2); // the access category of the routing messages
	int dataLengthBits = default(18272bit) @unit(bit); // the length of a data packet 8 * (2312-20-8)
	int dataPriority = default(3); // the access category for data packets
	int flyingInterval = default(500ms) @unit(ms); // the interval of actually updating position while flying
	int decideInterval = default(10000ms) @unit(ms); // the interval of making a decision
	int stopFlyingMs = default(5000ms) @unit(ms); // when remainingMs reach this value, stop flying
	int theta = default(20); // sector granularity measured in degree
	int expand = default(3); // expand to how many adjacent sectors to smooth the density in average

	double minGap = default(200); // minimum distance between two UAVs
	double k_a = default(1000);   // coefficient of attractive force of ground users
	double K_r = default(2000);   // coefficient of repulsive force of other UAVs
	double R_opt = default(200);  // distance threshold within which bring about repulsive force
}
