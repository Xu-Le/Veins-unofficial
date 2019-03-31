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

	int routingLengthBits = default(424bit) @unit(bit); // the length of a routing packet
	int routingPriority = default(0); // the access category of the routing messages
	// see "Figure 8-33——A-MSDU Subframe structure for Mesh Data" of IEEE Std 802.11-2012
	int dataLengthBits = default(18432bit) @unit(bit); // the length of a data packet 8*2304
	int dataPriority = default(2); // the access category for data packets
	int bufferQueueCap = default(64); // the capacity of data packets buffer queue
	int pktNetLayerDelay = default(1000); // network layer to MAC layer measured in us

	int flyingInterval = default(500ms) @unit(ms); // the interval of actually updating position while flying
	int decideInterval = default(12000ms) @unit(ms); // the interval of making a decision
	int stopFlyingMs = default(6000ms) @unit(ms); // when remainingMs reach this value, stop flying
	int theta = default(30); // sector granularity measured in degree
	int expand = default(3); // expand to how many adjacent sectors to smooth the density in average

	double minGap = default(180); // minimum distance between two UAVs
	double k_a = default(100);    // coefficient of attractive force of ground users
	double K_r = default(0.01);   // coefficient of repulsive force of other UAVs
	double R_opt = default(250);  // distance threshold within which bring about repulsive force
	double recvPowerThres = default(-81dBm) @unit(dBm); // received power threshold below which the link is going to break
}
