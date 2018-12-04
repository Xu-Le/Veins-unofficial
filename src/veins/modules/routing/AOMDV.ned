//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

simple AOMDV extends BaseWaveApplLayer
{
	@class(AOMDV);
	@display("i=block/routing");
	int routingLengthBits = default(40bit) @unit(bit); // the length of a routing packet
	int routingPriority = default(0); // the access category of the routing messages
	// see "Figure 8-33——A-MSDU Subframe structure for Mesh Data" of IEEE Std 802.11-2012
	int dataLengthBits = default(18432bit) @unit(bit); // the length of a data packet 8*2304
	int dataPriority = default(2); // the access category for data packets
	int bufferQueueCap = default(64); // the capacity of data packets buffer queue

	int pktNetLayerDelay = default(500);  // network layer to MAC layer measured in us
	int pktApplLayerDelay = default(500); // application layer to network layer measured in us

	xml routingPlan;
	xml triggerPlan;
}
