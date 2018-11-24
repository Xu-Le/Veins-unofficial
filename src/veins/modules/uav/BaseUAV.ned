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

import org.car2x.veins.base.modules.BaseApplLayer;

simple BaseUAV extends BaseApplLayer
{
	@class(BaseUAV);
	@display("i=block/process");

	bool dataOnSch = default(true); // tells the applayer whether to use a service channel for datapackets or the control channel

	int beaconLengthBits = default(1216bit) @unit(bit); // the length of a beacon packet
	int beaconPriority = default(0); // the access category of the beacon messages

	double beaconInterval = default(1s) @unit(s); // the interval between 2 beacon messages
	double examineVehiclesInterval = default(2s) @unit(s);  // the interval between 2 examine vehicle whether in communication range messages
	double examineNeighborsInterval = default(2s) @unit(s); // the interval between 2 examine neighbor whether in communication range messages
	double vehicleElapsed = default(3s) @unit(s);  // if exceeds this time, assuming has lose connectivity with the vehicle
	double neighborElapsed = default(3s) @unit(s); // if exceeds this time, assuming has lose connectivity with the neighbor
}
