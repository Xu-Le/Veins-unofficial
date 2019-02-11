//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>, 2016 Xu Le <xmutongxinXuLe@163.com>
//
// Documentation for these modules is at http://veins.car2x.org/
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

package org.car2x.veins.modules.wave;

import org.car2x.veins.base.modules.BaseApplLayer;
import org.car2x.veins.modules.wave.IHeterogeneousLayer;

//
// WAVE application layer base class.
//
// @author David Eckhoff, Xu Le
//
// @see BaseApplLayer
// @see Nic80211p
// @see Mac1609_4
// @see PhyLayer80211p
//
simple BaseWaveApplLayer extends BaseApplLayer like IHeterogeneousLayer
{
	@class(BaseWaveApplLayer);
	@display("i=block/routing");

	bool sendWhileParking  = default(false); // send messages when the car is parked
	bool sendBeacons = default(true); // tell the applayer to periodically send beacons
	bool dataOnSch = default(true); //tells the applayer whether to use a service channel for datapackets or the control channel

	int beaconLengthBits = default(416bit) @unit(bit); // the length of a beacon packet
	int beaconPriority = default(0); // the access category of the beacon messages

	double maxStoreTime = default(5s) @unit(s); // the maximum time to store routing message from others
	double guidUsedTime = default(10s) @unit(s); // the maximum time from a GUID's allocated time to its recycled time

	double beaconInterval = default(1s) @unit(s); // the intervall between 2 beacon messages
	double examineNeighborsInterval = default(1.5s) @unit(s); // the intervall between 2 examine neighbors messages
	double forgetMemoryInterval = default(15s) @unit(s); // the intervall between 2 forget the memory of received packets messages
	double neighborElapsed = default(2s) @unit(s); // if exceeds this time haven't receive beacon message from neighbors, regard it turning into disconnected
	double memoryElapsed = default(15s) @unit(s); // if exceeds this time, the memory of received packet will be forgotten

	gates:
		input cellularIn @directIn; // cellularIn gate for sendDirect, receive packets from cellular base station
}
