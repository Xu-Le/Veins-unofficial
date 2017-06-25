//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

import org.car2x.veins.base.modules.BaseApplLayer;

simple BaseRSU extends BaseApplLayer
{
	@class(BaseRSU);
	@display("i=block/process");

	double positionX;
	double positionY;
	double positionZ;

	bool sendWarnings = default(true);  // whether this RSU is at work
	bool helpRoutings = default(false); // whether this RSU will help vehicle rebroadcast routing messages.
	bool sendContents = default(false); // whether this RSU should send data
	bool dataOnSch = default(true); //tells the applayer whether to use a service channel for datapackets or the control channel

	int wiredHeaderLength = 224bit @unit(bit); // header length of the wired IP and UDP packet
	int warningLengthBits = default(960bit) @unit(bit); // the length of a warning packet
	int warningPriority = default(0); // the access category of the warning messages
	int contentLengthBits = default(768bit) @unit(bit); // the length of a content packet
	int contentPriority = default(3); // the access category of the content messages
	int dataLengthBits = default(18272bit) @unit(bit); //the length of a data packet 8 * (2312-20-8)
	int dataPriority = default(3); //the access category for data packets
	int maxHopConstraint = default(5); // the maximum of routing message hop count constraint
	int whichSide = default(0); // which side direction relative to road this RSU locate

	double examineVehiclesInterval = default(2s) @unit(s); // the intervall between 2 examine vehicle whether in communication range messages
	double forgetMemoryInterval = default(15s) @unit(s);   // the intervall between 2 forget the memory of received packets messages
	double vehicleElapsed = default(3s) @unit(s); // if exceeds this time, assuming has lose connectivity with the vehicle
	double memoryElapsed = default(15s) @unit(s); // if exceeds this time, the memory of received packet will be forgotten

	gates:
		input wiredIn;   // wiredIn gate connected to file content server
		output wiredOut; // wiredOut gate connected to file content server
		input westIn;    // westIn gate connected to west neighbor RSU
		output westOut;  // westOut gate connected to west neighbor RSU
		input eastIn;    // eastIn gate connected to east neighbor RSU
		output eastOut;  // eastOut gate connected to east neighbor RSU
}
