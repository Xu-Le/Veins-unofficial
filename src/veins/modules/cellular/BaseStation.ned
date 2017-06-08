//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
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

package org.car2x.veins.modules.cellular;

simple BaseStation
{
	parameters:
    	@class(BaseStation);
    	@display("i=device/antennatower_l");
    	int wiredHeaderLength = 224bit @unit(bit); // header length of the wired UDP/IP packet
    	int wirelessHeaderLength = default(160bit) @unit(bit); // header length of the cellular packet

    gates:
        input wirelessIn @directIn; // wirelessIn gate for sendDirect, received from vehicles
        input wiredIn;   // wiredIn gate connected to file content server
        output wiredOut; // wiredOut gate connected to file content server
}
