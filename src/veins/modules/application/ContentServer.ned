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

package org.car2x.veins.modules.application;

simple ContentServer
{
	parameters:
    	@class(ContentServer);
    	@display("i=device/server2");
    	int rsuNum; // number of RSUs in current scenario
    	int headerLength = 224bit @unit(bit); // header length of the wired UDP/IP packet

    gates:
        input cellularIn;   // cellularIn gate connected to cellular base station
        output cellularOut; // cellularOut gate connected to cellular base station
        input rsuIn[rsuNum]; // rsuIn gates connected to road side units
        output rsuOut[rsuNum]; // rsuOut gates connected to road side units
}
