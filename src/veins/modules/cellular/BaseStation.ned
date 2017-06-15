//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
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

package org.car2x.veins.modules.cellular;

simple BaseStation
{
	parameters:
		@class(BaseStation);
		@display("i=device/antennatower_l");
		int wiredHeaderLength = 224bit @unit(bit); // header length of the wired UDP/IP packet
		int wirelessHeaderLength = default(160bit) @unit(bit); // header length of the cellular packet
		int wirelessBitsRate = default(100000000bps) @unit(bps); // data transmission rate of TD-LTE radio
		int wirelessDataLength = default(18272bit) @unit(bit); // the length of a data packet 8*(2312-20-8)

	gates:
		input wirelessIn @directIn; // wirelessIn gate for sendDirect, received from vehicles
		input wiredIn;   // wiredIn gate connected to file content server
		output wiredOut; // wiredOut gate connected to file content server
}
