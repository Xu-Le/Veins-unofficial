//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

package org.car2x.veins.tests.highway_notls;

import org.car2x.veins.nodes.Scenario;

import org.car2x.veins.modules.cellular.BaseStation;
import org.car2x.veins.modules.application.BaseServer;
import org.car2x.veins.nodes.RSU;

network scenario extends Scenario
{
	parameters:
		int numRSU = 7; // number of RSUs

	submodules:
		BS: BaseStation {
			@display("p=4500,24");
		}
		server: BaseServer {
			parameters:
				rsuNum = numRSU;
			@display("p=5000,24");
		}
		rsu[numRSU]: RSU {
			@display("p=$posX,$posY;b=10,10,oval");
		}

	connections:
		BS.wiredIn <-- ned.DatarateChannel{datarate=100Mbps;} <-- server.cellularOut;
		BS.wiredOut --> ned.DatarateChannel{datarate=100Mbps;} --> server.cellularIn;
		for i = 0..numRSU-1 {
			rsu[i].veinswiredIn <-- ned.DatarateChannel{datarate=100Mbps;} <-- server.rsuOut[i];
			rsu[i].veinswiredOut --> ned.DatarateChannel{datarate=100Mbps;} --> server.rsuIn[i];
		}
		for i = 0..numRSU-2 {
			rsu[i].veinseastIn <-- ned.DatarateChannel{datarate=100Mbps;} <-- rsu[i+1].veinswestOut;
			rsu[i].veinseastOut --> ned.DatarateChannel{datarate=100Mbps;} --> rsu[i+1].veinswestIn;
		}
}
