//
// Copyright (C) 2018 Xu Le <xmutongxinXuLe@163.com>
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

package org.car2x.veins.examples.bologna.pasubio;

import org.car2x.veins.nodes.Scenario;

import org.car2x.veins.modules.uav.DeployStatisticCollector;
import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.UAV;

network scenario extends Scenario
{
	parameters:
	    @signal[optimalityCalculation](type="DeployStatisticCollector");
		int numRSU = 1; // number of RSUs
		int numUAV = 3; // number of UAVs

	submodules:
	    deployCollector: DeployStatisticCollector {
	        @display("p=700,24");
	    }
		rsu[numRSU]: RSU {
			@display("p=$posX,$posY;b=10,10,oval");
		}
		//uav[numUAV]: UAV {
		//	@display("p=$posX,$posY;b=10,10,oval");
		//}
}
