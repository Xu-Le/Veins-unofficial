//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

package org.car2x.veins.nodes;

import org.car2x.veins.base.modules.BaseMobility;
import org.car2x.veins.modules.nic.Nic80211p;
import org.car2x.veins.modules.rsu.RoutingRSU;
import org.car2x.veins.modules.rsu.WarningRSU;
import org.car2x.veins.modules.rsu.ContentRSU;

module RSU
{
    parameters:
        double posX; // x size of the area the nodes are in (in meters)
        double posY; // y size of the area the nodes are in (in meters)
        double posZ; // z size of the area the nodes are in (in meters)
        string nicType = default("Nic80211p"); // type of network interface card
        
    gates:
        input veinswiredIn;   // transmit data to appl module
        output veinswiredOut; // receive data from appl module
        input veinswestIn;   // transmit data to appl module
        output veinswestOut; // receive data from appl module
        input veinseastIn;   // transmit data to appl module
        output veinseastOut; // receive data from appl module
        input veinsradioIn @directIn; // gate for sendDirect
        
    submodules:
        appl: ContentRSU {
            parameters:
                positionX = posX;
                positionY = posY;
                positionZ = posZ;
                @display("p=60,50");
        }

        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=60,166");
        }

        mobility: BaseMobility {
            parameters:
                @display("p=130,166;i=block/cogwheel");
                x = posX;
                y = posY;
                z = posZ;
        }
        
    connections:
        nic.upperLayerOut --> appl.lowerLayerIn;
        nic.upperLayerIn <-- appl.lowerLayerOut;
        nic.upperControlOut --> appl.lowerControlIn;
        nic.upperControlIn <-- appl.lowerControlOut;

		veinswiredIn --> appl.wiredIn;
		veinswiredOut <-- appl.wiredOut;
		veinswestIn --> appl.westIn;
		veinswestOut <-- appl.westOut;
		veinseastIn --> appl.eastIn;
		veinseastOut <-- appl.eastOut;
        veinsradioIn --> nic.radioIn;
}
