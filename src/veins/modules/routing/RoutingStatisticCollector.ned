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

package org.car2x.veins.modules.routing;

simple RoutingStatisticCollector
{
	@class(RoutingStatisticCollector);
	@display("i=abstract/db");

	int trafficDensity = default(720vph) @unit(vph);
	int roadSpeedLimit = default(60km) @unit(km);
	int rtExpirationTime = default(6s) @unit(s);
	int packetsPerSecond = default(50);
	string resultFile; // csv file storing results
}
