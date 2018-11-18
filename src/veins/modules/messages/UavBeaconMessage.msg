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

cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

packet UavBeaconMessage extends WaveShortMessage
{
	int hop; // minimum hop count to the RSU
	simtime_t decideAt; // time instant when making the last deployment decision
	double hopDist; // distance to the next hop
	double averageDensity; // average density of all sectors
	double densityDivision = 1.0; // maximum sector density division by average sector density
}
