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

cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

packet RoutingMessage extends WaveShortMessage
{
	@descriptor(false);
	bool routingSuccess = false; // Is routing path successfully found?
	bool backward = false; // Is routing result known, notify it backward to sender?
	int GUID = 0; // Unique identifier of this RoutingMessage
	int hopCount = 0;   // Total hop count from sender
	long destination = -1; // Destination vehicle
	long nextHop = -1;  // Next selected hop vehicle in routing process
}
