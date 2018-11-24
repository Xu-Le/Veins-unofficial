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

class noncobject HopItems;

packet WarningMessage extends WaveShortMessage
{
	bool warningSuccess = false; // Is warning message successfully arrive at farthest vehicle?
	bool direction = false; // Is target to opposite vehicle?
	int GUID = 0; // Unique identifier of this WarningMessage
	int hopCount = 0;   // Total hop count from sender
	double farthestDistance = 1000; // The farthest target vehicle for warning message in ROI
	string laneId; // Indicates which lane is the warning message's target
	HopItems hopInfo;   // Info about every hop vehicle
}
