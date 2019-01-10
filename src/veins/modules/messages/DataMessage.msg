//
// Copyright (C) 2017-2018 Xu Le <xmutongxinXuLe@163.com>
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

packet DataMessage extends WaveShortMessage
{
	@descriptor(readonly);
	bool isAck;    // if isAck is true, then sequence field indicates which sequence receives acknowledgment
	long receiverAddress; // L3 Address
	long source;   // identifier of source's application
	long destination; // identifier of destination's application
	int sequence;  // current content offset of the data this packet contains
	int bytesNum;  // this variable exists due to performance consideration, for sending many packets once in transmission
	int altitude;  // hop count from current receiving node to the source, used by dynamic path shortening
}
