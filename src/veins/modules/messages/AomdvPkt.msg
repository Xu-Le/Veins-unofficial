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
#include "veins/modules/messages/RoutingMessage_m.h"

enum AOMDVPacketType
{
	RREQ = 1,
	RREP = 2,
	RERR = 3,
	RREPACK = 4
};
// Helper struct to represent (L3Address, sequence number) pairs in a RERR message
struct UnreachableNode
{
	long addr;
	unsigned int seqno;
};
}}

class RoutingMessage;
class noncobject UnreachableNode;

// Represents an AOMDV Route Request
packet RREQMessage extends RoutingMessage
{
	bool joinFlag;
	bool repairFlag;
	bool gratuitousRREPFlag;
	bool destOnlyFlag;
	// bool unknownSeqnoFlag;
	unsigned char hopCount;
	unsigned int rreqId;
	long destAddr;
	unsigned int destSeqno;
	long originatorAddr;
	unsigned int originatorSeqno;
	long firstHop;
}

// Represents an AOMDV Route Reply
packet RREPMessage extends RoutingMessage
{
	bool repairFlag;
	bool ackRequiredFlag;
	unsigned char prefixSize;
	unsigned char hopCount;
	long destAddr;
	unsigned int destSeqno;
	long originatorAddr;
	simtime_t lifeTime;
	unsigned int rreqId;
	long firstHop;
}

// Represents an AOMDV Route Error
packet RERRMessage extends RoutingMessage
{
	// bool noDeleteFlag;
	unsigned char destCount;
	UnreachableNode unreachableNodes[];
}

// Represents an AOMDV Route Reply ACK
// packet RREPACKMessage extends RoutingMessage {}

// Represents a timer for a Route Reply packet
message WaitForRREPMessage
{
    long destAddr;
    unsigned char lastTTL;
}
