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
#include "veins/modules/application/ContentUtils.h"

enum ContentMsgCC {
	CONTENT_REQUEST,
	CONTENT_RESPONSE,
	SCHEME_DISTRIBUTION,
	ACKNOWLEDGEMENT,
	CARRIER_SELECTION,
	CARRIER_ENCOUNTER,
	DOWNLOADING_COMPLETED,
	LINK_BREAK_DIRECT,
	LINK_BREAK_DR,
	LINK_BREAK_RR,
	RELAY_DISCOVERY,
	DISCOVERY_RESPONSE,
	STATUS_QUERY
};

class SchemeTuple
{
public:
	SchemeTuple() : slot(-1), receiver(-1), downloader(-1), amount(-1), _offset(), offset(&_offset) {}
	SchemeTuple(int s, int r, int d, int a) : slot(s), receiver(r), downloader(d), amount(a), _offset(), offset(&_offset) {}
	SchemeTuple(const SchemeTuple& rhs) : slot(rhs.slot), receiver(rhs.receiver), downloader(rhs.downloader), amount(rhs.amount)
	{
		_offset.assign(&rhs._offset);
		offset = &_offset;
	}

	SchemeTuple& operator=(const SchemeTuple& rhs)
	{
		if (this == &rhs)
			return *this;
		slot = rhs.slot;
		receiver = rhs.receiver;
		downloader = rhs.downloader;
		amount = rhs.amount;
		_offset.assign(&rhs._offset);
		offset = &_offset;
		return *this;
	}

	int slot;
	int receiver;
	int downloader;
	int amount;
	Segment _offset; ///< internal variable, head node of segment list, thus the whole list can be cleared when its destructor automatically called.
	Segment *offset; ///< external variable, use offset = offset->next to iterate the segment list.
};

typedef std::map<long /* addr */, std::list<SchemeTuple> > SchemeItems;
}}

class WaveShortMessage;
class noncobject Coord;
class noncobject NeighborItems;
class noncobject SchemeItems;

packet ContentMessage extends WaveShortMessage
{
	// @brief express which kind of control signaling message
	// 0: content request;
	// 1: RSU's response to content request;
	// 2: distribute current RSU's slot decision to relays;
	// 3: acknowledgement received offset to RSU and relay at the end of each slot;
	// 4: selected to be carrier by the cooperative RSU;
	// 5: carrier encounter the downloader its carried data belongs to;
	// 6: downloading precess completion notification broadcast by downloader;
	// 7: link break notify - the communication link between downloader and RSU will break soon;
	// 8: link break notify - the communication link between downloader and relay will break soon;
	// 9: link break notify - the communication link between relay and RSU will break soon;
	// 10: cooperative RSU's discover notification sends to the first entering relay;
	// 11: the first entering relay response to cooperative RSU's discover notification,
	// 12: query downloading status of downloader.
	int controlCode;
	int receiver;    // identifier of receiver's application
	int downloader;  // which downloader this content message aims to
	int contentSize; // content size of request large-volume file
	int receivedOffset; // data amount received offset
	int consumedOffset; // data amount consumed offset
	int consumingRate;  // data amount consuming rate measured in Bytes each second
	SchemeItems scheme; // transmitssion scheme each vehicle should obey (controlCode: 2)
	Coord position;  // position of the co-downloader
	Coord speed;     // speed of the co-downloader
	NeighborItems neighbors; // neighbors of the co-downloader
}
