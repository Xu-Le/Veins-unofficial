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
	CONTENT_REQUEST,       ///< content request;
	CONTENT_RESPONSE,      ///< RSU's response to content request;
	SCHEME_DISTRIBUTION,   ///< distribute current RSU's slot decision to relays;
	ACKNOWLEDGEMENT,       ///< acknowledgment received offset to RSU and relay at the end of each slot;
	CARRIER_SELECTION,     ///< selected to be carrier by the cooperative RSU;
	CARRIER_ENCOUNTER,     ///< carrier encounter the downloader its carried data belongs to;
	DOWNLOADING_COMPLETED, ///< downloading process completion notification broadcast by downloader;
	LINK_BREAK_DIRECT,     ///< link break notify - the communication link between downloader and RSU will break soon;
	LINK_BREAK_DR,         ///< link break notify - the communication link between downloader and relay will break soon;
	LINK_BREAK_RR,         ///< link break notify - the communication link between relay and RSU will break soon;
	LINK_BREAK_ACK,        ///< acknowledgment to link break notification;
	LINK_BROKEN_DR,        ///< link broken notify - the communication link between downloader and relay has broken already;
	DOWNLOADER_DISCOVERY,  ///< cooperative RSU's discover notification sends to the entering co-downloader;
	DISCOVERY_RESPONSE,    ///< the response to cooperative RSU's discover notification;
	STATUS_REPORT,         ///< report downloading status to RSU (used only in multi-downloader scenarios);
	REBROADCAST_BEACON     ///< rebroadcast co-downloader's beacon message to ensure co-RSU knows the latest position of the co-downloader.
};

class SchemeTuple
{
public:
	SchemeTuple() : slot(-1), receiver(-1), downloader(-1), amount(-1), _offset(), offset(&_offset) {}
	SchemeTuple(int s, int r, int d, int a) : slot(s), receiver(r), downloader(d), amount(a), _offset(), offset(&_offset) {}
	SchemeTuple(const SchemeTuple& rhs) : slot(rhs.slot), receiver(rhs.receiver), downloader(rhs.downloader), amount(rhs.amount)
	{
		_offset = rhs._offset;
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
		_offset = rhs._offset;
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
class noncobject Segment;
class noncobject DownloaderItems;
class noncobject SchemeItems;

packet ContentMessage extends WaveShortMessage
{
	int controlCode; // express which kind of control signaling message, see enum ContentMsgCC.
	long receiver;    // identifier of receiver's application
	long downloader;  // which downloader this content message aims to
	int contentSize; // content size of request large-volume file (controlCode: 0)
	int receivedOffset; // data amount received offset (controlCode: 2,7,8,9)
	int consumedOffset; // data amount consumed offset (controlCode: 2,7,8,9)
	int consumingRate;  // data amount consuming rate measured in Bytes each second (controlCode: 0)
	simtime_t reportAt; // RSU tells downloader when to report its downloading status (controlCode: 1)
	Segment lackOffset; // lacking data segment's offset (controlCode: 3)
	SchemeItems scheme; // transmission scheme each vehicle should obey (controlCode: 2)
	Coord position;  // position of the co-downloader (controlCode: 7,8,9)
	Coord speed;     // speed of the co-downloader (controlCode: 7,8,9)
	DownloaderItems downloaders; // downloader list in transmission scheme distribution message (controlCode: 2)
}
