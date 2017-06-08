//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
enum ContentMsgCC {
    CONTENT_REQUEST,
    CONTENT_RESPONSE,
    SCHEME_DISTRIBUTION,
    ACKNOWLEDGEMENT,
    CARRIER_SELECTION,
    LINK_BREAK_DIRECT,
    LINK_BREAK_DR,
    LINK_BREAK_RR,
    RELAY_DISCOVERY,
    DISCOVERY_RESPONSE
};

class SchemeTuple
{
public:
	SchemeTuple() : slot(-1), receiver(-1), downloader(-1), amount(-1) {}
    SchemeTuple(int s, int r, int d, int a) : slot(s), receiver(r), downloader(d), amount(a) {}
    SchemeTuple(const SchemeTuple& rhs) : slot(rhs.slot), receiver(rhs.receiver), downloader(rhs.downloader), amount(rhs.amount) {}

	SchemeTuple& operator=(const SchemeTuple& rhs)
	{
	    if (this == &rhs)
	    	return *this;
		slot = rhs.slot;
		receiver = rhs.receiver;
		downloader = rhs.downloader;
		amount = rhs.amount;
		return *this;
	}

    int slot;
    int receiver;
    int downloader;
    int amount;
};
typedef std::list<std::pair<Coord /* pos */, Coord /* speed */> > NeighborItems;
typedef std::map<long /* addr */, std::list<SchemeTuple> > SchemeItems;
}}

class WaveShortMessage;
class noncobject NeighborItems;
class noncobject SchemeItems;

packet ContentMessage extends WaveShortMessage
{
    // @brief express which kind of control signaling message
    // 0: content request;
    // 1: RSU's response to content request;
    // 2: distribute current RSU's slot decision to relays;
    // 3: selected to be carrier by the cooperative RSU;
    // 4: link break notify - the communication link between downloader and RSU will break soon;
    // 5: link break notify - the communication link between downloader and relay will break soon;
    // 6: link break notify - the communication link between relay and RSU will break soon;
    // 7: cooperative RSU's discover notification sends to the first entering relay;
    // 8: the first entering relay response to cooperative RSU's discover notification.
    int controlCode;
    int downloader;  // which downloader this content message aims to
    int contentSize; // content size of request large-volume file
    int receivedOffset; // data amount received offset
    int consumedOffset; // data amount consumed offset
    int consumingRate;  // data amount consuming rate measured in Bytes each second
    NeighborItems neighborInfo; // neighbors information of current downloader (controlCode: 4,5,6)
    SchemeItems scheme; // transmitssion scheme each vehicle should obey (controlCode: 2)
}
