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
enum CellularMsgCC {
	TRANSMISSION_STARTING, ///< request to start transmission;
	TRANSMISSION_ENDING,   ///< request to end transmission;
	DATA_PACKET_NORMAL,    ///< normal data packet transmission;
	DATA_PACKET_LAST,      ///< last data packet transmission;
	DOWNLOADING_COMPLETING ///< downloading process completion notification
};
}}

packet CellularMessage
{
	int controlCode; // express which kind of control signaling message, see enum CellularMsgCC
	long downloader; // identifier of downloader's application
	int contentSize; // content size of requested large volume file
	int curOffset;   // current content offset of the data this packet contains
	int startOffset; // start offset position of this content
	int endOffset;   // end offset position of this content
}
