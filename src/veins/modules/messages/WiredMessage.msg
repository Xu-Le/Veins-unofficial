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
enum WiredMsgCC {
    START_TRANSMISSION,
    END_TRANSMISSION,
    NORMAL_DATA_PACKET,
    LAST_DATA_PACKET
};
}}

packet WiredMessage
{
    // @brief express which kind of control signaling message.
    // 0: request to start transmission;
    // 1: request to end transmission;
    // 2: normal data packet transmission;
    // 3: last data packet transmission;
    int controlCode;
    int downloader;  // identifier of downloader's application
    int contentSize; // content size of requested large volume file
    int curOffset;   // current content offset of the data this packet contains
	int startOffset; // start offset position of this content
    int endOffset;   // end offset position of this content
}
