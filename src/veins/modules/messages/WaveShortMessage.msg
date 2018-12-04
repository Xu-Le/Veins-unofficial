//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>, 2018 Xu Le <xmutongxinXuLe@163.com>
//
// Documentation for these modules is at http://veins.car2x.org/
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
#include "veins/base/utils/Coord.h"
struct WAVEInformationElement
{
    WAVEInformationElement() : id(0), length(0) { value.uval = 0; }
    WAVEInformationElement(unsigned char _id, unsigned char _length, unsigned char _uval) : id(_id), length(_length) { value.uval = _uval; }

	unsigned char id;
	unsigned char length;
	union {
		unsigned char uval;
		char val;
	} value;
};
}}

class noncobject Coord;
class noncobject WAVEInformationElement;

packet WaveShortMessage {
	unsigned char wsmpVersion = 11; // Subtype, Option Indicator, WSMP Version, see "8.3.2.1 General" of IEEE Std 1609.3-2016
	unsigned char nHeaderExCount = 4; // WAVE Information Element Extension of WSMP-N-Header
	WAVEInformationElement channelNumber; // see 8.3.4.2 of IEEE Std 1609.3-2016
	WAVEInformationElement dataRate; // see 8.3.4.3 of IEEE Std 1609.3-2016
	WAVEInformationElement transmitPowerUsed; // see 8.3.4.4 of IEEE Std 1609.3-2016
	WAVEInformationElement channelLoad; // see 8.3.4.5 of IEEE Std 1609.3-2016
	unsigned char TPID; // Transport Protocol Identifier, see "8.3.3.2 Transport protocol features" of IEEE Std 1609.3-2016
	unsigned short PSID = 32775; // Provider Service Identifier, the PSID value for WSA 0p80-07, see "G.1 WSA example" of IEEE Std 1609.3-2016
	unsigned short wsmLength; // Length of Wave Short Message, see "8.1.3 Length and Count field encoding" of IEEE Std 1609.3-2016

	int priority = 3; // Power Level with which this packet was sent
	long senderAddress = -1; // L3 Address
	long recipientAddress = -1; // L2 Address
	Coord senderPos;   // The current position of sender
	Coord senderSpeed; // The current speed of sender
}
