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
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

packet WaveServiceAdvertisment extends WaveShortMessage
{
	@descriptor(false);
	unsigned char wsaOptionIndicator = 56; // WSA Version, WSA Header Option Indicator, see 8.2.2.2 of IEEE Std 1609.3-2016
	unsigned char wsaIdentifier; // WSA Identifier, Content Count, see 8.2.2.4 of IEEE Std 1609.3-2016
}
