//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
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
class HopItem
{
public:
	HopItem() : addr(0), posX(0.0), posY(0.0), posZ(0.0), speedX(0.0), speedY(0.0), speedZ(0.0) {}
    HopItem(long a, double px, double py, double pz, double sx, double sy, double sz) : addr(a), posX(px), posY(py), posZ(pz), speedX(sx), speedY(sy), speedZ(sz) {}
    HopItem(const HopItem& rhs) : addr(rhs.addr), posX(rhs.posX), posY(rhs.posY), posZ(rhs.posZ), speedX(rhs.speedX), speedY(rhs.speedY), speedZ(rhs.speedZ) {}
    
	long addr;
	double posX;
	double posY;
	double posZ;
	double speedX;
	double speedY;
	double speedZ;
};
typedef std::list<HopItem> HopItems;
}}

class noncobject Coord;

packet WaveShortMessage {
	int wsmVersion = 1; // Version of the Wave Short Message
	int GUID = 0; // Unique identifier of this WaveShortMessage
	int securityType = 0; // Determine which security mechanism was used
	int channelNumber; // Channel Number on which this packet was sent
	int dataRate = 1; // Data rate with which this packet was sent
	int priority = 3; // Power Level with which this packet was sent
	int serial = 0;
	int psid = 0; // Unique number to identify the service
	int wsmLength; // Length of Wave Short Message
	long senderAddress = -1; // Equal to sender

	double senderAngle; // The driving angle of sender
	// string psc = "Service with some Data"; // Provider Service Context

	Coord senderPos;   // The current position of sender
	Coord senderSpeed; // The current speed of sender
}
