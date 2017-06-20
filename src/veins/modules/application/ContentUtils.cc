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

#include "veins/modules/application/ContentUtils.h"

int ContentUtils::rateTable[50] = { 5600, 5600, 5500, 5500, 5500, 5500, 5400, 5400, 5300, 5300,
5300, 5300, 5200, 5200, 5200, 5200, 5200, 5200, 5200, 5200,
5100, 5100, 5100, 5100, 5000, 4900, 4700, 4400, 4100, 3750,
3400, 3000, 2600, 2100, 1600, 1150, 700, 550, 400, 400,
350, 350, 300, 300, 300, 250, 250, 250, 200, 200 };

Segment::Segment() : begin(-1), end(-1), next(NULL)
{

}

Segment::~Segment()
{
	if (next != NULL) // delete recursively
		delete next;
}

void Segment::assign(const Segment *rhs)
{
	begin = rhs->begin;
	end = rhs->end;
	if (rhs->next != NULL) // assign recursively
	{
		next = new struct Segment;
		next->assign(rhs->next);
	}
	else // reach the last segment
		next = NULL;
}

void Segment::print()
{
	EV << '(' << begin << '-' << end << ") ";
	if (next != NULL) // print recursively
		next->print();
	else // reach the last segment
		EV << "\n";
}

int ContentUtils::calcLinkBytes(int applBytes, int headerLen, int dataLen)
{
	int completePacketNum = applBytes / dataLen;
	int lastPacketLength = 0;
	if ((lastPacketLength = applBytes % dataLen) != 0)
		lastPacketLength += headerLen; // plus header length
	return completePacketNum * (headerLen + dataLen) + lastPacketLength;
}
