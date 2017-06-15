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

int ContentUtils::rateTable[25] = { 5600, 5500, 5500, 5400, 5300,
5300, 5200, 5200, 5200, 5200,
5100, 5100, 5000, 4700, 4100,
3400, 2600, 1600, 700, 400,
400, 300, 300, 200, 200 };

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
