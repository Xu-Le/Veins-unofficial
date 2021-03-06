//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
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

#include "veins/modules/utility/Utils.h"

#if RAND_MAX == 0x7fff
bool WaveUtils::GUIDPool[32767] = { 0 };
#else
bool WaveUtils::GUIDPool[65535] = { 0 };
#endif

double WaveUtils::_length(Coord& point1, Coord& point2)
{
	return sqrt(square(point1.x - point2.x) + square(point1.y - point2.y));
}

double WaveUtils::dotProduct(Coord& point1, Coord& point2)
{
	return point1.x * point2.x + point1.y * point2.y;
}

double WaveUtils::crossProduct(Coord& point1, Coord& point2)
{
	return point1.x * point2.y - point1.y * point2.x;
}

double WaveUtils::distanceToLine(Coord& P, Coord& A, Coord& B)
{
	return fabs((B.y - A.y) * P.x - (B.x - A.x) * P.y - crossProduct(A, B)) / _length(A, B);
}

double WaveUtils::distanceToLine(Coord& P, Coord& A, Coord& B, Coord& D, double& _AD)
{
	Vector AP(P.x - A.x, P.y - A.y);
	Vector AB(B.x - A.x, B.y - A.y);
	double _AB2 = square(AB.x) + square(AB.y);
	double product = dotProduct(AP, AB);
	Vector AD(AB.x * product / _AB2, AB.y * product / _AB2); // project vector AP onto vector AB
	D.x = A.x + AD.x;
	D.y = A.y + AD.y;
	if (product > 0)
		_AD = sqrt(square(AD.x) + square(AD.y));
	else if (product < 0)
		_AD = -sqrt(square(AD.x) + square(AD.y));
	else
		_AD = 0.0;
	return _length(P, D);
}

short WaveUtils::relativeDirection(double ownDir, double otherDir)
{
	double diff = fabs(ownDir - otherDir);
	if ( diff > 0.75*M_PI && diff < 1.25*M_PI )
		return OPPOSITE;
	else if ( diff >= M_PI_4 && diff <= 0.75*M_PI )
		return ownDir > otherDir ? RIGHT : LEFT;
	else if ( diff >= 1.25*M_PI_4 && diff <= 1.75*M_PI )
		return ownDir > otherDir ? LEFT : RIGHT;
	else // (diff >= 0.0 && diff < M_PI_4) || (diff > 1.75*M_PI && diff < 2.0*M_PI)
		return SAME;
}

int WaveUtils::generateGUID()
{
	int high = rand(), low = rand();
#if RAND_MAX == 0x7fff
	while ( GUIDPool[high] ) // loop until rand() function produce a random number hasn't been used
		high = rand();
	GUIDPool[high] = true; // denote this random number is used by now
	while ( GUIDPool[low] ) // loop until rand() function produce a random number hasn't been used
		low = rand();
	GUIDPool[low] = true; // denote this random number is used by now
	return (high << 16) + low;
#else
    while ( GUIDPool[high >> 16] ) // loop until rand() function produce a random number hasn't been used
        high = rand();
    GUIDPool[high >> 16] = true; // denote this random number is used by now
    while ( GUIDPool[low & 0xffff] ) // loop until rand() function produce a random number hasn't been used
        low = rand();
    GUIDPool[low & 0xffff] = true; // denote this random number is used by now
    return (high & 0xffff0000) + (low & 0xffff);
#endif
}

void WaveUtils::recycleGUID(int GUID)
{
	int low = GUID & 0xffff, high = GUID >> 16;
	GUIDPool[low] = false;
	GUIDPool[high] = false;
}
