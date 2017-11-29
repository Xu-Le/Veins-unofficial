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

#ifndef __COORD_H__
#define __COORD_H__

#include <math.h>

#define square(x)  (x) * (x)

class Coord
{
public:
	Coord() : x(0.0), y(0.0), z(0.0) {}
	Coord(double _x, double _y, double _z = 0.0) : x(_x), y(_y), z(_z) {}
	Coord(const Coord& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) {}

	Coord& operator=(const Coord& rhs) { x = rhs.x; y = rhs.y; z= rhs.z; return *this; }

	/** @brief Adds two coordinate vectors. */
	friend Coord operator+(const Coord& a, const Coord& b)
	{
		Coord tmp(a);
		tmp += b;
		return tmp;
	}

	/** @brief Subtracts two coordinate vectors. */
	friend Coord operator-(const Coord& a, const Coord& b)
	{
		Coord tmp(a);
		tmp -= b;
		return tmp;
	}

	/** @brief Multiplies a coordinate vector by a real number. */
	friend Coord operator*(const Coord& a, double f)
	{
		Coord tmp(a);
		tmp *= f;
		return tmp;
	}

	/** @brief Divides a coordinate vector by a real number. */
	friend Coord operator/(const Coord& a, double f)
	{
		Coord tmp(a);
		tmp /= f;
		return tmp;
	}

	Coord& operator+=(const Coord& a)
	{
		x += a.x;
		y += a.y;
		z += a.z;
		return *this;
	}

	Coord& operator-=(const Coord& a)
	{
		x -= a.x;
		y -= a.y;
		z -= a.z;
		return *this;
	}

	Coord& operator*=(double f)
	{
		x *= f;
		y *= f;
		z *= f;
		return *this;
	}

	Coord& operator/=(double f)
	{
		x /= f;
		y /= f;
		z /= f;
		return *this;
	}

	static double length(Coord& point1, Coord& point2)
	{
		return sqrt(square(point1.x - point2.x) + square(point1.y - point2.y));
	}

	double x;
	double y;
	double z;

	/** @brief Constant with all values set to 0. */
	static const Coord ZERO;
};

#endif /* __COORD_H__ */

