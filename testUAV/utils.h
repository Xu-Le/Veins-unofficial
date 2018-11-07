//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <utility>
#include <vector>
#include <list>
#include <stack>
#include <queue>
#include <set>
#include <map>
#include <algorithm>

#include <QDebug>

class Point
{
public:
	Point() : x(0.0), y(0.0), z(0.0) {}
	Point(double X, double Y) : x(X), y(Y), z(0.0) {}

	double x;
	double y;
	double z;

	static Point zero;
};

namespace math {

bool equal0(double n);

double square(double n);

double dist(const Point& A, const Point& B);

double dist2(const Point& A, const Point& B);

double dist3D(const Point& A, const Point& B);

double dotProduct(const Point& A, const Point& B);

double crossProduct(const Point& A, const Point& B);

}

std::string itoa(int n);

int parseInput(const char *filename);

#endif /* __UTILS_H__ */
