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

#include "utils.h"

extern double gX;
extern double gY;
extern int numVehicle;
extern Point *vehicleTable;
extern double *velocityTable;
extern double *directionTable;
extern int *coveredTable;

Point Point::zero;

namespace math {

bool equal0(double n)
{
	return n > -1e-8 && n < 1e-8;
}

double square(double n)
{
	return n * n;
}

double dist(const Point& A, const Point& B)
{
	return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

double dist2(const Point& A, const Point& B)
{
	return (A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y);
}

double dist3D(const Point& A, const Point& B)
{
	return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y) + (A.z - B.z)*(A.z - B.z));
}

double dotProduct(const Point& A, const Point& B)
{
	return A.x * B.x + B.x * B.y;
}

double crossProduct(const Point& A, const Point& B)
{
	return A.x * B.y - B.x * A.y;
}

}

std::string itoa(int n)
{
	static char digits[19] = { '9', '8', '7', '6', '5', '4', '3', '2', '1', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	static char *zero = digits + 9;
	std::string s;
	bool negative = n < 0;
	do {
		s.push_back(zero[n%10]);
		n /= 10;
	}
	while (n != 0);
	if (negative)
		s.push_back('-');
	std::reverse(s.begin(), s.end());
	return s;
}

int parseInput(const char *filename)
{
	FILE *fd = fopen(filename, "r");
	if (fd == NULL)
	{
		qFatal("Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	int _x = 0, _y = 0, numAvailableUAV = 0;
	fscanf(fd, "%d,%d,%d,%d\n", &_x, &_y, &numVehicle, &numAvailableUAV);
	gX = _x, gY = _y;

	vehicleTable = new Point[numVehicle];
	velocityTable = new double[numVehicle];
	directionTable = new double[numVehicle];
	coveredTable = new int[numVehicle];
	int theta = 0;
	for (int i = 0; i < numVehicle; ++i)
	{
		fscanf(fd, "%lf,%lf,%lf,%d\n", &vehicleTable[i].x, &vehicleTable[i].y, &velocityTable[i], &theta);
		directionTable[i] = M_PI * theta / 180.0;
		coveredTable[i] = 0;
	}

	fclose(fd);

	return numAvailableUAV;
}
