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

#ifndef __UAV_H__
#define __UAV_H__

#include "utils.h"

class UAV;
class Solution;

class EventInfo
{
public:
	EventInfo(int _indicator, int _priority, void *_data) : indicator(_indicator), priority(_priority), data(_data) {}

	EventInfo(const EventInfo&) = default;
	EventInfo& operator=(const EventInfo&) = default;

	friend bool operator==(const EventInfo& lhs, const EventInfo& rhs);
	friend bool operator<(const EventInfo& lhs, const EventInfo& rhs);

	int indicator;
	int priority;
	void *data;
};

class RSU
{
public:
	enum EventType {
		CHECKCONN,
		EMITUAV,
		ATTAINDENSITY,
		NOTIFYSEND
	};

	explicit RSU(Solution *_env);
	~RSU();

	double getX() { return pos.x; }
	double getY() { return pos.y; }
	const Point& getPos() { return pos; }
	int getSelfIndex() { return selfIndex; }
	double getAverageDensity() { return averageDensity; }
	double getDensityDivision() { return densityDivision; }
	void setX(double x) { pos.x = x; }
	void setY(double y) { pos.y = y; }
	void setSelfIndex(int n) { selfIndex = n; }
	void setRemainingNum(int n) { remainingNum = n; }

	void handleEvent(int what, void *data);

private:
	void checkConn();
	void emitUAV();
	void attainDensity();
	void notifySend(int sourceIndex);

	UAV* getNeighbor(int index);

private:
	Solution *env;
	Point pos; ///< 3D position.
	int selfIndex;
	int remainingNum;
	double averageDensity;
	double densityDivision;
	std::list<int> vehicles;
	std::list<int>::iterator itV;
	std::list<UAV*> neighbors;
	std::list<UAV*>::iterator itN;
	std::map<int, int> routingTable;
	std::map<int, int>::iterator itRT;

public:
	static double r;
	static double R;
};

class UAV
{
	friend class RSU;

public:
	enum EventType {
		CHECKCONN,
		FLYING,
		DECIDE,
		NOTIFYSEND,
		NOTIFYRECV
	};

	UAV(Solution *_env, Point& _pos, int _selfIndex);
	~UAV();

	double getX() { return pos.x; }
	double getY() { return pos.y; }
	const Point& getPos() { return pos; }
	int getSelfIndex() { return selfIndex; }

	void handleEvent(int what, void *data);

private:
	void checkConn();
	void flying();
	void decide();
	void notifySend(int sourceIndex);
	void notifyRecv(int sourceIndex);

	void attainSectorDensity();
	int attainForbiddenSector();
	int getNextHop();
	UAV* getNeighbor(int index);
	double maxRelativeMoving();

private:
	Solution *env;
	RSU *rsu;
	Point pos; ///< 3D position.
	Point dir;
	int hop;
	int selfIndex;
	int decideAt;
	int flyingTime;
	double hopDist;
	double averageDensity;
	double densityDivision;
	std::list<int> vehicles;
	std::list<int>::iterator itV;
	std::list<UAV*> neighbors;
	std::list<UAV*>::iterator itN;
	std::map<int, int> routingTable;
	std::map<int, int>::iterator itRT;
	std::vector<double> sectorDensity;
	std::vector<int> forbidden;

public:
	static double r;
	static double R;
	static double v;
	static double minGap;
	static double radTheta;
	static int theta;
	static int sectorNum;
};

class ReadConfig
{
public:
	ReadConfig(const char *filename);
	~ReadConfig();

	void parse(std::string& key, std::string& value);
	bool hasNext();

private:
	FILE *fd;
	char buf[100];
};

/** configure static members of class UAV and RSU. */
void configEnv();

#endif /* __UAV_H__ */
