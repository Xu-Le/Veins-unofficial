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

// H. Zhao, H. Wang, W. Wu, and J. Wei, "Deployment algorithms for UAV airborne networks towards on-demand coverage,"
// IEEE J. Sel. Areas Commun., vol. 36, no. 7, pp. 1–16, Jul. 2018.
// SECTION V. DISTRIBUTED MOTION CONTROL ALGORITHM
#define USE_VIRTUAL_FORCE_MODEL

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
		NOTIFYSEND,
		CHECKTABLE
	};

	explicit RSU(Solution *_env);
	~RSU();

	double getX() { return pos.x; }
	double getY() { return pos.y; }
	const Point& getPos() { return pos; }
	int getSelfIndex() { return selfIndex; }
#ifndef USE_VIRTUAL_FORCE_MODEL
	double getAverageDensity() { return averageDensity; }
	double getDensityDivision() { return densityDivision; }
#endif
	void setX(double x) { pos.x = x; }
	void setY(double y) { pos.y = y; }
	void setSelfIndex(int n) { selfIndex = n; }
	void setRemainingNum(int n) { remainingNum = n; }

	void handleEvent(int what, void *data);

private:
	void checkConn();
	void emitUAV();
#ifndef USE_VIRTUAL_FORCE_MODEL
	void attainDensity();
#endif
	void notifySend(int sourceIndex);
	void checkTable();

	UAV* getNeighbor(int index);

private:
	Solution *env;
	Point pos; ///< 3D position.
	int selfIndex;
	int remainingNum;
	int routingTableIncorrectTimes;
#ifndef USE_VIRTUAL_FORCE_MODEL
	double averageDensity;
	double densityDivision;
#endif
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

	void handleEvent(int what, void *data);

private:
	void checkConn();
	void flying();
	void decide();
	void notifySend(int sourceIndex);
	void notifyRecv(int sourceIndex);
#ifndef USE_VIRTUAL_FORCE_MODEL
	void attainSectorDensity();
	int attainForbiddenSector();
#else
	void attainResultantForce(); // calculate dir
#endif
	bool isForbidden(Point dst, std::list<UAV*>& nbLess, std::list<UAV*>& nbEqual, std::list<UAV*>& nbGreater);
	int getNextHop(int index);
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
#ifndef USE_VIRTUAL_FORCE_MODEL
	double averageDensity;
	double densityDivision;
#endif
	std::list<int> vehicles;
	std::list<int>::iterator itV;
	std::list<UAV*> neighbors;
	std::list<UAV*>::iterator itN;
	std::map<int, int> routingTable;
	std::map<int, int>::iterator itRT;
#ifndef USE_VIRTUAL_FORCE_MODEL
	std::vector<double> sectorDensity;
	std::vector<int> forbidden;
#endif

public:
	static double r;
	static double R;
	static double v;
#ifndef USE_VIRTUAL_FORCE_MODEL
	static double minGap;
	static double radTheta;
	static int theta;
	static int expand;
	static int sectorNum;
#else
	static double k_a;
	static double K_r;
	static double R_opt;
#endif
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
