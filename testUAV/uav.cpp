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

#include "deploy.h"

#define CHECKCONN_INTERVAL   10
#define FLYING_INTERVAL      10
#define DECIDE_INTERVAL     100
#define EMITUAV_INTERVAL   1000
#define ATTAIN_INTERVAL     100

#define CHECKCONN_PRIORITY    0
#define FLYING_PRIORITY       4
#define DECIDE_PRIORITY       2
#define ATTAIN_PRIORITY       1
#define NOTIFY_PRIORITY       3

double RSU::r = 0.0;
double RSU::R = 0.0;
double UAV::r = 0.0;
double UAV::R = 0.0;
double UAV::v = 5.0;
double UAV::minGap = 100.0;
double UAV::radTheta = 0.0;
int UAV::theta = 30;
int UAV::sectorNum = 12;

extern double gX;
extern double gY;
extern int numVehicle;
extern Point *vehicleTable;
extern double *velocityTable;
extern double *directionTable;
extern int *coveredTable;

bool operator==(const EventInfo& lhs, const EventInfo& rhs)
{
	return lhs.priority == rhs.priority && lhs.indicator == rhs.indicator;
}

bool operator<(const EventInfo& lhs, const EventInfo& rhs)
{
	return lhs.priority < rhs.priority || (lhs.priority == rhs.priority && lhs.indicator < rhs.indicator);
}

//////////////////////////////    RSU    //////////////////////////////
RSU::RSU(Solution *_env) : env(_env), pos(), selfIndex(0), remainingNum(0)
{

}

RSU::~RSU()
{
	vehicles.clear();
	neighbors.clear();
	routingTable.clear();
}

void RSU::handleEvent(int what, void *data)
{
	int *idata = NULL;
	switch (what)
	{
	case EventType::CHECKCONN:
		checkConn();
		break;
	case EventType::EMITUAV:
		emitUAV();
		break;
	case EventType::ATTAINDENSITY:
		attainDensity();
		break;
	case EventType::NOTIFYSEND:
		idata = static_cast<int*>(data);
		notifySend(*idata);
		break;
	default:
		qCritical() << "Unknown EventType for class RSU.";
	}
	delete idata;
}

void RSU::checkConn()
{
	env->attainVehiclesNearby(pos, vehicles);
	env->attainNeighbors(pos, selfIndex, neighbors);
	env->schedule(CHECKCONN_INTERVAL, selfIndex, RSU::EventType::CHECKCONN, CHECKCONN_PRIORITY);
	qDebug().nospace() << "RSU check connection at " << env->eventSequence << ", vn: " << vehicles.size() << ", nn: " << neighbors.size();
}

void RSU::emitUAV()
{
	int who = static_cast<int>(env->uavs.size());
	env->uavs.emplace_back(env, pos, who);
	env->schedule(0, who, UAV::EventType::CHECKCONN, CHECKCONN_PRIORITY);
	// ensure that UAV::checkConn() has been invoked before invoking UAV::decide()
	env->schedule(0, who, UAV::EventType::DECIDE, DECIDE_PRIORITY);
	if (--remainingNum > 0)
		env->schedule(EMITUAV_INTERVAL, selfIndex, RSU::EventType::EMITUAV, 9);
	qDebug().nospace() << "RSU emit UAV " << who << " at " << env->eventSequence;
}

void RSU::attainDensity()
{
	env->schedule(ATTAIN_INTERVAL, selfIndex, EventType::ATTAINDENSITY, ATTAIN_PRIORITY);

	const double closeMulitplier = 1.0;
	std::vector<double> sectorDensity(UAV::sectorNum, 0.0);
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		double angle = acos((vehicleTable[*itV].x - pos.x) / math::dist(pos, vehicleTable[*itV]));
		if (vehicleTable[*itV].y < pos.y)
			angle = 2*M_PI - angle;
		int sector = angle / UAV::radTheta;
		double coverCoefficient = 1.0, closeCoefficient = 0.0;
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
			if ((closeCoefficient = math::dist((*itN)->getPos(), vehicleTable[*itV])/UAV::r) < 1.0)
				coverCoefficient += 1.0 + closeMulitplier*(1.0 - closeCoefficient);
		sectorDensity[sector] += 1.0/coverCoefficient;
	}
	QDebug qDebug(QtMsgType::QtDebugMsg);
	qDebug.nospace() << "sector density:";
	for (size_t k = 0; k < sectorDensity.size(); ++k)
		qDebug << ' ' << sectorDensity[k];

	double maxSectorDensity = sectorDensity[0];
	averageDensity = sectorDensity[0];
	for (int i = 1; i < UAV::sectorNum; ++i)
	{
		averageDensity += sectorDensity[i];
		if (maxSectorDensity < sectorDensity[i])
			maxSectorDensity = sectorDensity[i];
	}
	averageDensity /= UAV::sectorNum;
	densityDivision = math::equal0(averageDensity) ? 1000000.0 : maxSectorDensity/averageDensity;
	qDebug << "\naverage density: " << averageDensity << ", density division: " << densityDivision;
}

void RSU::notifySend(int sourceIndex)
{
	std::list<int> &path = env->accessPaths[sourceIndex];
	for (std::list<int>::iterator it = path.begin(); it != path.end(); ++it)
	{
		// any original direct previous hops that are not neighbors anymore need to be updated to this one
		if (routingTable.find(*it) != routingTable.end())
			for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
				if (itRT->second == routingTable[*it] && getNeighbor(itRT->second) == NULL)
					itRT->second = path.back();
		routingTable[*it] = path.back(); // path.back() is previous hop
	}
	env->schedule(0, path.back(), UAV::EventType::NOTIFYRECV, NOTIFY_PRIORITY, static_cast<void*>(new int(sourceIndex)));

	QDebug qDebug(QtMsgType::QtDebugMsg);
	qDebug.nospace() << "RSU notifySend at " << env->eventSequence << endl;
	qDebug << "routing table:";
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
		qDebug << ' ' << itRT->first << " -> " << itRT->second << ',';
}

UAV* RSU::getNeighbor(int index)
{
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		if ((*itN)->getSelfIndex() == index)
			return *itN;
	return NULL;
}

//////////////////////////////    UAV    //////////////////////////////
UAV::UAV(Solution *_env, Point& _pos, int _selfIndex) : env(_env), rsu(NULL), pos(_pos), dir(), hop(1), selfIndex(_selfIndex), decideAt(0), flyingTime(0), hopDist(0.0), sectorDensity(UAV::sectorNum, 0.0), forbidden(UAV::sectorNum, 0)
{

}

UAV::~UAV()
{
	vehicles.clear();
	neighbors.clear();
	routingTable.clear();
	sectorDensity.clear();
	forbidden.clear();
}

void UAV::handleEvent(int what, void *data)
{
	int *idata = NULL;
	switch (what)
	{
	case EventType::CHECKCONN:
		checkConn();
		break;
	case EventType::FLYING:
		flying();
		break;
	case EventType::DECIDE:
		decide();
		break;
	case EventType::NOTIFYSEND:
		idata = static_cast<int*>(data);
		notifySend(*idata);
		break;
	case EventType::NOTIFYRECV:
		idata = static_cast<int*>(data);
		notifyRecv(*idata);
		break;
	default:
		qCritical() << "Unknown EventType for class UAV.";
	}
	delete idata;
}

void UAV::checkConn()
{
	env->schedule(CHECKCONN_INTERVAL, selfIndex, UAV::EventType::CHECKCONN, CHECKCONN_PRIORITY);
	env->attainVehiclesNearby(pos, vehicles);
	env->attainNeighbors(pos, selfIndex, neighbors);
	for (itRT = routingTable.begin(); itRT != routingTable.end();)
	{
		if (itRT->first != env->rsu.getSelfIndex() && getNeighbor(itRT->second) == NULL)
			routingTable.erase(itRT++);
		else
			++itRT;
	}
	bool linkNotify = false;
	if (math::dist(pos, env->rsu.getPos()) < UAV::R)
	{
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
			routingTable[(*itN)->selfIndex] = (*itN)->selfIndex;
		if (rsu == NULL)
		{
			env->accessPaths[selfIndex].push_back(selfIndex);
			env->schedule(0, env->rsu.getSelfIndex(), RSU::EventType::NOTIFYSEND, NOTIFY_PRIORITY, static_cast<void*>(new int(selfIndex)));
			linkNotify = true;
		}
		rsu = &env->rsu;
		hop = 1;
		hopDist = math::dist(rsu->getPos(), pos);
		routingTable[rsu->getSelfIndex()] = rsu->getSelfIndex();
	}
	else
	{
		Q_ASSERT(neighbors.empty() == false);
		int minNbHop = 1000, nextHop = -1;
		double minNbDist = 100000.0;
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		{
			UAV *nb = *itN;
			routingTable[nb->selfIndex] = nb->selfIndex;
			double nbDist = math::dist(pos, nb->pos);
			if (minNbHop > nb->hop || (minNbHop == nb->hop && minNbDist > nbDist))
			{
				minNbHop = nb->hop;
				minNbDist = nbDist;
				nextHop = nb->selfIndex;
			}
		}
		hopDist = minNbDist;
		if (rsu != NULL || hop != minNbHop + 1 || routingTable[env->rsu.getSelfIndex()] != nextHop)
		{
			env->accessPaths[selfIndex].push_back(selfIndex);
			env->schedule(0, nextHop, UAV::EventType::NOTIFYSEND, NOTIFY_PRIORITY, static_cast<void*>(new int(selfIndex)));
			linkNotify = true;
		}
		rsu = NULL;
		hop = minNbHop + 1;
		routingTable[env->rsu.getSelfIndex()] = nextHop;
	}

	QDebug qDebug(QtMsgType::QtDebugMsg);
	if (linkNotify)
	{
		qDebug.nospace() << "UAV " << selfIndex << " notifySend at " << env->eventSequence << endl;
		qDebug << "routing table:";
		for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
			qDebug << ' ' << itRT->first << " -> " << itRT->second << ',';
		qDebug << endl;
	}
	qDebug.nospace() << "UAV " << selfIndex << " check connection at " << env->eventSequence << ", vn: " << vehicles.size() << ", nn: " << neighbors.size() << ", hop: " << hop;
}

void UAV::flying()
{
	pos.x += dir.x;
	pos.y += dir.y;
	if (--flyingTime > 0)
		env->schedule(FLYING_INTERVAL, selfIndex, UAV::EventType::FLYING, FLYING_PRIORITY);
	// qDebug().nospace() << "UAV " << selfIndex << " flying at " << env->eventSequence << ", pos: (" << pos.x << ',' << pos.y << ')';
}

void UAV::decide()
{
	env->schedule(DECIDE_INTERVAL, selfIndex, UAV::EventType::DECIDE, DECIDE_PRIORITY);
	decideAt = env->eventSequence;
	flyingTime = DECIDE_INTERVAL / FLYING_INTERVAL / 2;

	attainSectorDensity();

	int forbiddenNum = attainForbiddenSector();
	if (forbiddenNum == sectorNum)
	{
		qDebug().nospace() << "UAV " << selfIndex << " decide at " << env->eventSequence << ", all sector are forbidden";
		return;
	}

	int i = 0;
	double maxSectorDensity = sectorDensity[0];
	averageDensity = sectorDensity[0];
	for (i = 1; i < sectorNum; ++i)
	{
		averageDensity += sectorDensity[i];
		if (maxSectorDensity < sectorDensity[i])
			maxSectorDensity = sectorDensity[i];
	}
	averageDensity /= sectorNum;
	densityDivision = math::equal0(averageDensity) ? 1000000.0 : maxSectorDensity/averageDensity;
	qDebug().nospace() << "average density: " << averageDensity << ", density division: " << densityDivision;
	if (math::dist(pos, env->rsu.getPos()) > UAV::minGap)
	{
		std::vector<double> nbAverage, nbDivision;
		if (rsu != NULL)
		{
			nbAverage.push_back(rsu->getAverageDensity());
			nbDivision.push_back(rsu->getDensityDivision());
		}
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		{
			nbAverage.push_back((*itN)->averageDensity);
			nbDivision.push_back((*itN)->densityDivision);
		}
		int halfNbNum = static_cast<int>(nbAverage.size()) / 2;
		std::nth_element(nbAverage.begin(), nbAverage.begin()+halfNbNum, nbAverage.end());
		std::nth_element(nbDivision.begin(), nbDivision.begin()+halfNbNum, nbDivision.end(), std::greater<double>());
		if (averageDensity > nbAverage[halfNbNum] && densityDivision < nbDivision[halfNbNum])
		{
			qDebug().nospace() << "UAV " << selfIndex << " decide at " << env->eventSequence << ", nb-mid average density: " << nbAverage[halfNbNum] << ", nb-mid density division: " << nbDivision[halfNbNum];
			return;
		}
	}

	// select an optimal moving direction
	maxSectorDensity = 0.0;
	int maxI = 0;
	for (i = 0; i < sectorNum; ++i)
	{
		if (forbidden[i] == 0 && maxSectorDensity < sectorDensity[i])
		{
			maxSectorDensity = sectorDensity[i];
			maxI = i;
		}
	}
	double phi = (maxI + 0.5) * radTheta;
	dir.x = UAV::v * cos(phi);
	dir.y = UAV::v * sin(phi);
	Point dst(pos.x + flyingTime*dir.x, pos.y + flyingTime*dir.y);
	env->schedule(FLYING_INTERVAL, selfIndex, UAV::EventType::FLYING, FLYING_PRIORITY);
	qDebug().nospace() << "UAV " << selfIndex << " decide at " << env->eventSequence << ", pos: (" << pos.x << ',' << pos.y << "), dst: (" << dst.x << ',' << dst.y << "), phi: " << 180*phi/M_PI;
}

void UAV::notifySend(int sourceIndex)
{
	QDebug qDebug(QtMsgType::QtDebugMsg);
	qDebug.nospace() << "UAV " << selfIndex << " notifySend at " << env->eventSequence << endl;

	std::list<int> &path = env->accessPaths[sourceIndex];
	UAV *prevHop = getNeighbor(path.back()); // path.back() is previous hop
	Q_ASSERT(prevHop != NULL);
	// delete indirect previous hops that cannot be routed from the direct previous hop anymore
	qDebug << "delete indirect:";
	for (itRT = routingTable.begin(); itRT != routingTable.end();)
	{
		if (itRT->second == prevHop->selfIndex && itRT->first != itRT->second && prevHop->routingTable.find(itRT->first) == prevHop->routingTable.end())
		{
			qDebug << ' ' << itRT->first << " -> " << itRT->second << ',';
			routingTable.erase(itRT++);
		}
		else
			++itRT;
	}
	qDebug << "\ninsert indirect:";
	// insert indirect previous hops that can be routed from the direct previous hop
	for (itRT = prevHop->routingTable.begin(); itRT != prevHop->routingTable.end(); ++itRT)
	{
		if (itRT->first != env->rsu.getSelfIndex() && itRT->first != selfIndex && routingTable.find(itRT->first) == routingTable.end() && getNeighbor(itRT->first) == NULL)
		{
			qDebug << ' ' << itRT->first << " -> " << prevHop->selfIndex << ',';
			int nextHop = getNeighbor(itRT->second) != NULL ? itRT->second : prevHop->selfIndex;
			routingTable.insert(std::pair<int, int>(itRT->first, nextHop));
		}
	}
	for (std::list<int>::iterator it = path.begin(); it != path.end(); ++it)
	{
		// any original direct previous hops that are not neighbors anymore need to be updated to this one
		if (routingTable.find(*it) != routingTable.end())
			for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
				if (itRT->second == routingTable[*it] && getNeighbor(itRT->second) == NULL)
					itRT->second = prevHop->selfIndex;
		routingTable[*it] = prevHop->selfIndex;
	}
	env->accessPaths[sourceIndex].push_back(selfIndex);
	int nextHop = getNextHop();
	if (nextHop != env->rsu.getSelfIndex())
		env->schedule(0, nextHop, UAV::EventType::NOTIFYSEND, NOTIFY_PRIORITY, static_cast<void*>(new int(sourceIndex)));
	else
		env->schedule(0, nextHop, RSU::EventType::NOTIFYSEND, NOTIFY_PRIORITY, static_cast<void*>(new int(sourceIndex)));

	qDebug << "\nrouting table:";
	for (itRT = routingTable.begin(); itRT != routingTable.end(); ++itRT)
		qDebug << ' ' << itRT->first << " -> " << itRT->second << ',';
}

void UAV::notifyRecv(int sourceIndex)
{
	std::list<int> &path = env->accessPaths[sourceIndex];
	path.pop_back(); // after pop_back(), path.back() is previous hop
	if (!path.empty())
		env->schedule(0, path.back(), UAV::EventType::NOTIFYRECV, NOTIFY_PRIORITY, static_cast<void*>(new int(sourceIndex)));
	qDebug().nospace() << "UAV " << selfIndex << " notifyRecv at " << env->eventSequence;
}

void UAV::attainSectorDensity()
{
	const double closeMulitplier = 1.0;
	for (size_t k = 0; k < sectorDensity.size(); ++k)
		sectorDensity[k] = 0.0;
	for (itV = vehicles.begin(); itV != vehicles.end(); ++itV)
	{
		double angle = acos((vehicleTable[*itV].x - pos.x) / math::dist(pos, vehicleTable[*itV]));
		if (vehicleTable[*itV].y < pos.y)
			angle = 2*M_PI - angle;
		int sector = angle / radTheta;
		double coverCoefficient = 1.0, closeCoefficient = 0.0;
		if (rsu != NULL && (closeCoefficient = math::dist(rsu->getPos(), vehicleTable[*itV])/UAV::r) < 1.0)
			coverCoefficient += 1.0 + closeMulitplier*(1.0 - closeCoefficient);
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
			if ((closeCoefficient = math::dist((*itN)->pos, vehicleTable[*itV])/UAV::r) < 1.0)
				coverCoefficient += 1.0 + closeMulitplier*(1.0 - closeCoefficient);
		sectorDensity[sector] += 1.0/coverCoefficient;
	}
	QDebug qDebug(QtMsgType::QtDebugMsg);
	qDebug.nospace() << "sector density:";
	for (size_t k = 0; k < sectorDensity.size(); ++k)
		qDebug << ' ' << sectorDensity[k];
}

int UAV::attainForbiddenSector()
{
	int i = 0;
	double R2 = UAV::R * UAV::R, minGap2 = UAV::minGap * UAV::minGap;
	std::list<UAV*> nbLess, nbEqual, nbGreater;
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
	{
		UAV *nb = *itN;
		if (nb->hop < hop)
			nbLess.push_back(nb);
		else if (nb->hop == hop)
			nbEqual.push_back(nb);
		else // nb->hop > hop
			nbGreater.push_back(nb);
	}
	for (i = 0; i < sectorNum; ++i)
	{
		forbidden[i] = 0;
		double phi = (i + 0.5) * radTheta;
		dir.x = UAV::v * cos(phi);
		dir.y = UAV::v * sin(phi);
		Point dst(pos.x + flyingTime*dir.x, pos.y + flyingTime*dir.y);
		bool flag = false;
		double dist2 = math::dist2(env->rsu.getPos(), pos);
		if (dist2 < minGap2) // boot process
		{
			flag = dist2 > math::dist2(env->rsu.getPos(), dst); // moving more close to the RSU
			forbidden[i] = flag ? 1 : 0;
			if (flag)
				continue;
			for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
				flag |= math::dist2((*itN)->getPos(), dst) < minGap2;
			forbidden[i] = flag ? 1 : 0;
			continue;
		}
		// case 1: leave any neighbor with greater hop count
		for (itN = nbGreater.begin(); itN != nbGreater.end(); ++itN)
			flag |= math::dist2((*itN)->pos, dst) >= math::square(UAV::R - (*itN)->maxRelativeMoving());
		forbidden[i] = flag ? 1 : 0;
		if (flag)
			continue;
		// case 2: leave all neighbors with less hop count
		flag = true;
		if (rsu == NULL)
		{
			for (itN = nbLess.begin(); itN != nbLess.end(); ++itN)
				flag &= math::dist2((*itN)->pos, dst) >= math::square(UAV::R - (*itN)->maxRelativeMoving());
		}
		else if (math::dist2(rsu->getPos(), dst) < R2) // within rsu
			flag = false;
		if (flag)
		{
			bool closest = true;
			for (itN = nbEqual.begin(); itN != nbEqual.end(); ++itN)
			{
				UAV *nb = *itN;
				if (math::dist2(nb->pos, dst) < math::square(UAV::R - nb->maxRelativeMoving()) && nb->hopDist < hopDist)
					closest = false;
			}
			forbidden[i] = closest ? 1 : 0;
			if (closest)
				continue;
		}
		// case 3: move too close to neighbors or rsu
		if (rsu != NULL && math::dist2(rsu->getPos(), dst) < minGap2)
		{
			forbidden[i] = 1;
			continue;
		}
		for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		{
			if (math::dist2((*itN)->getPos(), dst) < minGap2)
			{
				forbidden[i] = 1;
				break;
			}
		}
	}
	std::string forbiddenStr;
	int forbiddenNum = 0;
	for (i = 0; i < sectorNum; ++i)
	{
		forbiddenStr.push_back(' ');
		forbiddenStr.push_back(forbidden[i] + '0');
		forbiddenNum += forbidden[i];
	}
	qDebug().nospace() << "forbidden:" << forbiddenStr.c_str();
	return forbiddenNum;
}

int UAV::getNextHop()
{
	itRT = routingTable.find(env->rsu.getSelfIndex());
	Q_ASSERT(itRT != routingTable.end());
	return itRT->second;
}

UAV* UAV::getNeighbor(int index)
{
	for (itN = neighbors.begin(); itN != neighbors.end(); ++itN)
		if ((*itN)->selfIndex == index)
			return *itN;
	return NULL;
}

double UAV::maxRelativeMoving()
{
	int decideSeconds = DECIDE_INTERVAL / 10, flyingSeconds = decideSeconds / 2, sinceDecide = env->eventSequence - decideAt;
	if (sinceDecide == DECIDE_INTERVAL)
		sinceDecide = 0; // the call of UAV::decide() has order, leading to concurrency problem
	sinceDecide /= 10;
	int movingSeconds = sinceDecide <= flyingSeconds ? flyingSeconds - sinceDecide : sinceDecide - flyingSeconds;
	return movingSeconds * UAV::v;
}

//////////////////////////////    ReadConfig    //////////////////////////////
ReadConfig::ReadConfig(const char *filename)
{
	fd = fopen(filename, "r");
	if (fd == NULL)
	{
		qFatal("Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	fgets(buf, 100, fd);
}

ReadConfig::~ReadConfig()
{
	fclose(fd);
}

void ReadConfig::parse(std::string& key, std::string& value)
{
	std::string line(buf);
	line.pop_back(); // drop '\n'
	for (size_t i = 0; i < line.size(); ++i)
	{
		if (line[i] == '=')
		{
			Q_ASSERT(i > 0 && i < line.size()-1);
			key = line.substr(0, i);
			value = line.substr(i+1);
			break;
		}
	}

	fgets(buf, 100, fd);
}

bool ReadConfig::hasNext()
{
	return buf[0] != '\n';
}

void configEnv()
{
	ReadConfig readConfig("DAVN.conf");
	std::string key, value;
	while (readConfig.hasNext())
	{
		readConfig.parse(key, value);
		if (key == "r")
			UAV::r = RSU::r = atof(value.c_str());
		else if (key == "R")
			UAV::R = RSU::R = atof(value.c_str());
		else if (key == "v")
			UAV::v = atof(value.c_str());
		else if (key == "minGap")
			UAV::minGap = atof(value.c_str());
		else if (key == "theta")
			UAV::theta = atoi(value.c_str());
	}
	UAV::sectorNum = 360 / UAV::theta;
	UAV::radTheta = M_PI * UAV::theta / 180.0;
}
