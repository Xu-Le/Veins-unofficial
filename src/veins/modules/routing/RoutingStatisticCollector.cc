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

#include <fstream>
#include "veins/modules/routing/RoutingStatisticCollector.h"

Define_Module(RoutingStatisticCollector);

long RoutingStatisticCollector::globalRequests = 0;
long RoutingStatisticCollector::globalArrivals = 0;
long RoutingStatisticCollector::globalDuplications = 0;
double RoutingStatisticCollector::globalDelayAccumulation = 0.0;

void RoutingStatisticCollector::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		EV << "RoutingStatisticCollector::initialize() called.\n";
	}
	else
	{
		EV << "RoutingStatisticCollector initialized.\n";
	}
}

void RoutingStatisticCollector::finish()
{
	EV << "RoutingStatisticCollector::finish() called.\n";

	// record statistics
	recordScalar("globalRequests", globalRequests);
	recordScalar("globalArrivals", globalArrivals);
	recordScalar("globalDuplications", globalDuplications);
	recordScalar("globalDelayAccumulation", globalDelayAccumulation);

	if (globalRequests == 0)
	    globalRequests = 1;
	if (globalArrivals == 0)
	    globalArrivals = 1;

	double deliveryRatio = static_cast<double>(globalArrivals) / globalRequests;
	double normalizedDuplications = static_cast<double>(globalDuplications) / globalRequests;
	double averageDelay = globalDelayAccumulation / globalArrivals;

	recordScalar("deliveryRatio", deliveryRatio);
	recordScalar("normalizedDuplications", normalizedDuplications);
	recordScalar("averageDelay", averageDelay);

	// append statistic to file for figuring in MATLAB
	std::ofstream fout("routingStatistics.csv", std::ios_base::out | std::ios_base::app);
	if ( !fout.is_open() )
	{
		error("cannot open file routingStatistics.csv!");
	}
	else
	{
		fout << globalRequests << ',' << globalArrivals << ',' << globalDuplications << ',' << globalDelayAccumulation << ',' << deliveryRatio << ',' << normalizedDuplications << ',' << averageDelay << std::endl;
	}
	fout.close();

	cComponent::finish();
}

RoutingStatisticCollector::~RoutingStatisticCollector()
{
	EV << "RoutingStatisticCollector::~RoutingStatisticCollector() called.\n";
}
