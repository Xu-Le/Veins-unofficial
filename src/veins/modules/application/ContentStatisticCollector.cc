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

#include <fstream>
#include "veins/modules/application/ContentStatisticCollector.h"

Define_Module(ContentStatisticCollector);

long ContentStatisticCollector::globalContentRequests = 0;
long ContentStatisticCollector::globalParticipatingRSUNum = 0;
double ContentStatisticCollector::globalDSRCFlux = 0.0;
double ContentStatisticCollector::globalCellularFlux = 0.0;
double ContentStatisticCollector::globalConsumingTime = 0.0;
double ContentStatisticCollector::globalDownloadingTime = 0.0;
double ContentStatisticCollector::globalInterruptedTime = 0.0;
double ContentStatisticCollector::globalConsumptionStartingDelay = 0.0;

void ContentStatisticCollector::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
		EV << "ContentStatisticCollector::initialize() called.\n";
}

void ContentStatisticCollector::finish()
{
	EV << "ContentStatisticCollector::finish() called.\n";

	// record statistics
	recordScalar("globalContentRequests", globalContentRequests);
	recordScalar("globalParticipatingRSUNum", globalParticipatingRSUNum);
	recordScalar("globalDSRCFlux", globalDSRCFlux);
	recordScalar("globalCellularFlux", globalCellularFlux);
	recordScalar("globalDownloadingTime", globalDownloadingTime);
	recordScalar("globalInterruptedTime", globalInterruptedTime);

	if (globalDSRCFlux + globalCellularFlux < 1e-6)
		globalDSRCFlux = globalCellularFlux = 1.0;
	if (globalConsumingTime < 1e-6)
		globalConsumingTime = 1.0;
	if (globalContentRequests == 0)
		globalContentRequests = 1;

	double DSRCFluxRatio = globalDSRCFlux / (globalDSRCFlux + globalCellularFlux);
	double interruptedTimeRatio = globalInterruptedTime / globalConsumingTime;
	double averageConsumptionStartingDelay = globalConsumptionStartingDelay / globalContentRequests;

	recordScalar("DSRCFluxRatio", DSRCFluxRatio);
	recordScalar("interruptedTimeRatio", interruptedTimeRatio);
	recordScalar("averageConsumptionStartingDelay", averageConsumptionStartingDelay);

	// append statistic to file for figuring in MATLAB
	std::ofstream fout("contentStatistics.csv", std::ios_base::out | std::ios_base::app);
	if ( !fout.is_open() )
	{
		error("cannot open file routingStatistics.csv!");
	}
	else
	{
		fout << globalContentRequests << ',' << globalParticipatingRSUNum << ',' << globalDSRCFlux << ',' << globalCellularFlux << ',' << globalDownloadingTime << ',' << globalInterruptedTime << ',';
		fout << DSRCFluxRatio << ',' << interruptedTimeRatio << ',' << averageConsumptionStartingDelay << std::endl;
	}
	fout.close();

	cComponent::finish();
}

ContentStatisticCollector::~ContentStatisticCollector()
{
	EV << "ContentStatisticCollector::~ContentStatisticCollector() called.\n";
}

