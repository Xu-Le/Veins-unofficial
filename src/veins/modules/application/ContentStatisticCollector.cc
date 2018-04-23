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
long ContentStatisticCollector::globalContentSize = 0;
double ContentStatisticCollector::globalDirectFlux = 0.0;
double ContentStatisticCollector::globalRelayFlux = 0.0;
double ContentStatisticCollector::globalCarryFlux = 0.0;
double ContentStatisticCollector::globalCellularFlux = 0.0;
double ContentStatisticCollector::globalStorageAmount = 0.0;
double ContentStatisticCollector::globalConsumingTime = 0.0;
double ContentStatisticCollector::globalDownloadingTime = 0.0;
double ContentStatisticCollector::globalInterruptedTime = 0.0;
double ContentStatisticCollector::globalConsumptionStartingDelay = 0.0;

void ContentStatisticCollector::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		EV << "ContentStatisticCollector::initialize() called.\n";

		slotNum = par("slotNum").longValue();
		contentSize = par("contentSize").longValue();
		contentQuality = par("contentQuality").longValue();
		cellularRate = par("cellularRate").longValue();
		trafficDensity = par("trafficDensity").longValue();
		vehicleSpeed = par("vehicleSpeed").longValue();
		deployInterval = par("deployInterval").longValue();
		isComparison = par("isComparison").boolValue();
	}
}

void ContentStatisticCollector::finish()
{
	EV << "ContentStatisticCollector::finish() called.\n";

	// record statistics
	recordScalar("globalContentRequests", globalContentRequests);
	recordScalar("globalContentSize", globalContentSize);
	recordScalar("globalDirectFlux", globalDirectFlux);
	recordScalar("globalRelayFlux", globalRelayFlux);
	recordScalar("globalCarryFlux", globalCarryFlux);
	recordScalar("globalCellularFlux", globalCellularFlux);
	recordScalar("globalStorageAmount", globalStorageAmount);
	recordScalar("globalConsumingTime", globalConsumingTime);
	recordScalar("globalDownloadingTime", globalDownloadingTime);
	recordScalar("globalInterruptedTime", globalInterruptedTime);

	double globalTotalFlux = globalDirectFlux + globalRelayFlux + globalCarryFlux + globalCellularFlux;
	if (globalTotalFlux < 1e-6)
		globalTotalFlux = 1.0;
	if (globalConsumingTime < 1e-6)
		globalConsumingTime = 1.0;
	if (globalDownloadingTime < 1e-6)
		globalDownloadingTime = 1.0;
	if (globalContentRequests == 0)
		globalContentRequests = 1;
	if (globalContentSize == 0)
		globalContentSize = 1;

	double averageDownloadingRate = globalContentSize / globalDownloadingTime / 128.0; // measured in kbps
	double directFluxRatio = globalDirectFlux / globalTotalFlux;
	double relayFluxRatio = globalRelayFlux / globalTotalFlux;
	double carryFluxRatio = globalCarryFlux / globalTotalFlux;
	double cellularFluxRatio = globalCellularFlux / globalTotalFlux;
	double redundantStorageRatio = globalStorageAmount / globalContentSize - 1.0;
	double interruptedTimeRatio = globalInterruptedTime / globalConsumingTime;
	double averageConsumptionStartingDelay = globalConsumptionStartingDelay / globalContentRequests;

	recordScalar("averageDownloadingRate", averageDownloadingRate);
	recordScalar("directFluxRatio", directFluxRatio);
	recordScalar("relayFluxRatio", relayFluxRatio);
	recordScalar("carryFluxRatio", carryFluxRatio);
	recordScalar("cellularFluxRatio", cellularFluxRatio);
	recordScalar("redundantStorageRatio", redundantStorageRatio);
	recordScalar("interruptedTimeRatio", interruptedTimeRatio);
	recordScalar("averageConsumptionStartingDelay", averageConsumptionStartingDelay);

	// append statistic to file for figuring in MATLAB
	const char *resultFile = isComparison ? "contentStatistics_cmp.csv" : "contentStatistics.csv";
	std::ofstream fout(resultFile, std::ios_base::out | std::ios_base::app);
	if ( !fout.is_open() )
	{
		error("cannot open file routingStatistics.csv!");
	}
	else
	{
		fout << slotNum << ',' << contentSize << ',' << contentQuality << ',' << trafficDensity << ',' << vehicleSpeed << ',' << deployInterval << ',';
		fout << globalContentRequests << ',' << globalContentSize << ',' << globalConsumingTime << ',' << globalDownloadingTime << ',' << globalInterruptedTime << ',';
		fout << averageDownloadingRate << ',' << directFluxRatio << ',' << relayFluxRatio << ',' << carryFluxRatio << ',' << cellularFluxRatio << ',';
		fout << redundantStorageRatio << ',' << interruptedTimeRatio << ',' << averageConsumptionStartingDelay << std::endl;
	}
	fout.close();

	cComponent::finish();
}

ContentStatisticCollector::~ContentStatisticCollector()
{
	EV << "ContentStatisticCollector::~ContentStatisticCollector() called.\n";
}

