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
#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/routing/WarningStatisticCollector.h"

Define_Module(WarningStatisticCollector);

std::map<int /* GUID */, LAddress::L3Type> warningMaxDelayHelper;

long WarningStatisticCollector::globalNotifications = 0;
long WarningStatisticCollector::globalTargets = 0;
long WarningStatisticCollector::globalPenetrations = 0;
long WarningStatisticCollector::globalInterferences = 0;
long WarningStatisticCollector::globalDuplications = 0;
double WarningStatisticCollector::globalMaxDelayAccumulation = 0.0;

void WarningStatisticCollector::initialize(int stage)
{
	cComponent::initialize(stage);

	if (stage == 0)
	{
		EV << "WarningStatisticCollector::initialize() called.\n";
	}
	else
	{
		EV << "WarningStatisticCollector initialized.\n";
	}
}

void WarningStatisticCollector::finish()
{
	EV << "WarningStatisticCollector::finish() called.\n";

	warningMaxDelayHelper.clear();

	// record statistics
	recordScalar("globalNotifications", globalNotifications);
	recordScalar("globalTargets", globalTargets);
	recordScalar("globalPenetrations", globalPenetrations);
	recordScalar("globalInterferences", globalInterferences);
	recordScalar("globalDuplications", globalDuplications);
	recordScalar("globalMaxDelayAccumulation", globalMaxDelayAccumulation);

	if (globalTargets == 0)
	    globalTargets = 1;
	if (globalNotifications == 0)
	    globalNotifications = 1;

	double penetrationRatio = static_cast<double>(globalPenetrations) / globalTargets;
	double normalizedInterferences = static_cast<double>(globalInterferences) / globalTargets;
	double normalizedDuplications = static_cast<double>(globalDuplications) / globalTargets;
	double averageMaxDelay = globalMaxDelayAccumulation / globalNotifications;

	recordScalar("penetrationRatio", penetrationRatio);
	recordScalar("normalizedInterferences", normalizedInterferences);
	recordScalar("normalizedDuplications", normalizedDuplications);
	recordScalar("averageMaxDelay", averageMaxDelay);

	// append statistic to file for figuring in MATLAB
	std::ofstream fout("warningStatistics.csv", std::ios_base::out | std::ios_base::app);
	if ( !fout.is_open() )
	{
		error("cannot open file warningStatistics.csv!");
	}
	else
	{
		fout << globalNotifications << ',' << globalTargets << ',' << globalPenetrations << ',' << globalInterferences << ',' << globalDuplications << ',' << globalMaxDelayAccumulation << ',';
		fout << penetrationRatio << ',' << normalizedInterferences << ',' << normalizedDuplications << ',' << averageMaxDelay << std::endl;
	}
	fout.close();

	cComponent::finish();
}

WarningStatisticCollector::~WarningStatisticCollector()
{
	EV << "WarningStatisticCollector::~WarningStatisticCollector() called.\n";
}
