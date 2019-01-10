//
// Copyright (C) 2016-2019 Xu Le <xmutongxinXuLe@163.com>
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

long RoutingStatisticCollector::gRREQs = 0;
long RoutingStatisticCollector::gRREPs = 0;
long RoutingStatisticCollector::gRERRs = 0;
long RoutingStatisticCollector::gRREPACKs = 0;
long RoutingStatisticCollector::gRREQps = 0;
long RoutingStatisticCollector::gRREPps = 0;
long RoutingStatisticCollector::gLocalRepairs = 0;
long RoutingStatisticCollector::gPktsLinkLost = 0;
long RoutingStatisticCollector::gPktsOverLost = 0;
int64_t RoutingStatisticCollector::gDataBitsSent = 0;
int64_t RoutingStatisticCollector::gDataBitsRecv = 0;
long RoutingStatisticCollector::gDataPktsSent = 0;
long RoutingStatisticCollector::gDataPktsRecv = 0;
long RoutingStatisticCollector::gDataPktsDuplicated = 0;
int64_t RoutingStatisticCollector::gDataBitsTransmitted = 0;
int64_t RoutingStatisticCollector::gCtrlBitsTransmitted = 0;
long RoutingStatisticCollector::gDataPktsTransmitted = 0;
long RoutingStatisticCollector::gCtrlPktsTransmitted = 0;
simtime_t RoutingStatisticCollector::gRouteAcquisitionTime;
simtime_t RoutingStatisticCollector::gEndtoendDelay;

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
	double gCtrlBitsFraction = 0.0;
	if (gCtrlBitsTransmitted > 0)
		gCtrlBitsFraction = static_cast<double>(gCtrlBitsTransmitted) / (gDataBitsTransmitted+gCtrlBitsTransmitted);
	if (gDataPktsRecv == 0)
		gDataPktsRecv = 1;
	gEndtoendDelay /= gDataPktsRecv;
	recordScalar("gRREQs", gRREQs);
	recordScalar("gRREPs", gRREPs);
	recordScalar("gRERRs", gRERRs);
	recordScalar("gRREPACKs", gRREPACKs);
	recordScalar("gRREQps", gRREQps);
	recordScalar("gRREPps", gRREPps);
	recordScalar("gLocalRepairs", gLocalRepairs);
	recordScalar("gPktsLinkLost", gPktsLinkLost);
	recordScalar("gPktsOverLost", gPktsOverLost);
	recordScalar("gDataBitsSent", gDataBitsSent);
	recordScalar("gDataBitsRecv", gDataBitsRecv);
	recordScalar("gDataPktsSent", gDataPktsSent);
	recordScalar("gDataPktsRecv", gDataPktsRecv);
	recordScalar("gDataPktsDuplicated", gDataPktsDuplicated);
	recordScalar("gDataBitsTransmitted", gDataBitsTransmitted);
	recordScalar("gCtrlBitsTransmitted", gCtrlBitsTransmitted);
	recordScalar("gDataPktsTransmitted", gDataPktsTransmitted);
	recordScalar("gCtrlPktsTransmitted", gCtrlPktsTransmitted);
	recordScalar("gCtrlBitsFraction", gCtrlBitsFraction);
	recordScalar("gRouteAcquisitionTime", gRouteAcquisitionTime.dbl());
	recordScalar("gAverageEndtoendDelay", gEndtoendDelay.dbl());

	// append statistic to file for figuring in MATLAB
	std::ofstream fout("routingStatistics.csv", std::ios_base::out | std::ios_base::app);
	if ( !fout.is_open() )
	{
		error("cannot open file routingStatistics.csv!");
	}
	else
	{
		fout << gRREQs << ',' << gRREPs << ',' << gRERRs << ',' << gRREPACKs << std::endl;
	}
	fout.close();

	cComponent::finish();
}

RoutingStatisticCollector::~RoutingStatisticCollector()
{
	EV << "RoutingStatisticCollector::~RoutingStatisticCollector() called.\n";
}
