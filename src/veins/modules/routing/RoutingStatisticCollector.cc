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

long RoutingStatisticCollector::gRoutings = 0;
long RoutingStatisticCollector::gComplete = 0;
long RoutingStatisticCollector::gIncomplete = 0;
long RoutingStatisticCollector::gRREQs = 0;
long RoutingStatisticCollector::gRREPs = 0;
long RoutingStatisticCollector::gRERRs = 0;
long RoutingStatisticCollector::gRREPACKs = 0;
long RoutingStatisticCollector::gRREQps = 0;
long RoutingStatisticCollector::gRREPps = 0;
long RoutingStatisticCollector::gLRs = 0;
long RoutingStatisticCollector::gDPSRs = 0;
long RoutingStatisticCollector::gRouteSuccess = 0;
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
long RoutingStatisticCollector::gDataPktsTransmittedSuccessful = 0;
simtime_t RoutingStatisticCollector::gRouteAcquisitionTime;
simtime_t RoutingStatisticCollector::gEndtoendDelay;
simtime_t RoutingStatisticCollector::gLatestRouting;

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
	int gLowerLayerLost = gDataPktsTransmitted - gDataPktsTransmittedSuccessful;
	double gCtrlBitsFraction = 0.0, gCtrlPktsFraction = 0.0;
	if (gCtrlBitsTransmitted > 0 && gDataBitsRecv > 0)
		gCtrlBitsFraction = static_cast<double>(gCtrlBitsTransmitted) / gDataBitsRecv;
	if (gCtrlPktsTransmitted > 0 && gDataPktsRecv > 0)
		gCtrlPktsFraction = static_cast<double>(gCtrlPktsTransmitted) / gDataPktsRecv;
	if (gRouteSuccess == 0)
		gRouteSuccess = 1;
	if (gDataPktsRecv == 0)
		gDataPktsRecv = 1;
	gRouteAcquisitionTime /= gRouteSuccess;
	gEndtoendDelay /= gDataPktsRecv;
	recordScalar("gComplete", gComplete);
	recordScalar("gIncomplete", gIncomplete);
	recordScalar("gRREQs", gRREQs);
	recordScalar("gRREPs", gRREPs);
	recordScalar("gRERRs", gRERRs);
	//recordScalar("gRREPACKs", gRREPACKs);
	recordScalar("gRREQps", gRREQps);
	recordScalar("gRREPps", gRREPps);
	recordScalar("gLRs", gLRs);
	recordScalar("gDPSRs", gDPSRs);
	recordScalar("gLocalRepairs", gLocalRepairs);
	recordScalar("gLowerLayerLost", gLowerLayerLost);
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
	recordScalar("gCtrlPktsFraction", gCtrlPktsFraction);
	recordScalar("gAverageRouteAcquisitionTime", gRouteAcquisitionTime.dbl());
	recordScalar("gAverageEndtoendDelay", gEndtoendDelay.dbl());

	// append statistic to file for figuring in MATLAB
	std::ofstream fout(par("resultFile").stringValue(), std::ios_base::out | std::ios_base::app);
	if (!fout.is_open())
		error("cannot open file routingStatistics.csv!");

	int trafficDensity = par("trafficDensity").longValue();
	int roadSpeedLimit = par("roadSpeedLimit").longValue();
	int rtExpirationTime = par("rtExpirationTime").longValue();
	int packetsPerSecond = par("packetsPerSecond").longValue();
	fout << trafficDensity << ',' << roadSpeedLimit << ',' << rtExpirationTime << ',' << packetsPerSecond << ',';
	fout << gDataPktsSent << ',' << gDataPktsRecv << ',' << gEndtoendDelay << ',' << gLowerLayerLost << ',' << gPktsOverLost << ',' << gCtrlPktsFraction << ',';
	fout << gComplete << ',' << gRREQs << ',' << gRREPs << ',' << gRERRs << ',' << gRREQps << ',' << gRREPps << ',' << gLRs << ',' << gDPSRs << ',' << gLocalRepairs << std::endl;

	fout.close();

	cComponent::finish();
}

RoutingStatisticCollector::~RoutingStatisticCollector()
{
	EV << "RoutingStatisticCollector::~RoutingStatisticCollector() called.\n";
}
