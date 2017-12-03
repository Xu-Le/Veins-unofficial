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

#ifndef __CONTENTSTATISTICCOLLECTOR_H__
#define __CONTENTSTATISTICCOLLECTOR_H__

#include <omnetpp/csimplemodule.h>

/**
 * @brief Collect global statistics produced by content client and content RSU for performance assessment.
 *
 * @author Xu Le
 * @ingroup applLayer
 */
class ContentStatisticCollector : public ::omnetpp::cSimpleModule
{
public:
	/** @name constructor, destructor. */
	///@{
	ContentStatisticCollector() : cSimpleModule() {}
	ContentStatisticCollector(unsigned stacksize) : cSimpleModule(stacksize) {}
	virtual ~ContentStatisticCollector();
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	int slotNum;        ///< predication slot number used by ContentRSU.
	int contentSize;    ///< total content size required by ContentClient.
	int trafficDensity; ///< traffic flow density measured in vph.
	int deployInterval; ///< deploy interval of RSUs measured in meters.
	bool isComparison;  ///< indicates whether are SimpleContentRSU's results.

public:
	static long globalContentRequests; ///< accumulated content request number in the whole simulation.
	static long globalContentSize;  ///< accumulated content size measured in bytes in the whole simulation.
	static double globalDirectFlux; ///< accumulated amount of flux measured in bytes produced by direct downloading approach.
	static double globalRelayFlux;  ///< accumulated amount of flux measured in bytes produced by relay approach.
	static double globalCarryFlux;  ///< accumulated amount of flux measured in bytes produced by carry-and-forward approach.
	static double globalStorageAmount; ///< accumulated storage amount measured in bytes in all nodes, it is used to calculate redundant data amount.
	static double globalDownloadingTime; ///< accumulated total time cost in cooperative downloading process.
};

#endif /* __CONTENTSTATISTICCOLLECTOR_H__ */
