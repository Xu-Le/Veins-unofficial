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

#ifndef __ROUTINGSTATISTICCOLLECTOR_H__
#define __ROUTINGSTATISTICCOLLECTOR_H__

#include <omnetpp/csimplemodule.h>

/**
 * @brief Collect global statistics produced by unicast routing protocol for performance assessment.
 *
 * @author Xu Le
 * @ingroup routingLayer
 */
class RoutingStatisticCollector : public ::omnetpp::cSimpleModule
{
public:
	/** @name constructor, destructor. */
	///@{
	RoutingStatisticCollector() : cSimpleModule() {}
	RoutingStatisticCollector(unsigned stacksize) : cSimpleModule(stacksize) {}
	virtual ~RoutingStatisticCollector();
	///@}

	void initialize(int stage) override;
	void finish() override;

public:
	static long globalRequests; ///< how many routing requests sent by source vehicle.
	static long globalArrivals; ///< how many routing requests successfully arrived at destination vehicle.
	static long globalDuplications; ///< the equivalent of network overhead, which measures how severe the broadcast storm is.
	static double globalDelayAccumulation; ///< accumulative total of routing delays of all the vehicles.
};

#endif /* __ROUTINGSTATISTICCOLLECTOR_H__ */
