//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __WARNINGSTATISTICCOLLECTOR_H__
#define __WARNINGSTATISTICCOLLECTOR_H__

#include <omnetpp/csimplemodule.h>

/**
 * @brief Collect global statistics produced by broadcast or geocast routing protocol for performance assessment.
 *
 * @author Xu Le
 * @ingroup routingLayer
 */
class WarningStatisticCollector : public ::omnetpp::cSimpleModule
{
public:
	/** @name constructors, destructor. */
	///@{
	WarningStatisticCollector() : cSimpleModule() {}
	WarningStatisticCollector(unsigned stacksize) : cSimpleModule(stacksize) {}
	virtual ~WarningStatisticCollector();
	///@}

	void initialize(int stage) override;
	void finish() override;

public:
	static long globalNotifications; ///< how many warning notifications sent by source RSU.
	static long globalTargets;       ///< how many vehicles is the targets of the warning notifications.
	static long globalPenetrations;  ///< how many targets successfully received warning notifications.
	static long globalInterferences; ///< how many vehicles outside ROI received warning notifications.
	static long globalDuplications;  ///< the equivalent of network overhead, which measures how severe the broadcast storm is.
	static double globalMaxDelayAccumulation; ///< accumulative total of max delay from RSU to the farthest vehicle each time.
};

#endif /* __WARNINGSTATISTICCOLLECTOR_H__ */
