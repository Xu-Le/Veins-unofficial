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

public:
    static long globalContentRequests; ///< accumulated content request number in the whole simulation.
    static long globalParticipatingRSUNum; ///< accumulated number of RSUs participated in the cooperative downloading process.
    static double globalDSRCFlux; ///< accumulated amount of flux produced by DSRC approach.
    static double globalCellularFlux; ///< accumulated amount of flux produced by cellular approach.
    static double globalConsumingTime; ///< accumulated total time cost in data consuming process.
    static double globalDownloadingTime; ///< accumulated total time cost in cooperative downloading process.
    static double globalInterruptedTime; ///< accumulated interrupted time in cooperative downloading process.
    static double globalConsumptionStartingDelay; ///< accumulated time delay from content request to content consumption starting.
};

#endif /* __CONTENTSTATISTICCOLLECTOR_H__ */
