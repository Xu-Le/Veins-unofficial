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

#ifndef __ROUTINGSTATISTICCOLLECTOR_H__
#define __ROUTINGSTATISTICCOLLECTOR_H__

#include <omnetpp/csimplemodule.h>

using omnetpp::simtime_t;

/**
 * @brief Collect global statistics produced by routing protocol for performance assessment.
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
	static long gRREQs;    ///< how many RREQs generated in simulation.
	static long gRREPs;    ///< how many RREPs generated in simulation.
	static long gRERRs;    ///< how many RERRs generated in simulation.
	static long gRREPACKs; ///< how many RREPACKs generated in simulation.
	static long gRREQps;   ///< how many RREQps generated in simulation.
	static long gRREPps;   ///< how many RREPps generated in simulation.
	static long gLocalRepairs; ///< how many times local repair performs in simulation.
	static long gPktsLinkLost; ///< how many data packets lost due to link broken in simulation.
	static long gPktsOverLost; ///< how many data packets lost due to buffer queue overflow in simulation.
	static int64_t gDataBitsSent; ///< how many bits sent by the sender.
	static int64_t gDataBitsRecv; ///< how many bits received by the destination.
	static long gDataPktsSent; ///< how many packets sent by the sender.
	static long gDataPktsRecv; ///< how many packets received by the destination.
	static long gDataPktsDuplicated; ///< how many duplicated packets received by the destination.
	static int64_t gDataBitsTransmitted; ///< this can be thought of as a measure of the bit efficiency of delivering data within the network.
	static int64_t gCtrlBitsTransmitted; ///< this measures the bit efficiency of the protocol in expending control overhead to delivery data.
	static long gDataPktsTransmitted; ///< rather than measuring pure algorithmic efficiency in terms of bit count, this measure tries to capture
	static long gCtrlPktsTransmitted; ///< a protocol's channel access efficiency, as the cost of channel access is high in contention-based link layers.
	static simtime_t gRouteAcquisitionTime; ///< accumulative total of routing delays of all the vehicles.
	static simtime_t gEndtoendDelay; ///< accumulative total of end-to-end delays of data packets.
};

#endif /* __ROUTINGSTATISTICCOLLECTOR_H__ */
