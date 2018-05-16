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

#ifndef __DV_CAST_H__
#define __DV_CAST_H__

#include "veins/modules/wave/BaseWaveApplLayer.h"
#include "veins/modules/routing/BroadcastRoutingInterface.h"

#define USE_WEIGHTED_P_PERSISTENCE    1
#define USE_SLOTTED_1_PERSISTENCE     0
#define USE_SLOTTED_P_PERSISTENCE     0

#define NOTIFY_WHEN_ARRIVED_AT_THE_FARTHEST_ONE    0

/**
 * @brief A concrete routing protocol.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseWaveApplLayer
 * @see BroadcastRoutingInterface
 * reference: DV-CAST: A DISTRIBUTED VEHICULAR BROADCAST PROTOCOL FOR VEHICULAR AD HOC NETWORKS, IEEE Wireless Communications, April 2010
 */
class DV_CAST : public BaseWaveApplLayer, private BroadcastRoutingInterface
{
public:
	/** @name constructors, destructor. */
	///@{
	DV_CAST() : BaseWaveApplLayer() {}
	DV_CAST(unsigned stacksize) : BaseWaveApplLayer(stacksize) {}
	~DV_CAST();
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;

	/** @brief wave short message decorate method. */
	void decorateWSM(WaveShortMessage *wsm) override;
	/** @brief call-back method of receiving beacon message. */
	void onBeacon(BeaconMessage *wsm) override;
	/** @brief call-back method of receiving routing message. */
	void onRouting(RoutingMessage *wsm) override;
	/** @brief call-back method of receiving warning message. */
	void onWarning(WarningMessage *wsm) override;
	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *wsm) override;
	/** @brief examine whether neighbors still in connected. */
	void examineNeighbors() override;

	/** @brief call a warning notify to a certain direction determined by warningPlanList. */
	void callWarning(double distance) override;

	/** @name interfaces implementation. */
	///@{
	/** @brief adopt broadcast suppression method to avoid broadcast storm. */
	void broadcastSuppression(WarningMessage *warningMessage) override;
	/** @brief rebroadcast routing request message. */
	void rebroadcast(WarningMessage *warningMessage) override;
	///@}

private:
	/** @brief Vehicle's routing state. */
	enum State {
		IDLE,
		BroadcastSuppression1,
		BroadcastSuppression2,
		Rebroadcast,
		WAIT_I,
		WAIT_II
	};

private:
	bool MDC;  ///< refer to routing protocol DV-CAST.
	bool ODC;  ///< refer to routing protocol DV-CAST.

	simtime_t WAIT_TIME; ///< the waiting period before rebroadcast.
	std::map<simtime_t, WaitTimeElapsedMessage*> waitTimeElapsedEvts; ///< how long suppress not to rebroadcast.

	std::set<LAddress::L3Type> NB_FRONT;    ///< front neighbors.
	std::set<LAddress::L3Type> NB_BACK;     ///< back neighbors.
	std::set<LAddress::L3Type> NB_OPPOSITE; ///< opposite neighbors.
	std::map<int /* GUID */, int> curState; ///< a certain packet's routing state.
	std::map<int /* GUID */, bool> DFlg; ///< refer to routing protocol DV-CAST.
};

#endif /* __DV_CAST_H__ */
