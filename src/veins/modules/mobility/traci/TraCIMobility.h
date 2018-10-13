//
// Copyright (C) 2006-2012 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
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

#ifndef VEINS_MOBILITY_TRACI_TRACIMOBILITY_H
#define VEINS_MOBILITY_TRACI_TRACIMOBILITY_H

#define TRACI_SIGNAL_PARKING_CHANGE_NAME "parkingStateChanged"

#include <string>
#include <fstream>
#include <list>
#include <stdexcept>

#include "veins/base/modules/BaseMobility.h"
#include "veins/base/utils/FindModule.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

/**
 * @brief
 * Used in modules created by the TraCIScenarioManager.
 *
 * This module relies on the TraCIScenarioManager for state updates
 * and can not be used on its own.
 *
 * See the Veins website <a href="http://veins.car2x.org/"> for a tutorial, documentation, and publications </a>.
 *
 * @author Christoph Sommer, David Eckhoff, Luca Bedogni, Bastian Halmos, Stefan Joerer
 *
 * @see TraCIScenarioManager
 * @see TraCIScenarioManagerLaunchd
 *
 * @ingroup mobility
 */
namespace Veins {
class TraCIMobility : public BaseMobility
{
public:
	class Statistics
	{
		public:
			double firstRoadNumber; /**< for statistics: number of first road we encountered (if road id can be expressed as a number) */
			simtime_t startTime; /**< for statistics: start time */
			simtime_t totalTime; /**< for statistics: total time traveled */
			simtime_t stopTime; /**< for statistics: stop time */
			double minSpeed; /**< for statistics: minimum value of currentSpeed */
			double maxSpeed; /**< for statistics: maximum value of currentSpeed */
			double totalDistance; /**< for statistics: total distance traveled */
			double totalCO2Emission; /**< for statistics: total CO2 emission */

			void initialize();
			void watch(cSimpleModule& module);
			void recordScalars(cSimpleModule& module);
	};

	TraCIMobility() : BaseMobility(), isPreInitialized(false), manager(0), commandInterface(0), vehicleCommandInterface(0), laneCommandInterface(0) {}
	~TraCIMobility() { delete vehicleCommandInterface; }

	virtual void initialize(int) override;
	virtual void finish() override;

	virtual void updatePosition() override;

	void preInitialize(std::string external_id, const Coord& position, std::string road_id = "", double speed = -1, double angle = -1);
	void nextPosition(const Coord& position, std::string road_id = "", double speed = -1, double angle = -1, TraCIScenarioManager::VehicleSignal signals = TraCIScenarioManager::VEH_SIGNAL_UNDEF);
	void changeParkingState(bool);
	void updateDisplayString();

	double getAntennaPositionOffset() const { return antennaPositionOffset; }
	Coord getPositionAt(const simtime_t& t) const { return move.getPositionAt(t); }
	bool getParkingState() const { return isParking; }
	bool getAtIntersection() const { return atIntersection; }
	std::string getRoadId() const
	{
		if (road_id == "") throw cRuntimeError("TraCIMobility::getRoadId called with no road_id set yet");
		return road_id;
	}
    std::string getLaneId() const
    {
        if (lane_id == "") throw cRuntimeError("TraCIMobility::getLaneId called with no lane_id set yet");
        return lane_id;
    }
	double getSpeed() const
	{
		if (speed == -1) throw cRuntimeError("TraCIMobility::getSpeed called with no speed set yet");
		return speed;
	}
	TraCIScenarioManager::VehicleSignal getSignals() const
	{
		if (signals == -1) throw cRuntimeError("TraCIMobility::getSignals called with no signals set yet");
		return signals;
	}
	/**
	 * returns angle in rads, 0 being east, with -M_PI <= angle < M_PI.
	 */
	double getAngleRad() const { return angle; }
	TraCIScenarioManager* getManager() const
	{
		if (!manager)
			manager = TraCIScenarioManagerAccess().get();
		return manager;
	}
	TraCICommandInterface* getCommandInterface() const { return commandInterface; }
	TraCICommandInterface::Vehicle* getVehicleCommandInterface() const { return vehicleCommandInterface; }
    TraCICommandInterface::Lane* getLaneCommandInterface(std::string laneId) const
    {
        if (!laneCommandInterface)
            laneCommandInterface = new TraCICommandInterface::Lane(getCommandInterface()->lane(laneId));
        return laneCommandInterface;
    }

private:
	cOutVector currentPosXVec; /**< vector plotting posx */
	cOutVector currentPosYVec; /**< vector plotting posy */
	cOutVector currentSpeedVec; /**< vector plotting speed */
	cOutVector currentAccelerationVec; /**< vector plotting acceleration */
	cOutVector currentCO2EmissionVec; /**< vector plotting current CO2 emission */

	Statistics statistics; /**< everything statistics-related */

	bool isPreInitialized; /**< true if preInitialize() has been called immediately before initialize() */

	double antennaPositionOffset; /**< front offset for the antenna on this car */

	simtime_t lastUpdate; /**< updated by nextPosition() */
	Coord roadPosition; /**< position of front bumper, updated by nextPosition() */
	std::string road_id; /**< updated by nextPosition() */
	std::string lane_id; /**< updated by nextPosition() */
	double speed; /**< updated by nextPosition() */
	double angle; /**< updated by nextPosition() */
	TraCIScenarioManager::VehicleSignal signals; /**<updated by nextPosition() */

	mutable TraCIScenarioManager* manager;
	mutable TraCICommandInterface* commandInterface;
	mutable TraCICommandInterface::Vehicle* vehicleCommandInterface;
	mutable TraCICommandInterface::Lane* laneCommandInterface;
	double last_speed;

	const static simsignalwrap_t parkingStateChangedSignal;

	bool isParking;
	bool atIntersection;

private:
	void fixIfHostGetsOutside() override; /**< called after each read to check for (and handle) invalid positions. */

	/**
	 * Returns the amount of CO2 emissions in grams/second, calculated for an average Car.
	 * @param v speed in m/s
	 * @param a acceleration in m/s^2
	 * @returns emission in g/s
	 */
	double calculateCO2emission(double v, double a) const;

	/** Calculates where the antenna of this car is, given its front bumper position. */
	Coord calculateAntennaPosition(const Coord& vehiclePos) const;
};

class TraCIMobilityAccess
{
public:
	TraCIMobility* get(cModule* host) {
		TraCIMobility* traci = FindModule<TraCIMobility*>::findSubModule(host);
		ASSERT(traci);
		return traci;
	}
};
}

#endif
