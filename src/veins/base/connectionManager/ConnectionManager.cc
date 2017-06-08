#include <cmath>
#include "veins/base/connectionManager/ConnectionManager.h"
#include "veins/base/modules/BaseWorldUtility.h"

Define_Module(ConnectionManager);

double ConnectionManager::calcInterfDist()
{
	// the minimum carrier frequency for this cell
	double carrierFrequency = par("carrierFrequency").doubleValue();
	// maximum transmission power possible
	double pMax = par("pMax").doubleValue();
	if (pMax <= 0)
		error("Max transmission power is <=0!");
	transmitPower_dBm = 10 * log10(pMax);

	// minimum signal attenuation threshold
	double sat = par("sat").doubleValue();
	// minimum path loss coefficient
	double alpha = par("alpha").doubleValue();

	double waveLength = (BaseWorldUtility::speedOfLight()/carrierFrequency);
	// minimum power level to be able to physically receive a signal
	double minReceivePower = pow(10.0, sat/10.0);

	double interfDistance = pow(waveLength*waveLength*pMax / (16.0*M_PI*M_PI*minReceivePower), 1.0 / alpha);

	EV << "max interference distance: " << interfDistance << "m.\n";

	return interfDistance;
}

