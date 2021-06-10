#ifndef IMU_UTILS_H_
#define IMU_UTILS_H_

#include <iostream>
#include <pthread.h>
#include <armadillo>

#include "Mat_Utils.h"

using namespace arma;

class IMU_Utils
{
public:

	static int findPeaks(vec &signal, mat& peakInds);
	static double calculateCompassBearing(double a0,double a1,double a2,double m0,double m1,double m2);
	static vec filter(mat &type, vec& toFilt);
	
	static int calculateOdometry(mat &shsignal, mat &shposition, mat &lastpositions, double ax_ref, double ay_ref, double az_ref, bool isbody, pthread_mutex_t &mutex);
};

#endif /* IMU_UTILS_H_ */
