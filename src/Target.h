/*
 * Target.h
 *
 *  Created on: Mar 1, 2017
 *      Author: Michael
 */

#pragma once
#include <Brahe.h>

class Target {
public:
	typedef enum trackState { SEARCHING, AQUIRED, TRACKING } trackingState_t ;
	Target();
	bool isAquired();
	bool isTracked();
	bool isLooking();
	double targetDistance();
	double targetAngle();

private:
	trackingState_t m_state;
	double m_distance;
	double m_angle;
	double m_frame[4];
};
