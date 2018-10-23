/*
 * IRsensor.h
 *
 *  Created on: Mar 26, 2017
 *      Author: Michael
 */
/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <Brahe.h>

class IRsensor
{
public:
	enum monitorState { MONITORING=0, STOP, NUM_STATES };
	IRsensor(int left, int right);
	IRsensor(AnalogInput *left, AnalogInput *right);
	IRsensor(AnalogInput &left, AnalogInput &right);
	~IRsensor();
	double GetAverageDistance();
	double GetLeftDistance();
	double GetRightDistance();
	double convertVoltage(double v);

private:
	monitorState m_state;
	AnalogInput *m_left;
	AnalogInput *m_right;
	bool m_needFree;
	double m_leftDistance;
	double m_rightDistance;

	void sensorInit();
	static void monitorDistance(void *ir);
};
