/*
 * Target.cpp
 *
 *  Created on: Mar 1, 2017
 *      Author: Michael
 */
#include <Target.h>
#include <math.h>
Target::Target(int cam0, int cam1)
{
	m_state = SEARCHING;
	m_distance = 0.0;
	m_angle = 0.0;
	TARGET_WIDTH = 2;
	FOV_H = 0;
	FOV_V = 0;
	RESOLUTION_X = 320;
	RESOLUTION_Y = 240;
	usbCamera0  = new cs::UsbCamera("USB Camera 0", 0);
	usbCamera1  = new cs::UsbCamera("USB Camera 1", 1);
}
void Target::process(){

}

double Target::targetAngle(){
	return acos((5 * r1.width) / (2 * r1.height));
}

double Target::targetDistance(){

	return ((5 * TARGET_WIDTH * RESOLUTION_X) / (4 * r1.width * tan(FOV_H / 2)));
}
