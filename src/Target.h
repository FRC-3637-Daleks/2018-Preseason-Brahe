/*
 * Target.h
 *
 *  Created on: Mar 1, 2017
 *      Author: Michael
 */
#pragma once

#include <Brahe.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <GripPipeline.h>

class Target {
public:

	typedef enum trackState { SEARCHING, AQUIRED, TRACKING } trackingState_t ;
	Target(int cam0, int cam1);
	void process();
	bool isAquired();
	bool isTracked();
	bool isLooking();
	double targetDistance();
	double targetAngle();

private:
	cs::UsbCamera usbCamera0, usbCamera1;
	cv::Rect r1, r2;
	trackingState_t m_state;
	int RESOLUTION_X, RESOLUTION_Y, TARGET_WIDTH;
	double FOV_H, FOV_V;
	double m_distance;
	double m_angle;
	double m_frame[4];
	grip::GripPipeline gp;
};
