/*
 * Target.h
 *
 *  Created on: Mar 1, 2017
 *      Author: Michael
 */

#pragma once
#include <Brahe.h>
#include <WPILib.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>
#include <GripPipeline.h>
#define D_FOV 75;
#define H_FOV 67;
#define V_FOV 41;
class Target {
public:
	typedef enum trackState { SEARCHING, AQUIRED, TRACKING } trackingState_t ;
	Target();
	void processMat(cv::Mat input);
	cv::Mat outputMat();
	int getR1X();
	int getR1Y();
	int getR2X();
	int getR2Y();
	bool isAquired();
	bool isTracked();
	bool isLooking();
	double targetDistance();
	double targetAngle();

private:
	trackingState_t m_state;
	cv::Mat source0;
	grip::GripPipeline gp;
	cv::Rect r1, r2;
	double m_distance;
	double m_angle;
	double m_frame[4];
};
