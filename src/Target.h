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
	void switchCam0();
	void switchCam1();
	bool isAquired();
	bool isTracked();
	bool isLooking();
	double targetDistance();
	double targetAngle();
	cv::Rect getR1();
	cv::Rect getR2();

private:
	cs::UsbCamera *usbCamera0, *usbCamera1;
	cs::CvSink *cvSink0;
	cs::MjpegServer *mjpegServer0;
	cs::CvSource cvSource0;
	cv::Rect r1, r2;
	cv::Mat source0;
	trackingState_t m_state;
	cv::Rect nullR;
	cs::VideoSource nullV;
	int RESOLUTION_X, RESOLUTION_Y, TARGET_WIDTH;
	double FOV_H, FOV_V;
	double m_distance;
	double m_angle;
	double m_frame[4];
	bool isCam0;
	grip::GripPipeline gp;
};
