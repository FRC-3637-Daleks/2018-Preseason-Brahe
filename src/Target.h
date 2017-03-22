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

#define TARGET_WIDTH 2
#define FOV_V 0
#define FOV_H 0
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

class Target {
public:
	typedef enum trackState { SEARCHING, AQUIRED, TRACKING } trackingState_t ;
	Target(int cam0, int cam1);
	void processFrame();
	void switchCam(enum Cameras cam);
	bool isAquired();
	bool isTracked();
	bool isLooking();
	double targetDistance();
	double targetAngle();
	cv::Rect getR1();
	cv::Rect getR2();

private:
	cv::Rect m_r1, m_r2;
	cv::Rect m_nullR;
	cv::Mat m_source;
	trackingState_t m_state;
	int m_resX, m_resY, m_target_width;
	int m_cam0, m_cam1;
	double m_fovV, m_fovH;
	double m_distance;
	double m_angle;
	double m_frame[4];
	enum Cameras m_feed;
	static void visionThread(void *t);
	grip::GripPipeline m_gp;
};
