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

// Width of one of the 2 rectangles of reflective tape
#define TARGET_WIDTH 2
// Width between outer edges of 2 rectangles of reflective tape
#define FULL_TARGET_WIDTH 10.25
// Field of View angular dimensions
#define FOV_V 49.5
#define FOV_H 63.1
// Resolution dimensions in pixels
#define RESOLUTION_X 320
#define RESOLUTION_Y 240
// Height to Width ratio has to be less than this percentage different from real ration (5 to 2) to be a match
#define HW_RATIO_TOLERANCE 0.2
// Actual height to width ratio of rectangle of reflective tape
#define TARGET_HW_RATIO 2.5
// Areas of smaller target must be less than this percentage different from larger taret to be a match
#define AREA_TOLERANCE 0.3
// Height of camera from the ground in inches
#define CAMERA_HEIGHT 13.5
// Height of top of target from the ground
#define TOP_OF_TARGET_HEIGHT 15.75


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
	cs::UsbCamera *m_usbCamera0, *m_usbCamera1;
	cs::CvSink *m_cvSink;
	cs::MjpegServer *m_mjpegServer;
	cs::CvSource m_cvSource;
	cv::Rect m_r1, m_r2;
	cv::Mat m_source;
	trackingState_t m_state;
	cv::Rect m_nullR;
	cv::Point m_targetCtrPt;
	cs::VideoSource m_nullV;
	int m_resX, m_resY, m_target_width;
	double m_fovV, m_fovH;
	double m_distance;
	double m_angle;
	double m_frame[4];
	bool m_isCam0;
	grip::GripPipeline m_gp;
};
