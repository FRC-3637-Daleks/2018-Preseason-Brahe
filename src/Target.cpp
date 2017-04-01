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
	m_state        = SEARCHING;
	m_distance     = 0.0;
	m_angle        = 0.0;
	m_target_width = FULL_TARGET_WIDTH;
	m_fovH         = FOV_H;
	m_fovV         = FOV_V;
	m_resX         = RESOLUTION_X;
	m_resY         = RESOLUTION_Y;
	m_cam0		   = cam0;
	m_cam1		   = cam1;
	m_feed         = FRONT_CAMERA;
	std::thread m_eyes(visionThread, (void *)this);
	m_eyes.detach();

	return;
}

void
Target::visionThread(void *t)
{
	Target *myt = (Target *)t;
	static long long cnt;
	cv::Mat source;
	cs::VideoSink server;
	cs::CvSink cvSink;

    cs::UsbCamera camera0 = CameraServer::GetInstance()->StartAutomaticCapture(myt->m_cam0);
    cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture(myt->m_cam1);

    camera0.SetResolution(640, 480);
    camera0.SetBrightness(45);
    camera0.SetExposureAuto();
    camera1.SetResolution(640, 480);
    camera1.SetBrightness(45);
    camera1.SetExposureAuto();

	server = CameraServer::GetInstance()->GetServer();
	cvSink = CameraServer::GetInstance()->GetVideo();
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Processed", 640, 480);

    while(true) {
		switch(myt->m_feed) {
			case FRONT_CAMERA:
				server.SetSource(camera0);
				cvSink.SetSource(camera0);
				break;
			case REAR_CAMERA:
				server.SetSource(camera1);
				cvSink.SetSource(camera1);
				break;
			default:
				myt->m_feed = FRONT_CAMERA;
				server.SetSource(camera0);
				cvSink.SetSource(camera0);
				break;
    	}
		// the grab should self regulate this thread, as it will timeout if
		// no frame is available.
		if(cvSink.GrabFrame(source)) {
			if(source.empty())
				continue;
			frc::SmartDashboard::PutNumber("frames seen", cnt++);
			if(myt->m_feed == FRONT_CAMERA) {
				myt->processFrame(source);
				outputStream.PutFrame(source);
			}
			else
				outputStream.PutFrame(source);
		}
	}
}

void
Target::processFrame(cv::Mat& src)
{
	std::vector<std::vector<cv::Point>> *dContours;
	std::vector<cv::Rect> candRects;
	cv::Rect r;
	float ratio_percent;
	float mid1, mid2;
	bool match_found = false;

	m_gp.Process(src);
	dContours = m_gp.GetFindContoursOutput();
	frc::SmartDashboard::PutNumber("Contours", dContours->size());
	m_r1 = m_nullR;
	m_r2 = m_nullR;
	// Find contours whose bounding rectangle matches the h-w ratio of the reflective tape
	// Add these rectangles to vector in descending order of area size
	for (unsigned int i = 0; i < dContours->size(); i++) {
		r = boundingRect(dContours->at(i));
		ratio_percent = fabs(((float)r.height/(float)r.width) - TARGET_HW_RATIO)/TARGET_HW_RATIO;
		frc::SmartDashboard::PutNumber("Ratio", ratio_percent);
		if(ratio_percent <= HW_RATIO_TOLERANCE) {
			std::vector<cv::Rect>::iterator it = candRects.begin();
			while ((it != candRects.end()) && (r.area() <= it->area()))
				it++;
			candRects.insert(it, r);
		}
	}
	frc::SmartDashboard::PutNumber("CandRects", candRects.size());
	if(candRects.size() >= 1) {
		// We have found at least one part of the target so set our state to indicate that
		if (m_state == SEARCHING)
			m_state = AQUIRED;
		else
			m_state = TRACKING;
		// look for 2 rectangles that are roughly same size & level & choose them as target components
		for (std::vector<cv::Rect>::iterator it1 = candRects.begin() ; (!match_found && it1 != candRects.end()); ++it1) {
			for (std::vector<cv::Rect>::iterator it2 = std::next(it1,1) ; (!match_found && it2 != candRects.end()); ++it2) {
				if (it1->area() / it2->area() - 1 < AREA_TOLERANCE) {
					mid1 = (float)(it1->tl().y) + (0.5 * (float)(it1->height));
					mid2 = (float)(it2->tl().y) + (0.5 * (float)(it2->height));
					if (fabs(mid1 / mid2 - 1) < TARGET_CENTER_HEIGHT_TOLERANCE) {
						match_found = true;
						m_r1 = *it1;
						m_r2 = *it2;
					}
				}
			}
		}
		if (!match_found) {
			// We didn't find 2 rectangles roughly same size and level, 
			// so just use the first (largest) one and hope!
			m_r1 = candRects.at(0);
			m_r2 = m_r1;
			// Since only 1 rectangle, don't use full target width
			m_target_width = TARGET_WIDTH;
		}
	} 
	else
		m_state = SEARCHING;
	
	if(m_r1.height != 0) {
		//FIXME: for debugging only, remove later
		cv::rectangle(src, m_r1, cv::Scalar(225,0,0), 5, 8, 0);
		cv::rectangle(src, m_r2, cv::Scalar(225,0,0), 5, 8, 0);
		// Set center point of composite target
		if(m_r1.tl().x < m_r2.tl().x)
			m_targetCtrPt.x = (m_r1.tl().x + m_r2.br().x) / 2;
		else
			m_targetCtrPt.x = (m_r2.tl().x + m_r1.br().x) / 2;
		if(m_r1.height > m_r2.height)
			m_targetCtrPt.y = (int)((float)(m_r1.tl().y) + (0.5 * (float)(m_r1.height)));
		else
			m_targetCtrPt.y = (int)((float)(m_r2.tl().y) + (0.5 * (float)(m_r2.height)));
	}
	return;
}

void
Target::switchCam(enum Cameras cam)
{
	m_feed = cam;
	return;
}

double
Target::targetAngle()
{
	switch(m_state) {
		case SEARCHING:
		case AQUIRED:
			return 0.0;
		case TRACKING:
			if(m_r1.height != 0) {
				//return acos((5 * m_r1.width) / (2 * m_r1.height));
				// Find ratio of distance from target center to FOV center to total FOV and then multiply by FOV in degrees
				// to get angle in degrees
				m_angle = (((float)m_targetCtrPt.x - (0.5 * (float)RESOLUTION_X)) / (float)RESOLUTION_X) * (float)FOV_H;
				return m_angle;
			}
			return 0.0;
		default:
			break;
	}
	return 0.0;
}

double
Target::targetDistance()
{
	double opp_side, ratio, angle;
	switch(m_state) {
		case SEARCHING:
		case AQUIRED:
			return 0.0;
		case TRACKING:
			// return ((5 * m_target_width * m_resX) / (4 * m_r1.width * tan(m_fovH / 2)));
			// Height of top of target - height of camera gives us the opposite side of a right triangle
			opp_side = fabs(TOP_OF_TARGET_HEIGHT - CAMERA_HEIGHT);
			// We can get the angle from the robot's camera to the top of the target by finding the distance in pixels from the
			// top of the target to the center of the FOV, dividing that by the entire FOV in pixels to get the ratio and then
			// multiply by the vertical FOV in degrees.
			ratio = ((float)RESOLUTION_Y/2.0 -  (float)(m_r1.tl().y))/(float)RESOLUTION_Y;
			angle = ratio * (float)FOV_V;
			frc::SmartDashboard::PutNumber("Dangle", angle);
			m_distance = opp_side / tan((angle/180.0) * 3.1415926);
			return m_distance;
		default:
			break;
	}
	return 0.0;
}

cv::Rect
Target::getR1()
{
	return m_r1;
}

cv::Rect
Target::getR2()
{
	return m_r2;
}

bool
Target::isAquired()
{
	return (m_state == AQUIRED);
}

bool
Target::isLooking()
{
	return (m_state == SEARCHING);
}

bool
Target::isTracked()
{
	return (m_state == TRACKING);
}
