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
	cs::VideoSink server;

    cs::UsbCamera camera0 = CameraServer::GetInstance()->StartAutomaticCapture(myt->m_cam0);
    cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture(myt->m_cam1);
    camera0.SetResolution(320, 480);
    camera1.SetResolution(320, 480);

    server = CameraServer::GetInstance()->GetServer();
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
    cs::CvSource outputStreamRear  = CameraServer::GetInstance()->PutVideo("Rear", 320, 480);
    cs::CvSource outputStreamFront = CameraServer::GetInstance()->PutVideo("Front", 320, 480);

    while(true) {
    	switch(myt->m_feed) {
    	case FRONT_CAMERA:
    		server.SetSource(camera0);
    		cvSink = CameraServer::GetInstance()->GetVideo();
    		break;
    	case REAR_CAMERA:
    		server.SetSource(camera1);
    		cvSink = CameraServer::GetInstance()->GetVideo();
    		break;
    	default:
    		myt->m_feed = FRONT_CAMERA;
    		server.SetSource(camera0);
    		cvSink = CameraServer::GetInstance()->GetVideo();
    		break;
    	}
    	// the grab should self regulate this thread, as it will timeout if
    	// no frame is available.
		if(cvSink.GrabFrame(myt->m_source)) {
			if(myt->m_source.empty())
				continue;
			frc::SmartDashboard::PutNumber("frames seen", cnt++);
			if(myt->m_feed == FRONT_CAMERA) {
				myt->processFrame();
				outputStreamFront.PutFrame(myt->m_source);
			}
			else {
				outputStreamRear.PutFrame(myt->m_source);
			}
    	}
    }
}

void
Target::processFrame()
{
	// TBD: need to add state changes based on what we see
	// we start in SEARCHING, change to AQUIRED if we think we may have found it
	// then finally TARGETING for when we can trust the values.  If we lose the
	// target we change the state back to SEARCHING.
	std::vector<std::vector<cv::Point>> *dContours;
	std::vector<cv::Rect> candRects;
	cv::Rect r;
	float ratio_percent;
<<<<<<< HEAD
	float mid1, mid2;
	bool match_found = false;
=======
>>>>>>> branch 'master' of https://github.com/FRC-3637-Daleks/Brahe.git

	m_gp.process(m_source);
	dContours = m_gp.getfindContoursOutput();
	m_r1 = m_nullR;
	m_r2 = m_nullR;
	for (unsigned int i = 0; i < dContours->size(); i++) {
		r = boundingRect(dContours->at(i));
		ratio_percent = fabs(r.height/r.width - TARGET_HW_RATIO)/TARGET_HW_RATIO;
		if(ratio_percent <= HW_RATIO_TOLERANCE) {
			std::vector<cv::Rect>::iterator it = candRects.begin();
			while ((it != candRects.end()) && (r.area() <= it->area()))
				it++;
			candRects.insert(it, r);
		}
	}
	if(candRects.size() == 1) {
		// only found 1 candidate assume it is one of the rectangles of the target
		m_r1 = candRects.at(0);
		m_r2 = m_r1;
		// Since only 1 rectangle, don't use full target width
		m_target_width = TARGET_WIDTH;
	}
	else if(candRects.size() == 2) {
		// All is good, found 2 candidates, yeah!
		//FIXME: should put some code here to make sure roughly same size/parallel
		m_r1 = candRects.at(0);
		m_r2 = candRects.at(1);
	}
	else if(candRects.size() > 2) {
		// look for 2 that are roughly same size & level
		//FIXME: add code to pick 2 best rectangles
		// For now, just use the first 2 which are the biggest ones
		m_r1 = candRects.at(0);
		m_r2 = candRects.at(1);
	}
	if(m_r1.height != 0) {
		//FIXME: for debugging only, remove later
		cv::rectangle(m_source, m_r1, cv::Scalar(225,0,0), 5, 8, 0);
		cv::rectangle(m_source, m_r2, cv::Scalar(225,0,0), 5, 8, 0);
		if(m_r1.tl().x < m_r2.tl().x)
			m_targetCtrPt.x = (m_r1.tl().x + m_r2.br().x) / 2;
		else
			m_targetCtrPt.x = (m_r2.tl().x + m_r1.br().x) / 2;
		if(m_r1.height > m_r2.height)
			m_targetCtrPt.y = m_r1.tl().y + .5 * m_r1.height;
		else
			m_targetCtrPt.y = m_r2.tl().y + .5 * m_r2.height;
	}

	frc::SmartDashboard::PutNumber("Contours", dContours->size());
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
			if(m_r1.height != 0)
				//return acos((5 * m_r1.width) / (2 * m_r1.height));
				// Find ratio of distance from target center to FOV center to total FOV and then multiply by FOV in degrees
				// to get angle in degrees
				m_angle = ((m_targetCtrPt.x - .5 * RESOLUTION_X) / RESOLUTION_X) * FOV_H;
				return m_angle;
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
			opp_side = TOP_OF_TARGET_HEIGHT - CAMERA_HEIGHT;
			// We can get the angle from the robot's camera to the top of the target by finding the distance in pixels from the
			// top of the target to the center of the FOV, dividing that by the entire FOV in pixels to get the ratio and then
			// multiply by the vertical FOV in degrees.
			ratio = RESOLUTION_Y/2 -  m_r1.tl().y;
			angle = ratio * FOV_V;
			m_distance = opp_side / tan(angle);
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
