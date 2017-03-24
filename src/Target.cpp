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
	m_target_width = TARGET_WIDTH;
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
    		break;
    	case REAR_CAMERA:
    		server.SetSource(camera1);
    		break;
    	default:
    		break;
    	}
    	// the grab should self regulate this thread, as it will timeout if
    	// no frame is available.
		if(cvSink.GrabFrame(myt->m_source)) {
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
	cv::Rect r;
	float ratio;
	int x, max;
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

	max = 0; m_r1 = m_nullR; m_r2 = m_nullR;
	m_gp.process(m_source);
	dContours = m_gp.getfindContoursOutput();
	if(dContours->size() > 1) {
		unsigned int i = 0;
		while(i < dContours->size()) {
			r = boundingRect(dContours->at(i));
			x = r.br().x - (r.width/2);
			ratio = r.height/r.width;
			ratio = (fabs(ratio) - 2.5)/2.5;
			if(ratio <= .2) {
				if(x > max) {
					m_r1 = r;
					max = x;
				}
				else
					m_r2 = r;
			}
			i++;
		}
		cv::rectangle(m_source, m_r1, cv::Scalar(225,0,0), 5, 8, 0);
		cv::rectangle(m_source, m_r2, cv::Scalar(225,0,0), 5, 8, 0);
	}
	else if(dContours->size() == 1) {
		m_r1 = boundingRect(dContours->at(0));
		cv::rectangle(m_source, m_r1, cv::Scalar(225,0,0), 1, 8, 0);
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
				return acos((5 * m_r1.width) / (2 * m_r1.height));
			return 0.0;
		default:
			break;
	}
	return 0.0;
}

double
Target::targetDistance()
{
	switch(m_state) {
		case SEARCHING:
		case AQUIRED:
			return 0.0;
		case TRACKING:
			return ((5 * m_target_width * m_resX) / (4 * m_r1.width * tan(m_fovH / 2)));
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
