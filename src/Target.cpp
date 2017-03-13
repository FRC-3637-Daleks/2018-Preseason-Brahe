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
	m_target_width = TARGET_WIDTH;
	m_fovH = FOV_H;
	m_fovV = FOV_V;
	m_resX = RESOLUTION_X;
	m_resY = RESOLUTION_Y;

	m_usbCamera0  = new cs::UsbCamera("USB Camera 0", 0);
	m_usbCamera0->SetExposureManual(.5);
	if(m_usbCamera0) {
		m_mjpegServer = new cs::MjpegServer("serve_USB Camera 0", 1181);
		m_cvSink      = new cs::CvSink("opencv_USB Camera 0");
		m_mjpegServer->SetSource(*m_usbCamera0);
		m_cvSink->SetSource(*m_usbCamera0);
		m_isCam0 = true;
	}
	m_usbCamera1 = new cs::UsbCamera("USB Camera 1", 1);
	m_cvSource  = CameraServer::GetInstance()->PutVideo("Output", 320, 240);
}

void
Target::processFrame()
{
	std::vector<std::vector<cv::Point>> *dContours;
	cv::Mat source;
	cv::Rect r;
	float ratio;
	int x, max;

	// if not on camera 0, then skip frame processing
	if(!m_isCam0)
		return;

	if(m_cvSink->GrabFrame(source)) {
		m_gp.process(source);
		max = 0;
		dContours = m_gp.getfindContoursOutput();
		m_r1 = m_nullR;
		m_r2 = m_nullR;
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
			cv::rectangle(source, m_r1, cv::Scalar(225,0,0), 5, 8, 0);
			cv::rectangle(source, m_r2, cv::Scalar(225,0,0), 5, 8, 0);

		}
		else if(dContours->size() == 1) {
			m_r1 = boundingRect(dContours->at(0));
			cv::rectangle(source, m_r1, cv::Scalar(225,0,0), 1, 8, 0);
		}
		frc::SmartDashboard::PutNumber("Contours", dContours->size());
		m_cvSource.PutFrame(source);
	}
	return;
}

void
Target::switchCam(enum Cameras cam)
{
	switch(cam) {
		case FRONT_CAMERA:
			if(m_usbCamera0) {
				m_mjpegServer->SetSource(m_nullV);
				m_cvSink->SetSource(m_nullV);
				m_mjpegServer->SetSource(*m_usbCamera0);
				m_cvSink->SetSource(*m_usbCamera0);
				m_isCam0 = true;
			}
			break;
		case REAR_CAMERA:
			if(m_usbCamera1) {
				m_mjpegServer->SetSource(m_nullV);
				m_cvSink->SetSource(m_nullV);
				m_mjpegServer->SetSource(*m_usbCamera1);
				m_cvSink->SetSource(*m_usbCamera1);
				m_isCam0 = false;
			}
			break;
		default:
			break;
	}
}

double
Target::targetAngle()
{
	if(m_r1.height != 0)
		return acos((5 * m_r1.width) / (2 * m_r1.height));
	return 0.0;
}

double
Target::targetDistance()
{
	return ((5 * m_target_width * m_resX) / (4 * m_r1.width * tan(m_fovH / 2)));
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
