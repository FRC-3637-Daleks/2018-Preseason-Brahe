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
	usbCamera0->SetExposureManual(.5);
	if(usbCamera0) {
		mjpegServer0 = new cs::MjpegServer("serve_USB Camera 0", 1181);
		cvSink0      = new cs::CvSink("opencv_USB Camera 0");
		mjpegServer0->SetSource(*usbCamera0);
		cvSink0->SetSource(*usbCamera0);
		isCam0 = true;
	}
	usbCamera1 = new cs::UsbCamera("USB Camera 1", 1);
	cvSource0  = CameraServer::GetInstance()->PutVideo("Output", 320, 240);

}
void Target::process(){
	std::vector<std::vector<cv::Point>> *dContours;
	if (cvSink0->GrabFrame(source0)){
		if(isCam0){
			gp.process(source0);
			dContours = gp.getfindContoursOutput();

			if (dContours->size()>1){
				unsigned int i = 0;
				int x1 = 0;
				while (i<dContours->size()){
					if ((std::abs((boundingRect(dContours->at(i)).height/boundingRect(dContours->at(i)).width)-(2.5)))/2.5 <= .2 ){
						if(boundingRect(dContours->at(i)).br().x-(boundingRect(dContours->at(i)).width/2) > x1){
							r1 = boundingRect(dContours->at(i));

							x1 = boundingRect(dContours->at(i)).br().x-(boundingRect(dContours->at(i)).width/2);
						}
						else{
							r2 = boundingRect(dContours->at(i));
						}
					}
					i++;
				}
				cv::rectangle(source0, r1, cv::Scalar(225,0,0), 5, 8, 0);
				cv::rectangle(source0, r2, cv::Scalar(225,0,0), 5, 8, 0);

			}
			else if(dContours->size()>0){
				r1 = boundingRect(dContours->at(0));
				r2 = nullR;
				cv::rectangle(source0, r1, cv::Scalar(225,0,0), 1, 8, 0);

			}
			frc::SmartDashboard::PutNumber("Contours", dContours->size());
		}
		cvSource0.PutFrame(source0);
	}
}

void Target::switchCam0(){
	mjpegServer0->SetSource(nullV);
	cvSink0->SetSource(nullV);
	mjpegServer0->SetSource(*usbCamera0);
	cvSink0->SetSource(*usbCamera0);
	isCam0 = true;
}
void Target::switchCam1(){
	if(usbCamera1) {
		mjpegServer0->SetSource(nullV);
		cvSink0->SetSource(nullV);
		mjpegServer0->SetSource(*usbCamera1);
		cvSink0->SetSource(*usbCamera1);
	}
	isCam0 = false;
}
double Target::targetAngle(){
	return acos((5 * r1.width) / (2 * r1.height));
}

double Target::targetDistance(){

	return ((5 * TARGET_WIDTH * RESOLUTION_X) / (4 * r1.width * tan(FOV_H / 2)));
}

cv::Rect Target::getR1(){
	return r1;
}

cv::Rect Target::getR2(){
	return r2;
}
