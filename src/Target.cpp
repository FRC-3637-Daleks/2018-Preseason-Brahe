/*
 * Target.cpp
 *
 *  Created on: Mar 1, 2017
 *      Author: Michael
 */
#include <Target.h>
#include <math.h>

using namespace cv;

Target::Target()
{
	m_state = SEARCHING;
	m_distance = 0.0;
	m_angle = 0.0;
}
void Target::processMat(cv::Mat input){
	source0=input;
	std::vector<std::vector<cv::Point>> *dContours;
	gp.process(source0);
	dContours = gp.getfindContoursOutput();


	if (dContours->size()>1){
		unsigned int i = 0;
		while (i<dContours->size()){

			if (i%2==0){
				r1 = boundingRect(dContours->at(i));
				cv::rectangle(source0, r1, cv::Scalar(225,0,0), 1, 8, 0);
			}
			if (i%2==1){
				r2 = boundingRect(dContours->at(i));
				cv::rectangle(source0, r2, cv::Scalar(225,0,0), 1, 8, 0);
			}
			i++;
		}
	}
	else if(dContours->size()>0){
		r1 = boundingRect(dContours->at(0));
		cv::rectangle(source0, r1, cv::Scalar(225,0,0), 1, 8, 0);

	}
}
cv::Mat Target::outputMat(){
	std::vector<std::vector<cv::Point>> *dContours;
	cv::Mat nullM;
	gp.process(source0);
	dContours = gp.getfindContoursOutput();


	if (dContours->size()>1){
		unsigned int i = 0;
		while (i<dContours->size()){

			if (i%2==0){
				r1 = boundingRect(dContours->at(i));
				cv::rectangle(source0, r1, cv::Scalar(225,0,0), 1, 8, 0);
			}
			if (i%2==1){
				r2 = boundingRect(dContours->at(i));
				cv::rectangle(source0, r2, cv::Scalar(225,0,0), 1, 8, 0);
			}
			i++;
		}

	}
	else if(dContours->size()>0){
		r1 = boundingRect(dContours->at(0));
		cv::rectangle(source0, r1, cv::Scalar(225,0,0), 1, 8, 0);

	}

	frc::SmartDashboard::PutNumber("Contours", dContours->size());
	return source0;
}

int Target::getR1X(){
	return r1.br().x-(r1.width/2);
}
int Target::getR1Y(){
	return r1.br().y-(r1.height/2);
}
int Target::getR2X(){
	return r2.br().x-(r2.width/2);
}
int Target::getR2Y(){
	return r2.br().y-(r2.height/2);
}

