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

