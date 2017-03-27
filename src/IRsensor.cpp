/*
 * IRsensor.cpp
 *
 *  Created on: Mar 26, 2017
 *      Author: Michael
 */

#include <IRsensor.h>

using namespace frc;

IRsensor::IRsensor(int left, int right)
{
	m_left          = new AnalogInput(left);
	m_right         = new AnalogInput(right);
	m_needFree      = true;
	m_leftDistance  = 0.0;
	m_rightDistance = 0.0;
	m_state         = MONITORING;
	sensorInit();
}

IRsensor::IRsensor(AnalogInput *left, AnalogInput *right)
{
	m_left          = left;
	m_right         = right;
	m_needFree      = false;
	m_leftDistance  = 0.0;
	m_rightDistance = 0.0;
	m_state         = MONITORING;
	sensorInit();
}

IRsensor::IRsensor(AnalogInput &left, AnalogInput &right)
{
	m_left          = &left;
	m_right         = &right;
	m_needFree      = false;
	m_leftDistance  = 0.0;
	m_rightDistance = 0.0;
	m_state         = MONITORING;
	sensorInit();
}

IRsensor::~IRsensor()
{
	m_state = STOP;
	if(m_needFree) {
		delete m_left;
		delete m_right;
	}
	return;
}

void
IRsensor::sensorInit()
{
	lw->AddSensor("IR Sensors", "Left sensor", m_left);
	lw->AddSensor("IR Sensors", "Right sensor", m_left);


	std::thread m_monitor(monitorDistance, (void *)this);
	m_monitor.detach();
	return;
}

double
IRsensor::convertVoltage(double v)
{
	// need to verify formula
	if(v <= 1.77)
		return 1.0/((v - .65)/(13.21208267) + .05) - .42;
	if(v <= 2.34)
		return 1.0/((v - 2.34)/(8.670916) + .1845) - .42;

	return 1.0/((v - 2.34)/(9.341717+.1845)) - .42;
}

void
IRsensor::monitorDistance(void *ir)
{
	IRsensor *myir = (IRsensor *)ir;

	while(true) {
		switch(myir->m_state) {
			case IRsensor::MONITORING:
				double leftVoltage;
				double rightVoltage;

				leftVoltage = myir->m_left->GetVoltage();
				rightVoltage = myir->m_right->GetVoltage();

				myir->m_leftDistance = myir->convertVoltage(leftVoltage);
				myir->m_rightDistance = myir->convertVoltage(rightVoltage);
				break;
			case IRsensor::STOP:
				return;
			default:
				break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
}

double
IRsensor::GetAverageDistance()
{
	return (m_leftDistance+m_rightDistance)/2.0;
}

double
IRsensor::GetLeftDistance()
{
	return m_leftDistance;
}

double
IRsensor::GetRightDistance()
{
	return m_rightDistance;
}
