/*
 * Climber.cpp
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */

#include <Climber.h>
#include <CANTalon.h>

using namespace frc;

Climber::Climber(int climbMotor, int piston)
{
		m_climb = new CANTalon(climbMotor);
		m_piston = new Solenoid(piston);
		m_needFree = true;
		m_piston->Set(false);
}

Climber::Climber(CANTalon *climbMotor, Solenoid *piston)
{
		m_climb = climbMotor;
		m_piston = piston;
		m_needFree = false;
		m_piston->Set(false);
}

Climber::Climber(CANTalon &climbMotor, Solenoid &piston)
{
		m_climb = &climbMotor;
		m_piston = &piston;
		m_needFree = false;
		m_piston->Set(false);
}

Climber::~Climber()
{
	if(m_needFree) {
		delete m_climb;
		delete m_piston;
	}
	return;
}

void
Climber::SwitchOn()
{
	m_piston->Set(true);
}

void
Climber::SwitchOff()
{
	m_piston->Set(false);
}

void
Climber::Up(double speed)
{
	m_climb->Set(speed);
}

void
Climber::Stop()
{
	m_climb->Set(0);
}




