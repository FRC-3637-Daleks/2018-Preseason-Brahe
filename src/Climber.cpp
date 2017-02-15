/*
 * Climber.cpp
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */

#include <Climber.h>
#include <CANTalon.h>
#include <math.h>

using namespace frc;

Climber::Climber(int climbMotor, int piston, int magswitch)
{
		m_climb  = new CANTalon(climbMotor);
		m_piston = new Solenoid(piston);
		m_index  = new DigitalInput(magswitch);
		m_needFree = true;
		m_piston->Set(false);
}

Climber::Climber(CANTalon *climbMotor, Solenoid *piston, int magswitch)
{
		m_climb  = climbMotor;
		m_piston = piston;
		m_index  = new DigitalInput(magswitch);
		m_needFree = false;
		m_piston->Set(false);
}

Climber::Climber(CANTalon &climbMotor, Solenoid &piston, int magswitch)
{
		m_climb  = &climbMotor;
		m_piston = &piston;
		m_index  = new DigitalInput(magswitch);
		m_needFree = false;
		m_piston->Set(false);
}

Climber::~Climber()
{
	delete m_index;
	if(m_needFree) {
		delete m_climb;
		delete m_piston;
	}
	return;
}

bool
Climber::IsIndexed()
{
	return (m_index->Get() == 1);
}

void
Climber::GrabRope()
{
	if(IsIndexed())
		m_piston->Set(true);
}

void
Climber::ReleaseRope()
{
	m_piston->Set(false);
}

void
Climber::ClimbRope(double speed)
{
	m_climb->Set(fabs(speed));
}

void
Climber::MoveToIndex()
{
	while(!IsIndexed())
		m_climb->Set(0.2);
	Stop();
}

void
Climber::Stop()
{
	m_climb->Set(0);
}
