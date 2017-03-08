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

Climber::Climber(int climbMotor, int piston, int magswitch, int climbswitch)
{
    m_climb  = new CANTalon(climbMotor);
    m_piston = new Solenoid(piston);
    m_index  = new DigitalInput(magswitch);
    m_kswitch = new DigitalInput(climbswitch);
    m_needFree = true;
    climberInit();
}

Climber::Climber(CANTalon *climbMotor, Solenoid *piston, int magswitch, int climbswitch)
{
    m_climb  = climbMotor;
    m_piston = piston;
    m_index  = new DigitalInput(magswitch);
    m_kswitch = new DigitalInput(climbswitch);
    m_needFree = false;
    climberInit();
}

Climber::Climber(CANTalon &climbMotor, Solenoid &piston, int magswitch, int climbswitch)
{
    m_climb  = &climbMotor;
    m_piston = &piston;
    m_index  = new DigitalInput(magswitch);
    m_kswitch = new DigitalInput(climbswitch);
    m_needFree = false;
    climberInit();
}

Climber::~Climber()
{
	delete m_index;
	delete m_kswitch;
	if(m_needFree) {
		delete m_climb;
		delete m_piston;
	}
	return;
}

void
climberInit()
{
    lw->AddActuator("Climbing Mechanism", "Climb Motor", m_climb);
    lw->AddActuator("Climbing Mechanism", "Climb Piston", m_piston);
    lw->AddSensor("Climbing Mechanism", "Climb Indexing sensor", m_index);
    lw->AddSensor("Climbing Mechanism", "Climb Kill switch", m_kswitch);
    m_piston->Set(false);
    return;
}

bool
Climber::IsIndexed()
{
	return (m_index->Get() == 0);
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
	if(speed > 0.0)
		speed *= -1.0;

	if(!IsAtTop())
		m_climb->Set(speed);
	else
		Stop();
}

void
Climber::MoveToIndex()
{
	while(!IsIndexed())
		ClimbRope(INDEX_SPEED);
	Stop();
}

bool
Climber::IsAtTop()
{
	return (m_kswitch->Get() == 0);
}

void
Climber::Stop()
{
	m_climb->Set(0);
}
