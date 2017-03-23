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
    m_climb    = new CANTalon(climbMotor);
    m_piston   = new Solenoid(piston);
    m_index    = new DigitalInput(magswitch);
    m_kswitch  = new DigitalInput(climbswitch);
    m_needFree = true;
    m_state    = INDEXING;

    climberInit();
}

Climber::Climber(CANTalon *climbMotor, Solenoid *piston, int magswitch, int climbswitch)
{
    m_climb    = climbMotor;
    m_piston   = piston;
    m_index    = new DigitalInput(magswitch);
    m_kswitch  = new DigitalInput(climbswitch);
    m_needFree = false;
    m_state    = INDEXING;

    climberInit();
}

Climber::Climber(CANTalon &climbMotor, Solenoid &piston, int magswitch, int climbswitch)
{
    m_climb    = &climbMotor;
    m_piston   = &piston;
    m_index    = new DigitalInput(magswitch);
    m_kswitch  = new DigitalInput(climbswitch);
    m_needFree = false;
    m_state    = INDEXING;

    climberInit();
}

Climber::~Climber()
{
	m_state = STOP;
	delete m_index;
	delete m_kswitch;
	if(m_needFree) {
		delete m_climb;
		delete m_piston;
	}
	return;
}

void
Climber::climberInit()
{
    lw->AddActuator("Climbing Mechanism", "Climb Motor", m_climb);
    lw->AddActuator("Climbing Mechanism", "Climb Piston", m_piston);
    lw->AddSensor("Climbing Mechanism", "Climb Indexing sensor", m_index);
    lw->AddSensor("Climbing Mechanism", "Climb Kill switch", m_kswitch);
    m_piston->Set(false);
    m_state = INDEXING;

    std::thread m_indexer(climbIndexer, (void *)this);
    m_indexer.detach();
    return;
}

void
Climber::GrabRope()
{
	if(m_state == INDEXED)
		m_piston->Set(true);
	return;
}

void
Climber::ReleaseRope()
{
	m_piston->Set(false);
}

void
Climber::ClimbRope()
{
	if(m_state == AT_TOP)
		return;
	m_state = CLIMBING;
	return;
}

void
Climber::MoveToIndex()
{
	m_state = INDEXING;
	return;
}

bool
Climber::IsIndexed()
{
	return (m_index->Get() == 0);
}

bool
Climber::IsAtTop()
{
	return (m_kswitch->Get() == 0);
}

void
Climber::Stop()
{
	if(m_state == INDEXING)
		return;
	m_state = INDEXED;
	m_climb->Set(0);
}

void
Climber::climbIndexer(void *c)
{
	Climber *climb = (Climber *)c;

	while(true) {
		frc::SmartDashboard::PutNumber("Current Climb State", climb->m_state);
		switch(climb->m_state) {
			case INDEXING:
				if(!climb->IsIndexed())
					climb->m_climb->Set(INDEX_SPEED);
				else {
					climb->m_state = INDEXED;
					climb->m_climb->Set(0);
					climb->GrabRope();
				}
				break;

			case CLIMBING:
				if(!climb->IsAtTop())
					climb->m_climb->Set(CLIMB_SPEED);
				else {
					climb->m_state = AT_TOP;
					climb->m_climb->Set(0);
				}
				break;

			case STOP:
				climb->m_climb->Set(0);
				break;

			default:
				break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
}
