/*
 * Claw.cpp
 *
 *  Created on: Feb 10, 2017
 *      Author: Michael
 */
/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Claw.h>

using namespace frc;

Claw::Claw(int piston, int pivot, int arm, int gearSwitch)
{
	m_piston     = new Solenoid(PCM_ID, piston);
	m_pivot      = new Solenoid(PCM_ID, pivot);
	m_arm        = new Solenoid(PCM_ID, arm);
	m_gearSwitch = new DigitalInput(gearSwitch);
	m_needFree   = true;
	TravelMode();
	return;
}

Claw::Claw(Solenoid *piston, Solenoid *pivot, Solenoid *arm,
		int gearSwitch)
{
	m_piston     = piston;
	m_pivot      = pivot;
	m_arm        = arm;
	m_gearSwitch = new DigitalInput(gearSwitch);
	m_needFree   = false;
	TravelMode();
	return;
}

Claw::Claw(Solenoid &piston, Solenoid &pivot, Solenoid &arm,
		int gearSwitch)
{
	m_piston     = &piston;
	m_pivot      = &pivot;
	m_arm        = &arm;
	m_gearSwitch = new DigitalInput(gearSwitch);
	m_needFree   = false;
	TravelMode();
	return;
}

Claw::~Claw()
{
	delete m_gearSwitch;
	if(m_needFree) {
		delete m_piston;
		delete m_arm;
		delete m_pivot;
	}
}

void
Claw::ExtendPiston()
{
	m_piston->Set(true);
}

void
Claw::RetractPiston()
{
	m_piston->Set(false);
}

void
Claw::ExtendPivot()
{
	m_pivot->Set(true);
}

void
Claw::RetractPivot()
{
	m_pivot->Set(false);
}

void
Claw::OpenClaw()
{
	m_arm->Set(false);
}

void
Claw::CloseClaw()
{
	m_arm->Set(true);
}

bool
Claw::IsOpen()
{
	return (m_arm->Get() == false);
}

bool
Claw::IsClosed()
{
	return (m_arm->Get() == true);
}

bool
Claw::GearPresent()
{
	return (m_gearSwitch->Get() == 0);
}

void
Claw::DeployMode()
{
	CloseClaw();
	ExtendPiston();
	RetractPivot();
	m_state = DEPLOY_MODE;
}

void
Claw::GroundMode()
{
	CloseClaw();
	ExtendPiston();
	ExtendPivot();
	Wait(0.1);
	OpenClaw();
	m_state = GROUND_MODE;
}

void
Claw::TravelMode()
{
	CloseClaw();
	RetractPiston();
	RetractPivot();
	m_state = TRAVEL_MODE;
}

void
Claw::CheckForGear()
{
	if (m_state != DEPLOY_MODE)
		if(GearPresent())
			CloseClaw();
}
