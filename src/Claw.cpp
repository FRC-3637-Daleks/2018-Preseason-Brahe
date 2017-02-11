/*
 * DalekDrive.h
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

Claw::Claw(int piston, int pivot, int arm, int gearSwitch, int pegSwitch)
{
	m_piston     = new Solenoid(PCM_ID, piston);
	m_pivot      = new Solenoid(PCM_ID, pivot);
	m_arm        = new Solenoid(PCM_ID, arm);
	m_gearSwitch = new DigitalInput(gearSwitch);
	m_pegSwitch  = new DigitalInput(pegSwitch);
	m_needFree   = true;
	return;
}

Claw::Claw(Solenoid *piston, Solenoid *pivot, Solenoid *arm,
		int gearSwitch, int pegSwitch)
{
	m_piston     = piston;
	m_pivot      = pivot;
	m_arm        = arm;
	m_gearSwitch = new DigitalInput(gearSwitch);
	m_pegSwitch  = new DigitalInput(pegSwitch);
	m_needFree   = false;
	return;
}

Claw::Claw(Solenoid &piston, Solenoid &pivot, Solenoid &arm,
		int gearSwitch, int pegSwitch)
{
	m_piston     = &piston;
	m_pivot      = &pivot;
	m_arm        = &arm;
	m_gearSwitch = new DigitalInput(gearSwitch);
	m_pegSwitch  = new DigitalInput(pegSwitch);
	m_needFree   = false;
	return;
}

Claw::~Claw()
{
	delete m_gearSwitch;
	delete m_pegSwitch;
	if(m_needFree) {
		delete m_piston;
		delete m_arm;
		delete m_pivot;
	}
}

void
Claw::OpenPiston()
{
	m_piston->Set(true);
}

void
Claw::ClosePiston()
{
	m_piston->Set(false);
}

void
Claw::OpenPivot()
{
	m_pivot->Set(false);
}

void
Claw::ClosePivot()
{
	m_pivot->Set(true);
}

void
Claw::OpenClaw()
{
	m_arm->Set(true);
}

void
Claw::CloseClaw()
{
	m_arm->Set(false);
}

bool
Claw::IsGearPresent()
{
	return (m_gearSwitch->Get() == 1);
}

bool
Claw::IsPegPresent()
{
	return (m_pegSwitch->Get() == 1);
}


