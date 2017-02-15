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
	TravelMode();
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
	TravelMode();
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
	TravelMode();
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

bool
Claw::PegPresent()
{
	return (m_pegSwitch->Get() == 0);
}

void
Claw::PegPlacementMode()
{
//	if(PegPresent()) {
		CloseClaw();
		RetractPivot();
		ExtendPiston();
//	}
}

void
Claw::GroundMode()
{
	OpenClaw();
	ExtendPivot();
	ExtendPiston();
}

void
Claw::TravelMode()
{
	CloseClaw();
	RetractPivot();
	RetractPiston();
}

