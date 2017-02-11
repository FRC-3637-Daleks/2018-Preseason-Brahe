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

Claw::Claw(int pistonA, int pistonB, int pivotA, int pivotB, int arm,
		int gearSwitch, int pegSwitch)
{
	m_piston     = new DoubleSolenoid(pistonA, pistonB);
	m_pivot      = new DoubleSolenoid(pivotA, pivotB);
	m_arm        = new Solenoid(arm);
	m_gearSwitch = new DigitalInput(gearSwitch);
	m_pegSwitch  = new DigitalInput(pegSwitch);
	m_needFree   = true;
	return;
}

Claw::Claw(DoubleSolenoid *piston, DoubleSolenoid *pivot, Solenoid *arm,
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

Claw::Claw(DoubleSolenoid &piston, DoubleSolenoid &pivot, Solenoid &arm,
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

void
Claw::OpenPiston()
{
	m_piston->Set(frc::DoubleSolenoid::kForward);
}

void
Claw::ClosePiston()
{
	m_piston->Set(frc::DoubleSolenoid::kReverse);
}

void
Claw::OpenPivot()
{
	m_pivot->Set(frc::DoubleSolenoid::kForward);
}

void
Claw::ClosePivot()
{
	m_pivot->Set(frc::DoubleSolenoid::kReverse);
}

void
Claw::OpenArms()
{
	m_arm->Set(true);
}

void
Claw::CloseArms()
{
	m_arm->Set(false);
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
