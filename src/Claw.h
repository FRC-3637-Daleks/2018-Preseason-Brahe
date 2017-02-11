/*
 * Claw.h
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

#pragma once
#include <Brahe.h>

class Claw {
public:
	Claw(int piston, int pivot, int arms,
			int gearSwitch, int pegSwitch);
	Claw(Solenoid *piston, Solenoid *pivot, Solenoid *arm,
			int gearSwitch, int pegSwitch);
	Claw(Solenoid &piston, Solenoid &pivot, Solenoid &arm,
			int gearSwtich, int pegSwitch);
	void OpenPiston();
	void ClosePiston();
	void OpenClaw();
	void CloseClaw();
	void OpenPivot();
	void ClosePivot();
	bool IsGearPresent();
	bool IsPegPresent();
	~Claw();

private:
	Solenoid *m_piston;
	Solenoid *m_pivot;
	Solenoid *m_arm;
	DigitalInput *m_gearSwitch;
	DigitalInput *m_pegSwitch;
	bool m_needFree;
};
