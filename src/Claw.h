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
	Claw(int pistonA, int pistonB, int pivotA, int pivotB, int arms,
			int gearSwitch, int pegSwitch);
	Claw(DoubleSolenoid *piston, DoubleSolenoid *pivot, Solenoid *arm,
			int gearSwitch, int pegSwitch);
	Claw(DoubleSolenoid &piston, DoubleSolenoid &pivot, Solenoid &arm,
			int gearSwtich, int pegSwitch);
	void OpenPiston();
	void ClosePiston();
	void OpenArms();
	void CloseArms();
	void OpenPivot();
	void ClosePivot();
	~Claw();

private:
	DoubleSolenoid *m_piston;
	DoubleSolenoid *m_pivot;
	Solenoid *m_arm;
	DigitalInput *m_gearSwitch;
	DigitalInput *m_pegSwitch;
	bool m_needFree;
};
