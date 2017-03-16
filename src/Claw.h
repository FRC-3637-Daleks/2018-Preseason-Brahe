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
	enum PistonState { TRAVEL_MODE, DEPLOY_MODE, GROUND_MODE, NUM_POSITIONS };
	Claw(int piston, int pivot, int arms, int gearSwitch, int cameraServo);
	Claw(Solenoid *piston, Solenoid *pivot, Solenoid *arm, int gearSwitch, int cameraServo);
	Claw(Solenoid &piston, Solenoid &pivot, Solenoid &arm, int gearSwtich, int cameraServo);
	Claw(Solenoid &piston, Solenoid &pivot, Solenoid &arm, int gearSwitch, int cameraServo);
	void ExtendPiston();
	void RetractPiston();
	void OpenClaw();
	void CloseClaw();
	void ExtendPivot();
	void RetractPivot();
	bool GearPresent();
	void DeployMode();
	void GroundMode();
	void TravelMode();
	void SetCameraView(double angle);
	void ServoDown();
	bool IsOpen();
	bool IsClosed();
	void CheckForGear();
	~Claw();

private:
	Solenoid *m_piston;
	Solenoid *m_pivot;
	Solenoid *m_arm;
	DigitalInput *m_gearSwitch;
	DigitalInput *m_pegSwitch;
	Servo *m_cameraServo;
	PistonState m_state;
	bool m_needFree;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
    void clawInit();
};
