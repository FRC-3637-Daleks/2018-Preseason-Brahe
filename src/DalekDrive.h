/*
 * DalekDrive.h
 *
 *  Created on: Jan 30, 2017
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

enum CANConstants {
	/* Which PID slot to pull gains from.  Starting 2018, you can choose
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	PIDSlotIdx = 0,

	/* Talon SRX/Victor SPX will supported multiple (cascaded) PID loops.
	 * For now we just want the primary one.
	 */
	PIDLoopIdx = 0,

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	CANTimeoutMs = 10,
};

/*
 * Utility class for handling Robot drive based on a definition of the motor
 * configuration.
 */
class DalekDrive {
  public:
	typedef enum Motors MotorType_t;
	typedef enum Gears GearType_t;

	DalekDrive(int leftMotorChannel, int rightMotorChannel);
	DalekDrive(int leftMotorChannel, int leftSlaveMotorChannel,
             int rightMotorChannel, int rightSlaveMotorChannel,
			 int shiftChannel);
	DalekDrive(WPI_TalonSRX* leftMotor, WPI_TalonSRX* rightMotor);
	DalekDrive(WPI_TalonSRX& leftMotor, WPI_TalonSRX& rightMotor);
	DalekDrive(WPI_TalonSRX* leftMotor, WPI_TalonSRX* leftSlaveMotor,
			WPI_TalonSRX* rightMotor, WPI_TalonSRX* rightSlaveMotor,
			 int shiftChannel);
	DalekDrive(WPI_TalonSRX& leftMotor, WPI_TalonSRX& leftSlaveMotor,
			WPI_TalonSRX& rightMotor, WPI_TalonSRX& rightSlaveMotor,
			 int shiftChannel);
	~DalekDrive();

	void Drive(double outputMagnitude, double curve);
	void TankDrive(frc::GenericHID* leftStick, frc::GenericHID* rightStick,
                 bool squaredInputs = true);
	void TankDrive(frc::GenericHID& leftStick, frc::GenericHID& rightStick,
                 bool squaredInputs = true);
	void TankDrive(double leftValue, double rightValue,
                 bool squaredInputs = true);
	void ArcadeDrive(frc::GenericHID* stick, bool squaredInputs = true);
	void ArcadeDrive(frc::GenericHID& stick, bool squaredInputs = true);
	void ArcadeDrive(double moveValue, double rotateValue,
                   bool squaredInputs = true);
	void SetLeftRightMotorOutputs(double leftOutput, double rightOutput);
	void SetInvertedMotor(MotorType_t motor, bool isInverted);
	void SetMaxOutput(double maxOutput);
	int  GetPosition(MotorType_t motor);
	void ShiftGear(GearType_t speed);
	bool DriveOk();

 private:
	void InitDalekDrive();
	bool printFaults();
	WPI_TalonSRX *m_leftMotor;
	WPI_TalonSRX *m_rightMotor;
	WPI_TalonSRX *m_leftSlaveMotor;
	WPI_TalonSRX *m_rightSlaveMotor;
	Solenoid *m_gearShift;
	frc::DifferentialDrive  *m_drive;
	bool m_needFree;
};
