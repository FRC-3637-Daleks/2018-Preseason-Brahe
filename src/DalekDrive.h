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

#include <memory>
#include <sstream>
#include <WPILib.h>
#include <CANTalon.h>
#include <Brahe.h>

/*
 * Utility class for handling Robot drive based on a definition of the motor
 * configuration.
 */
class DalekDrive {
  public:
	typedef enum DriveMotors MotorType_t;
	typedef enum Gears GearType_t;

	DalekDrive(int leftMotorChannel, int rightMotorChannel);
	DalekDrive(int leftMotorChannel, int leftSlaveMotorChannel,
             int rightMotorChannel, int rightSlaveMotorChannel,
			 int forwardChannel, int reverseChannel);
	DalekDrive(CANTalon* leftMotor, CANTalon* rightMotor);
	DalekDrive(CANTalon& leftMotor, CANTalon& rightMotor);
	DalekDrive(CANTalon* leftMotor, CANTalon* leftSlaveMotor,
             CANTalon* rightMotor, CANTalon* rightSlaveMotor,
			 int forwardChannel, int reverseChannel);
	DalekDrive(CANTalon& leftMotor, CANTalon& leftSlaveMotor,
             CANTalon& rightMotor, CANTalon& rightSlaveMotor,
			 int forwardChannel, int reverseChannel);
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
	void SetSensitivity(double sensitivity);
	void SetMaxOutput(double maxOutput);
	void ShiftGear();

 protected:
	void InitDalekDrive();
	double Limit(double num);
	void Normalize(double* wheelSpeeds);
	void RotateVector(double& x, double& y, double angle);

	static const int kMaxNumberOfMotors = 4;
	double m_sensitivity = 0.5;
	double m_maxOutput = 1.0;
	CANTalon *m_leftMotor;
	CANTalon *m_rightMotor;
	CANTalon *m_leftSlaveMotor;
	CANTalon *m_rightSlaveMotor;
	DoubleSolenoid *m_gearShift;
	frc::RobotDrive *m_drive;
	bool m_needFree;
};
