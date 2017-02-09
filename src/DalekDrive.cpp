/*
 * DalekDrive.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Michael
 */

#include <DalekDrive.h>
#include <CANTalon.h>

using namespace frc;

DalekDrive::DalekDrive(int leftMotorChannel, int rightMotorChannel)
{
	m_leftMotor       = new CANTalon(leftMotorChannel);
	m_rightMotor      = new CANTalon(rightMotorChannel);
	m_leftSlaveMotor  = NULL;
	m_rightSlaveMotor = NULL;
	m_gearShift       = NULL;
	m_drive           = new RobotDrive(m_leftMotor, m_rightMotor);
	InitDalekDrive();

	m_needFree = true;
}

DalekDrive::DalekDrive(int leftMotorChannel, int leftSlaveMotorChannel,
         int rightMotorChannel, int rightSlaveMotorChannel,
		 int forwardChannel, int reverseChannel)
{
	m_leftMotor       = new CANTalon(leftMotorChannel);
	m_rightMotor      = new CANTalon(rightMotorChannel);
	m_leftSlaveMotor  = new CANTalon(leftSlaveMotorChannel);
	m_rightSlaveMotor = new CANTalon(rightSlaveMotorChannel);
	m_gearShift       = new DoubleSolenoid(forwardChannel, reverseChannel);
	m_drive           = new RobotDrive(m_leftMotor, m_rightMotor);
	InitDalekDrive();

	m_needFree = true;
}

DalekDrive::DalekDrive(CANTalon* leftMotor, CANTalon* rightMotor)
{
	m_leftMotor       = leftMotor;
	m_rightMotor      = rightMotor;
	m_leftSlaveMotor  = NULL;
	m_rightSlaveMotor = NULL;
	m_gearShift       = NULL;
	m_drive           = new RobotDrive(m_leftMotor, m_rightMotor);
	InitDalekDrive();

	m_needFree = false;
}

DalekDrive::DalekDrive(CANTalon& leftMotor, CANTalon& rightMotor)
{
	m_leftMotor       = &leftMotor;
	m_rightMotor      = &rightMotor;
	m_leftSlaveMotor  = NULL;
	m_rightSlaveMotor = NULL;
	m_gearShift       = NULL;
	m_drive           = new RobotDrive(m_leftMotor, m_rightMotor);
	InitDalekDrive();

	m_needFree = false;
}

DalekDrive::DalekDrive(CANTalon* leftMotor, CANTalon* leftSlaveMotor,
         CANTalon* rightMotor, CANTalon* rightSlaveMotor,
		 int forwardChannel, int reverseChannel)
{
	m_leftMotor       = leftMotor;
	m_leftSlaveMotor  = leftSlaveMotor;
	m_rightMotor      = rightMotor;
	m_rightSlaveMotor = rightSlaveMotor;
	m_gearShift       = new DoubleSolenoid(forwardChannel, reverseChannel);
	m_drive           = new RobotDrive(m_leftMotor, m_rightMotor);
	InitDalekDrive();

	m_needFree = false;
}

DalekDrive::DalekDrive(CANTalon& leftMotor, CANTalon& leftSlaveMotor,
         CANTalon& rightMotor, CANTalon& rightSlaveMotor,
		 int forwardChannel, int reverseChannel)
{
	m_leftMotor       = &leftMotor;
	m_leftSlaveMotor  = &leftSlaveMotor;
	m_rightMotor      = &rightMotor;
	m_rightSlaveMotor = &rightSlaveMotor;
	m_gearShift       = new DoubleSolenoid(forwardChannel, reverseChannel);
	m_drive           = new RobotDrive(m_leftMotor, m_rightMotor);
	InitDalekDrive();

	m_needFree = false;
}

DalekDrive::~DalekDrive()
{
	delete m_drive;
	if(m_gearShift)
		delete m_gearShift;
	if(m_needFree) {
		delete m_leftMotor;
		delete m_rightMotor;
		if(m_leftSlaveMotor)
			delete m_leftSlaveMotor;
		if(m_rightSlaveMotor)
			delete m_rightSlaveMotor;
	}
	m_needFree = false;
	return;
}

void
DalekDrive::Drive(double outputMagnitude, double curve)
{
	if(m_drive)
		m_drive->Drive(outputMagnitude, curve);
	return;
}

void
DalekDrive::TankDrive(GenericHID* leftStick, GenericHID* rightStick,
             bool squaredInputs)
{
	if(m_drive)
		m_drive->TankDrive(leftStick, rightStick, squaredInputs);
}

void
DalekDrive::TankDrive(GenericHID& leftStick, GenericHID& rightStick,
             bool squaredInputs)
{
	if(m_drive)
		m_drive->TankDrive(leftStick, rightStick, squaredInputs);
}

void
DalekDrive::TankDrive(double leftValue, double rightValue,
             bool squaredInputs)
{
	if(m_drive)
		m_drive->TankDrive(leftValue, rightValue, squaredInputs);
}

void
DalekDrive::ArcadeDrive(GenericHID* stick, bool squaredInputs)
{
	if(m_drive)
		m_drive->ArcadeDrive(stick, squaredInputs);
}

void
DalekDrive::ArcadeDrive(GenericHID& stick, bool squaredInputs)
{
	if(m_drive)
		m_drive->ArcadeDrive(stick, squaredInputs);
}

void
DalekDrive::ArcadeDrive(double moveValue, double rotateValue,
               bool squaredInputs)
{
	if(m_drive)
		m_drive->ArcadeDrive(moveValue, rotateValue, squaredInputs);
}

void
DalekDrive::SetLeftRightMotorOutputs(double leftOutput, double rightOutput)
{
	if(m_drive)
		m_drive->SetLeftRightMotorOutputs(leftOutput, rightOutput);
}

void
DalekDrive::SetInvertedMotor(MotorType_t motor, bool isInverted)
{
	switch(motor) {
	case LEFT_DRIVEMOTOR:
		if(m_leftMotor) {
			m_leftMotor->SetInverted(isInverted);
			if(m_leftSlaveMotor)
				m_leftSlaveMotor->SetInverted(isInverted);
		}
		break;

	case RIGHT_DRIVEMOTOR:
		if(m_rightMotor) {
			m_rightMotor->SetInverted(isInverted);
			if(m_rightSlaveMotor)
					m_rightSlaveMotor->SetInverted(isInverted);
		}
		break;

	default:
		break;
	}
}

void
DalekDrive::SetSensitivity(double sensitivity)
{
	if(m_drive)
		m_drive->SetSensitivity(sensitivity);
}

void
DalekDrive::SetMaxOutput(double maxOutput)
{
	if(m_drive)
		m_drive->SetMaxOutput(maxOutput);
}

void
DalekDrive::InitDalekDrive(void)
{
	// Configure the Talon's as needed
	m_leftMotor->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	m_rightMotor->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	m_leftMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	m_rightMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	m_leftMotor->ConfigEncoderCodesPerRev(ENCODER_TICKS_PER_REV*4);
	m_rightMotor->ConfigEncoderCodesPerRev(ENCODER_TICKS_PER_REV*4);
	m_leftMotor->SetSensorDirection(true);
	m_rightMotor->SetSensorDirection(false);
	m_leftMotor->SetInverted(false);
	m_rightMotor->SetInverted(false);
	if(m_leftSlaveMotor) {
		m_leftSlaveMotor->SetControlMode(CANTalon::ControlMode::kFollower);
		m_leftSlaveMotor->Set(m_leftMotor->GetDeviceID());
		m_leftSlaveMotor->SetInverted(false);
	}
	if(m_rightSlaveMotor) {
		m_rightSlaveMotor->SetControlMode(CANTalon::ControlMode::kFollower);
		m_rightSlaveMotor->Set(m_rightMotor->GetDeviceID());
		m_rightSlaveMotor->SetInverted(false);
	}
	if(m_gearShift)
		m_gearShift->Set(DoubleSolenoid::kForward);
}

void
DalekDrive::ShiftGear()
{
	if(m_gearShift) {
		DoubleSolenoid::Value current = m_gearShift->Get();
		switch(current) {
		case DoubleSolenoid::kForward:
			m_gearShift->Set(DoubleSolenoid::kReverse);
			break;
		case DoubleSolenoid::kReverse:
			m_gearShift->Set(DoubleSolenoid::kForward);
			break;
		case DoubleSolenoid::kOff:
			m_gearShift->Set(DoubleSolenoid::kForward);
			break;
		default:
			break;
		}
	}
}
