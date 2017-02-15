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
		 int shiftChannel)
{
	m_leftMotor       = new CANTalon(leftMotorChannel);
	m_rightMotor      = new CANTalon(rightMotorChannel);
	m_leftSlaveMotor  = new CANTalon(leftSlaveMotorChannel);
	m_rightSlaveMotor = new CANTalon(rightSlaveMotorChannel);
	m_gearShift       = new Solenoid(PCM_ID, shiftChannel);
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
		 int shiftChannel)
{
	m_leftMotor       = leftMotor;
	m_leftSlaveMotor  = leftSlaveMotor;
	m_rightMotor      = rightMotor;
	m_rightSlaveMotor = rightSlaveMotor;
	m_gearShift       = new Solenoid(PCM_ID, shiftChannel);
	m_drive           = new RobotDrive(m_leftMotor, m_rightMotor);
	InitDalekDrive();

	m_needFree = false;
}

DalekDrive::DalekDrive(CANTalon& leftMotor, CANTalon& leftSlaveMotor,
         CANTalon& rightMotor, CANTalon& rightSlaveMotor,
		 int shiftChannel)
{
	m_leftMotor       = &leftMotor;
	m_leftSlaveMotor  = &leftSlaveMotor;
	m_rightMotor      = &rightMotor;
	m_rightSlaveMotor = &rightSlaveMotor;
	m_gearShift       = new Solenoid(PCM_ID, shiftChannel);
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
	m_leftMotor->ConfigNominalOutputVoltage(0.0f, -0.0f);
	m_rightMotor->ConfigNominalOutputVoltage(0.0f, -0.0f);
	m_leftMotor->ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_rightMotor->ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_leftMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	m_rightMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	m_leftMotor->ConfigEncoderCodesPerRev(ENCODER_TICKS_PER_REV);
	m_rightMotor->ConfigEncoderCodesPerRev(ENCODER_TICKS_PER_REV);
	m_leftMotor->SetSensorDirection(true);
	m_rightMotor->SetSensorDirection(false);
	m_leftMotor->SetPosition(0);
	m_rightMotor->SetPosition(0);
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
		ShiftGear(HIGH_GEAR);
}

void
DalekDrive::ShiftGear(GearType_t speed)
{
	// Assumes that the forward setting of the solenoid == HIGH_GEAR
	if(m_gearShift) {
		switch(speed) {
		case HIGH_GEAR:
			m_gearShift->Set(false);
			break;
		case LOW_GEAR:
			m_gearShift->Set(true);
			break;
		default:
			break;
		}
	}
}

void
DalekDrive::printFaults(MotorType_t p, int faults)
{
	if(p == LEFT_DRIVEMOTOR) {
		frc::SmartDashboard::PutNumber("Left Talon reported faults", faults);
		if(m_leftSlaveMotor) {
			frc::SmartDashboard::PutNumber("Left slave status",
					m_leftSlaveMotor->GetFaults());
		}
	}
	if(p == RIGHT_DRIVEMOTOR) {
		frc::SmartDashboard::PutNumber("Right Talon reported faults", faults);
		if(m_rightSlaveMotor) {
			frc::SmartDashboard::PutNumber("Right slave status",
					m_rightSlaveMotor->GetFaults());
		}
	}
	return;
}

bool
DalekDrive::DriveOk()
{
	CANTalon::FeedbackDeviceStatus estat;
	int mstat;

	// update dashboard of current draw for motors
	frc::SmartDashboard::PutNumber("Left Motor current",
			m_leftMotor->GetOutputCurrent());
	if(m_leftSlaveMotor)
		frc::SmartDashboard::PutNumber("Left Slave Motor current",
			m_leftSlaveMotor->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Right Motor current",
			m_rightMotor->GetOutputCurrent());
	if(m_rightSlaveMotor)
		frc::SmartDashboard::PutNumber("Right Slave Motor current",
			m_rightSlaveMotor->GetOutputCurrent());

	// check sensor status
	estat = m_leftMotor->IsSensorPresent(CANTalon::FeedbackDevice::QuadEncoder);
	if(estat != CANTalon::FeedbackDeviceStatus::FeedbackStatusPresent)
		return false;
	estat = m_rightMotor->IsSensorPresent(CANTalon::FeedbackDevice::QuadEncoder);
	if(estat != CANTalon::FeedbackDeviceStatus::FeedbackStatusPresent)
		return false;

	// check for motor faults
	mstat = m_leftMotor->GetFaults();
	if(mstat != 0) {
		printFaults(LEFT_DRIVEMOTOR, mstat);
		return false;
	}
	mstat = m_leftMotor->GetStickyFaults();
	if(mstat) {
		printFaults(LEFT_DRIVEMOTOR, mstat);
		m_leftMotor->ClearStickyFaults();
		return false;
	}

	mstat = m_rightMotor->GetFaults();
	if(mstat) {
		printFaults(RIGHT_DRIVEMOTOR, mstat);
		return false;
	}
	mstat = m_rightMotor->GetStickyFaults();
	if(mstat) {
		printFaults(RIGHT_DRIVEMOTOR, mstat);
		return false;
	}
	return true;
}
