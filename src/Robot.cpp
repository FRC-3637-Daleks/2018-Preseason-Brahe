#include <iostream>
#include <memory>
#include <string>

#include <math.h>

#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Brahe.h>

class Robot: public frc::IterativeRobot {

public:
	WPI_TalonSRX *leftMotor, *rightMotor, *leftSlaveMotor, *rightSlaveMotor;
	WPI_TalonSRX *climbMotor;
	Joystick *leftJoystick, *rightJoystick;
	Relay *lightswitch;

	frc::DifferentialDrive *drive;
	Solenoid *gearShift;

	void RobotInit() {
		lightswitch = new Relay(LIGHT_SWITCH);
		leftMotor = new WPI_TalonSRX(LEFT_DRIVEMOTOR);
		leftSlaveMotor = new WPI_TalonSRX(LEFT_SLAVEMOTOR);
		rightMotor = new WPI_TalonSRX(RIGHT_DRIVEMOTOR);
		rightSlaveMotor = new WPI_TalonSRX(RIGHT_SLAVEMOTOR);
		leftJoystick = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);

		drive = new DifferentialDrive(*leftMotor, *rightMotor);
		leftSlaveMotor->Set(ControlMode::Follower, leftMotor->GetDeviceID());
		rightSlaveMotor->Set(ControlMode::Follower, rightMotor->GetDeviceID());
		leftMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
				10);
		rightMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
				10);
		leftMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMotor->GetSensorCollection().SetQuadraturePosition(0, 0);
		rightMotor->GetSensorCollection().SetQuadraturePosition(0, 0);

		gearShift = new Solenoid(PCM_ID, SHIFTER_SOLENOID);

	}

	void RobotPeriodic() {
	}

	void DisabledInit() {

	}

	void DisabledPeriodic() {

	}

	void AutonomousInit() {

	}

	bool AutonomousMobility(double dist, double lSpeed, double rSpeed) {

		return false;
	}

	void AutonomousGearPlacement() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		lightswitch->Set(Relay::kForward);
	}

	void TeleopPeriodic() {
		drive->TankDrive(leftJoystick->GetY(), rightJoystick->GetY(), true);

		double lspeed =
				leftMotor->GetSensorCollection().GetQuadratureVelocity();
		double rspeed =
				rightMotor->GetSensorCollection().GetQuadratureVelocity();
		frc::SmartDashboard::PutNumber("Left Poop Speed", lspeed);
		frc::SmartDashboard::PutNumber("Right Motor Speed", rspeed);

		if (lspeed == 0 and rspeed == 0) {
			if (leftJoystick->GetTrigger())
				gearShift->Set(true);
			if (rightJoystick->GetTrigger())
				gearShift->Set(false);
		}

	}

	void TestInit() {
	}

	void TestPeriodic() {

	}
};

START_ROBOT_CLASS(Robot)
