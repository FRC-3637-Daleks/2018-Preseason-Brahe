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


class Robot: public frc::IterativeRobot
{
public:
	WPI_TalonSRX *leftMotor, *rightMotor, *leftSlaveMotor, *rightSlaveMotor;
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	Relay *lightswitch;
	frc::DifferentialDrive *drive;

	void
	RobotInit()
	{
		leftMotor = new WPI_TalonSRX(LEFT_DRIVEMOTOR);
		rightMotor = new WPI_TalonSRX(RIGHT_DRIVEMOTOR);
		leftSlaveMotor = new WPI_TalonSRX(LEFT_SLAVEMOTOR);
		rightSlaveMotor = new WPI_TalonSRX(RIGHT_SLAVEMOTOR);
		leftJoystick = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		lightswitch = new Relay(LIGHT_SWITCH);
		drive = new DifferentialDrive(*leftMotor, *rightMotor);

		//set slave motors to follow the main motors
		leftSlaveMotor->Set(ControlMode::Follower, leftMotor->GetDeviceID());
		rightSlaveMotor->Set(ControlMode::Follower, rightMotor->GetDeviceID());
		//set motors to get input from QuadEncoder (native to First)
		leftMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		rightMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		//sets up motor initial positions and sensors
		leftMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMotor->GetSensorCollection().SetQuadraturePosition(0, 0);
		rightMotor->GetSensorCollection().SetQuadraturePosition(0, 0);
		}

	void
	RobotPeriodic()
	{

	}

	void
	DisabledInit()
	{

	}

	void
	DisabledPeriodic()
	{

	}

	void
	AutonomousInit()
	{

	}

	bool
	AutonomousMobility(double dist, double lSpeed, double rSpeed)
	{

      return false;
	}

	void
	AutonomousGearPlacement()
	{

	}

	void
	AutonomousPeriodic()
	{

	}

	void
	TeleopInit()
	{
		lightswitch->Set(Relay::kForward);
	}

	void
	TeleopPeriodic()
	{
		
	}

	void
	TestInit()
	{

    }

	void
	TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)
