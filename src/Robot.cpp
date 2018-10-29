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
	WPI_TalonSRX *leftMotor, *rightMotor, *leftSlave, *rightSlave;
	WPI_TalonSRX *climbMotor;
	Joystick *leftJoystick, *rightJoystick;
	Relay *lightswitch;

	void
	RobotInit()
	{
		lightswitch = new Relay (LIGHT_SWITCH);
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
