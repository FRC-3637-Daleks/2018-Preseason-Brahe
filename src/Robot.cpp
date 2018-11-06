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

	frc::DifferentialDrive *drive;
	Solenoid *gearShift;

	void
	RobotInit()
	{
		//introducing light switch
		lightswitch = new Relay(LIGHT_SWITCH);

		//introducing motors based on FRC class; left (master) drive motor, right (master) drive motor
		leftMotor = new WPI_TalonSRX(LEFT_DRIVEMOTOR);
		leftSlave = new WPI_TalonSRX(LEFT_SLAVEMOTOR);
		rightMotor = new WPI_TalonSRX(RIGHT_DRIVEMOTOR);
		rightSlave = new WPI_TalonSRX(RIGHT_SLAVEMOTOR);
		leftJoystick = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);

		//introducing differential drive
		drive = new DifferentialDrive(*leftMotor, *rightMotor);

		//setting leftSlave and rightSlave as followers of the leftMaster and rightMaster
		leftSlave->Set(ControlMode::Follower, leftMotor->GetDeviceID());
		rightSlave->Set(ControlMode::Follower, rightMotor->GetDeviceID());

		/*leftMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
				10);
		  rightMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
				10);*/

		leftMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMotor->SetSelectedSensorPosition(0, 0, 10);
		//leftMotor->GetSensorCollection().SetQuadraturePosition(0, 0);
		//rightMotor->GetSensorCollection().SetQuadraturePosition(0, 0);

		gearShift = new Solenoid(PCM_ID, SHIFTER_SOLENOID);

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
		lightswitch->Set(Relay::kOn);
	}

	void
	TeleopPeriodic()
	{
		//Do tank drive, position of left joystick and right joystick
		drive->TankDrive(leftJoystick->GetY(), rightJoystick->GetY(), true);
		double lSpeed =
						leftMotor->GetSensorCollection().GetQuadratureVelocity();
		double rSpeed =
						rightMotor->GetSensorCollection().GetQuadratureVelocity();
				/*frc::SmartDashboard::PutNumber("Left Poop Speed", lSpeed);
				frc::SmartDashboard::PutNumber("Right Motor Speed", rSpeed); */

		if (lSpeed == 0 and rSpeed == 0) {
			if (leftJoystick->GetTrigger())
				gearShift->Set(true);
			if (rightJoystick->GetTrigger())
				gearShift->Set(false);
		}
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
