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
	Solenoid *driveSolenoid;
	Solenoid *armSolenoid;
	Solenoid *pivotSolenoid;
	Solenoid *pistonSolenoid;
	XboxController *xBoxController;
	
	frc::DifferentialDrive *drive;

	void
	RobotInit()
	{
		lightswitch = new Relay(LIGHT_SWITCH);
		leftMotor = new WPI_TalonSRX(LEFT_DRIVEMOTOR);
		leftSlave = new WPI_TalonSRX(LEFT_SLAVEMOTOR);
		rightMotor = new WPI_TalonSRX(RIGHT_DRIVEMOTOR);
		rightSlave = new WPI_TalonSRX(RIGHT_SLAVEMOTOR);
		
		leftJoystick = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		driveSolenoid = new Solenoid(PCM_ID, SHIFTER_SOLENOID);
		
		drive = new DifferentialDrive(*leftMotor, *rightMotor);
		leftSlave->Set(ControlMode::Follower, leftMotor->GetDeviceID());
		rightSlave->Set(ControlMode::Follower, rightMotor->GetDeviceID());
		leftMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		rightMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		leftMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMotor->GetSensorCollection().SetQuadraturePosition(0, 0);
		rightMotor->GetSensorCollection().SetQuadraturePosition(0, 0);

		armSolenoid = new Solenoid(PCM_ID, ARM_SOLENOID);
		xBoxController = new XboxController(XBOX_CONTROLS);
		pivotSolenoid       = new Solenoid(PCM_ID, PIVOT_SOLENOID);
		pistonSolenoid      = new Solenoid(PCM_ID, PISTON_SOLENOID);
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
		drive->TankDrive(leftJoystick->GetY(), rightJoystick->GetY());
		
		double lspeed =
				leftMotor->GetSensorCollection().GetQuadratureVelocity();

		double rspeed =
				rightMotor->GetSensorCollection().GetQuadratureVelocity();
//
//		frc::SmartDashboard::PutNumber("Left Motor Speed", lspeed);
//		frc::SmartDashboard::PutNumber("Right Motor Speed", rspeed);

		if(lspeed == 0 && rspeed == 0){
			if(leftJoystick->GetTrigger()){
				driveSolenoid->Set(true);
			}
			if(rightJoystick->GetTrigger()){
				driveSolenoid->Set(false);
			}
		}

		if(xBoxController->GetAButtonPressed()){
			armSolenoid->Set(true);
		}
		if(xBoxController->GetBButtonPressed()){
			armSolenoid->Set(false);
		}

		if(xBoxController->GetYButtonPressed()){
			pivotSolenoid->Set(true);
		}
		if(xBoxController->GetXButtonPressed()){
			pivotSolenoid->Set(false);
		}


		if(xBoxController->GetStartButtonPressed()){
			pistonSolenoid->Set(true);
		}
		if(xBoxController->GetBackButtonPressed()){
			pistonSolenoid->Set(false);
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
