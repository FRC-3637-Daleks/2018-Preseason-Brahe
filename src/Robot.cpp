#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <WPILib.h>
#include <CANTalon.h>
#include <Brahe.h>
#include <DalekDrive.h>
#include <Claw.h>

class Robot: public frc::IterativeRobot
{
public:
	Compressor *c;
	CANTalon *leftMotor, *rightMotor, *leftSlave, *rightSlave;
	Joystick *leftJoystick, *rightJoystick;
	DalekDrive *d;
	Claw *claw;

	void
	RobotInit()
	{
		leftMotor  = new CANTalon(LEFT_DRIVEMOTOR);
		leftSlave  = new CANTalon(LEFT_SLAVEMOTOR);
		rightMotor = new CANTalon(RIGHT_DRIVEMOTOR);
		rightSlave = new CANTalon(RIGHT_SLAVEMOTOR);

		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);

		// CameraServer::GetInstance()->StartAutomaticCapture();
		c = new Compressor();
		d = new DalekDrive(leftMotor, leftSlave, rightMotor, rightSlave,
				SHIFT_A, SHIFT_B);
		claw = new Claw(PISTON_A, PISTON_B, PIVOT_A, PIVOT_B, ARM,
				GEAR_SWITCH, PEG_SWITCH);

		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}

	void
	AutonomousInit() override
	{
		autoSelected = chooser.GetSelected();
		std::cout << "Auto selected: " << autoSelected << std::endl;
		c->Start();
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void
	AutonomousPeriodic()
	{
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit()
	{
		autoSelected = chooser.GetSelected();
		c->Start();
	}

	void TeleopPeriodic()
	{
		bool useArcade;

		useArcade = (leftJoystick->GetZ() == -1.0);

		if (!useArcade)
			d->TankDrive(leftJoystick, rightJoystick);
		else
			d->ArcadeDrive(leftJoystick);

		if(leftJoystick->GetTrigger())
			d->ShiftGear(LOW_GEAR);
		if(rightJoystick->GetTrigger())
			d->ShiftGear(HIGH_GEAR);
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void DashboardUpdates()
	{
		frc::SmartDashboard::PutNumber("Left Encoder", leftMotor->GetSpeed());
		frc::SmartDashboard::PutNumber("Right Encoder", rightMotor->GetSpeed());

	}
private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto Mode";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
