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
#include <Climber.h>
#include <JoyStick.h>
class Robot: public frc::IterativeRobot
{
public:
	CANTalon *leftMotor, *rightMotor;
	Joystick *leftJoystick, *rightJoystick;
	Climber *OlReliable;
	XboxController *xBox;

	void
	RobotInit()
	{
		leftMotor  = new CANTalon(LEFT_FRONT_DRIVEMOTOR);
		rightMotor = new CANTalon(RIGHT_FRONT_DRIVEMOTOR);

		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		xBox = new XboxController(NUM_JOYSTICKS);

		OlReliable = new Climber(GEARBOX, PISTON, FORWARD, REVERSE, xBox);
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}

	void
	AutonomousInit() override
	{
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

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

	}

	void TeleopPeriodic()
	{

		double y1, y2;

		y1 = leftJoystick->GetY();
		y2 = rightJoystick->GetY();

		leftMotor->Set(y1);
		rightMotor->Set(y2);
		OlReliable->Play();


	}

	void TestPeriodic()
	{
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
