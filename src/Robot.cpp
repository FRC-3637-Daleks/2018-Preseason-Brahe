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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <GripPipeline.cpp>
#include <GripPipeline.h>
class Robot: public frc::IterativeRobot
{
public:
	CANTalon *leftMotor, *rightMotor;
	Joystick *leftJoystick, *rightJoystick;
	Climber *OlReliable;
	XboxController *xBox;

	/*static void VisionThread()
	{
	        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
	        camera.SetResolution(640, 480);
	        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
	        cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
	        cv::Mat source;
	        cv::Mat output;
	        while(true) {
	        	cvSink.GrabFrame(source);
	        	cvtColor(source, output, cv::COLOR_BGR2GRAY);
	        	outputStreamStd.PutFrame(output);
	        }
	}*/
	void
	RobotInit()
	{

		leftMotor  = new CANTalon(LEFT_FRONT_DRIVEMOTOR);
		rightMotor = new CANTalon(RIGHT_FRONT_DRIVEMOTOR);

		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		xBox = new XboxController(NUM_JOYSTICKS);

		OlReliable = new Climber(GEARBOX, PISTON, NUM_JOYSTICKS);
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

		double x, y;

		x = leftJoystick->GetX();
		y = rightJoystick->GetY();

		leftMotor->Set(y);
		rightMotor->Set(y);
		if (y < (1-x/2) && y > (-1+x/2)){
			leftMotor->Set((leftMotor->Get() + x/2));
			rightMotor->Set((rightMotor->Get() - x/2));
		}
		else{
			if(x>0){
				if(y>0){
					rightMotor->Set(rightMotor->Get()-1+leftMotor->Get());
					leftMotor->Set(1);

				}
				else{
					rightMotor->Set(rightMotor->Get()+1+leftMotor->Get());
					leftMotor->Set(-1);
				}
			}
			else{
				if(y>0){
					leftMotor->Set(leftMotor->Get()-1+rightMotor->Get());
					rightMotor->Set(1);
				}
				else{
					leftMotor->Set(leftMotor->Get()+1+rightMotor->Get());
					rightMotor->Set(-1);
				}
			}
		}

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
