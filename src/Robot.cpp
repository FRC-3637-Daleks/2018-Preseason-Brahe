#include <iostream>
#include <memory>
#include <string>

#include <math.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <WPILib.h>
#include <CANTalon.h>
#include <Brahe.h>
#include <DalekDrive.h>
#include <Claw.h>
#include <Climber.h>

class Robot: public frc::IterativeRobot
{
public:
	Compressor *c;
	CANTalon *leftMotor, *rightMotor, *leftSlave, *rightSlave;
	CANTalon *climbMotor;
	Joystick *leftJoystick, *rightJoystick;
	XboxController *xbox;
	DalekDrive *d;
	Claw *claw;
	Climber *climb;
	Solenoid *climbPiston;

#ifdef OMIT
	static 
	void VisionThread()
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
	}
#endif
	void
	RobotInit()
	{
		leftMotor  = new CANTalon(LEFT_DRIVEMOTOR);
		leftSlave  = new CANTalon(LEFT_SLAVEMOTOR);
		rightMotor = new CANTalon(RIGHT_DRIVEMOTOR);
		rightSlave = new CANTalon(RIGHT_SLAVEMOTOR);
		climbMotor = new CANTalon(CLIMB_MOTOR);
		climbPiston = new Solenoid(PCM_ID, CLIMB);

		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		xbox          = new XboxController(XBOX_CONTROLS);

		c = new Compressor(PCM_ID);
		d = new DalekDrive(leftMotor, leftSlave, rightMotor, rightSlave, SHIFTER);
		claw = new Claw(PISTON, PIVOT, ARM, GEAR_SWITCH, PEG_SWITCH);
		climb = new Climber(climbMotor, climbPiston, DRUM_SWITCH);

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
		claw->TravelMode();
	}

	void TeleopPeriodic()
	{
		bool useArcade, useDrive;
		double climbvalue;

		useArcade = (leftJoystick->GetZ() == -1.0);
		useDrive  = (rightJoystick->GetZ() == -1.0);

		// Drive controls
		if (useArcade)
			d->ArcadeDrive(leftJoystick);
		else if (useDrive) {
			double outputMagnitude = rightJoystick->GetY();
			double curve = leftJoystick->GetX() / 2;
				if (outputMagnitude + curve <= 1 && outputMagnitude - curve >= -1){
					d->TankDrive(outputMagnitude + curve, outputMagnitude - curve);
				}
				else{
					if (outputMagnitude > 0){
						if(curve > 0){
							d->TankDrive(1, outputMagnitude - 1 + outputMagnitude);
						}
						else{
							d->TankDrive(outputMagnitude -1 + outputMagnitude, 1);
						}
					}
					else{
						if(curve > 0){
							d->TankDrive(-1, outputMagnitude + 1 + outputMagnitude);
						}
						else{
							d->TankDrive(outputMagnitude + 1 + outputMagnitude, -1);
						}
					}
				}
		}
		else
			d->TankDrive(leftJoystick, rightJoystick);

		if(leftJoystick->GetTrigger())
			d->ShiftGear(LOW_GEAR);
		if(rightJoystick->GetTrigger())
			d->ShiftGear(HIGH_GEAR);

		// Gear controls
		if(xbox->GetAButton())
			claw->PegPlacementMode();
		if(xbox->GetBButton())
			claw->GroundMode();
		if(xbox->GetXButton())
			claw->TravelMode();
		if(xbox->GetYButton()) {
			if(claw->IsOpen())
				claw->OpenClaw();
			else
				claw->CloseClaw();
		}
		// Climber controls
		climbvalue = fabs(xbox->GetY(frc::GenericHID::JoystickHand::kLeftHand));
		if(xbox->GetStartButton())
			climb->GrabRope();
		if(xbox->GetBackButton())
			climb->ReleaseRope();

		if(climbvalue > 0.1)
			climb->ClimbRope(climbvalue);
		else
			climb->Stop();

		DashboardUpdates();
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void DashboardUpdates()
	{
		frc::SmartDashboard::PutNumber("Left Encoder", leftMotor->GetSpeed());
		frc::SmartDashboard::PutNumber("Right Encoder", rightMotor->GetSpeed());
		frc::SmartDashboard::PutBoolean("Peg Switch", claw->PegPresent());
		frc::SmartDashboard::PutBoolean("Gear Switch", claw->GearPresent());
		frc::SmartDashboard::PutBoolean("Drum Switch", climb->IsIndexed());
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto Mode";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
