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
	int camnum = 0;
	Compressor *c;
	CANTalon *leftMotor, *rightMotor, *leftSlave, *rightSlave;
	CANTalon *climbMotor;
	Joystick *leftJoystick, *rightJoystick;
	XboxController *xbox;
	DalekDrive *d;
	Claw *claw;
	Climber *climb;
	Solenoid *climbPiston;


	void
	RobotInit()
	{
		leftMotor  = new CANTalon(LEFT_DRIVEMOTOR);
		leftSlave  = new CANTalon(LEFT_SLAVEMOTOR);
		rightMotor = new CANTalon(RIGHT_DRIVEMOTOR);
		rightSlave = new CANTalon(RIGHT_SLAVEMOTOR);
		climbMotor = new CANTalon(CLIMB_MOTOR);
		climbPiston = new Solenoid(PCM_ID, CLIMB);
		CameraServer::GetInstance()->StartAutomaticCapture();
		CameraServer::GetInstance()->StartAutomaticCapture();

		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		xbox          = new XboxController(XBOX_CONTROLS);
#ifdef CAMERA_INUSE
		std::thread visionThread(VisionThread);
		visionThread.detach();
#endif

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

	void
	TeleopInit()
	{
		autoSelected = chooser.GetSelected();
		c->Start();
		claw->TravelMode();
	}

	void
	TeleopPeriodic()
	{
		static bool sawButtonRelease = true;
		bool useArcade, useDrive, toggleClaw;
		double climbvalue;

		useArcade = (leftJoystick->GetZ() == -1.0);
		useDrive  = (rightJoystick->GetZ() == -1.0);

		// Drive controls
		if (useArcade)
			d->ArcadeDrive(leftJoystick);
		else if (useDrive) {
			double outputMagnitude = rightJoystick->GetY();
			double curve = -leftJoystick->GetX() / 2;

			if (outputMagnitude + curve <= 1 && outputMagnitude - curve >= -1)
				d->TankDrive(outputMagnitude + curve, outputMagnitude - curve);
			else {
				if (outputMagnitude > 0) {
					if (curve > 0)
						d->TankDrive(1, outputMagnitude - 1 + outputMagnitude);
					else
						d->TankDrive(outputMagnitude -1 + outputMagnitude, 1);
				}
				else {
					if (curve > 0)
						d->TankDrive(-1, outputMagnitude + 1 + outputMagnitude);
					else
						d->TankDrive(outputMagnitude + 1 + outputMagnitude, -1);
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

		toggleClaw = xbox->GetYButton();
		if(sawButtonRelease && toggleClaw) {
			if(claw->IsOpen())
				claw->CloseClaw();
			else
				claw->OpenClaw();
			sawButtonRelease = false;
		}
		else if (!toggleClaw)
			sawButtonRelease = true;

		// Climber controls
		climbvalue = fabs(xbox->GetY(frc::GenericHID::JoystickHand::kLeftHand));
		if(xbox->GetStartButton())
			climb->GrabRope();
		if(xbox->GetBackButton())
			climb->ReleaseRope();

		if(climbvalue > 0.3)
			climb->ClimbRope(climbvalue);
		else
			climb->Stop();

		// Camera controls
		if (xbox->GetBumper(frc::GenericHID::JoystickHand::kRightHand))
			camnum = 0;
		
		if (xbox->GetBumper(frc::GenericHID::JoystickHand::kLeftHand))
			camnum = 1;

		DashboardUpdates();
	}

	void
	TestPeriodic()
	{
		lw->Run();
	}

	void
	DashboardUpdates()
	{
		frc::SmartDashboard::PutNumber("Left Encoder", leftMotor->GetSpeed());
		frc::SmartDashboard::PutNumber("Right Encoder", rightMotor->GetSpeed());
		frc::SmartDashboard::PutBoolean("Peg Switch", claw->PegPresent());
		frc::SmartDashboard::PutBoolean("Gear Switch", claw->GearPresent());
		frc::SmartDashboard::PutBoolean("Drum Switch", climb->IsIndexed());
		frc::SmartDashboard::PutBoolean("Claw Open????", claw->IsOpen());
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();

#ifdef CAMERA_INUSE
	static void 
	VisionThread()
	{
		cv::Mat source;
		cv::Mat output;

		cs::UsbCamera GearCam = CameraServer::GetInstance()->StartAutomaticCapture("gearcam",   0);
		cs::UsbCamera ClimbCam = CameraServer::GetInstance()->StartAutomaticCapture("climbcam", 1);

		GearCam.SetResolution(640, 480);
		ClimbCam.SetResolution(640, 480);

		cs::CvSink gearSink = CameraServer::GetInstance()->GetVideo("gearcam");
		cs::CvSink climbSink = CameraServer::GetInstance()->GetVideo("climbcam");
		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Output", 640, 480);

		while(true) {
        		if (camnum == 0)
				gearSink.GrabFrame(source);
			else
        			climbSink.GrabFrame(source);

        		outputStreamStd.PutFrame(source);
        	}
	}
#endif

};

START_ROBOT_CLASS(Robot)
