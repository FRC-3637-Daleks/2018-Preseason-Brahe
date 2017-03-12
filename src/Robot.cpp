#include <iostream>
#include <memory>
#include <string>

#include <math.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/Features2d.hpp>

#include <WPILib.h>
#include <CANTalon.h>
#include <Brahe.h>
#include <DalekDrive.h>
#include <Claw.h>
#include <Climber.h>
#include <Target.h>
#include <GripPipeline.h>

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
	Target *targeter;
	/*cs::UsbCamera *usbCamera0, *usbCamera1;
	cs::MjpegServer *mjpegServer0;
	cs::CvSink *cvSink0;
    cs::CvSource cvSource0;*/
	grip::GripPipeline gp;
	cv::Rect r1, r2;
	std::vector<std::vector<cv::Point>> *dContours;
    int startPosition;
    int autoStage;
    bool isCam0;

	void
	RobotInit()
	{
		leftMotor   = new CANTalon(LEFT_DRIVEMOTOR);
		leftSlave   = new CANTalon(LEFT_SLAVEMOTOR);
		rightMotor  = new CANTalon(RIGHT_DRIVEMOTOR);
		rightSlave  = new CANTalon(RIGHT_SLAVEMOTOR);
		climbMotor  = new CANTalon(CLIMB_MOTOR);
		climbPiston = new Solenoid(PCM_ID, CLIMB);
		targeter 	= new Target(0, 1);
		/*usbCamera0  = new cs::UsbCamera("USB Camera 0", 0);
		if(usbCamera0) {
			mjpegServer0 = new cs::MjpegServer("serve_USB Camera 0", 1181);
			cvSink0      = new cs::CvSink("opencv_USB Camera 0");
			mjpegServer0->SetSource(*usbCamera0);
			cvSink0->SetSource(*usbCamera0);

			usbCamera0->SetResolution(640, 480);
			usbCamera0->SetExposureManual(.75);
			usbCamera0->SetBrightness(90);
			isCam0 = true;
		}
		usbCamera1 = new cs::UsbCamera("USB Camera 1", 1);

		if(usbCamera1) {
			usbCamera1->SetResolution(640, 480);
			usbCamera1->SetBrightness(45);
			usbCamera1->SetExposureAuto();
		}
		cvSource0     = CameraServer::GetInstance()->PutVideo("Output", 320, 240);
		*/

		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		xbox          = new XboxController(XBOX_CONTROLS);
		c             = new Compressor(PCM_ID);
		d             = new DalekDrive(leftMotor, leftSlave, rightMotor, rightSlave, SHIFTER);
		claw          = new Claw(PISTON, PIVOT, ARM, GEAR_SWITCH);
		climb         = new Climber(climbMotor, climbPiston, DRUM_SWITCH, CLIMB_SWITCH);
	}

	void
	DisableInit()
	{
		d->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void
	DisablePeriodic()
	{
		d->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void
	AutonomousInit()
	{
        // get autonomous start position
        startPosition = 1;
        autoStage = 0;
		c->Start();
		claw->TravelMode();
		d->SetLeftRightMotorOutputs(0.0, 0.0);
		leftMotor->SetPosition(0);
		rightMotor->SetPosition(0);
		d->ShiftGear(HIGH_GEAR);
	}

	void
	AutonomousPeriodic()
	{
        static double target_acquisition_distance = 6720; // ~4.6 ft
        static double backup_distance;
        static bool target_acquired = false;
        double ldist, rdist, distance;
        double targetAngle, targetDistance;

		frc::SmartDashboard::PutNumber("Autonomous Stage", autoStage);
        ldist = abs(leftMotor->GetPosition());
        rdist = abs(rightMotor->GetPosition());
        distance = (ldist >  rdist) ? ldist : rdist;
        frc::SmartDashboard::PutNumber("Distance traveled", distance);
        targeter->switchCam0();
        if(autoStage == 0) {
            // initial start stage - move to target acquisition point
            // for now we will assume that we are oriented in such a
            // way that 60" travel is sufficient distance to get where
            // can find the target
            if (distance < target_acquisition_distance)
                d->SetLeftRightMotorOutputs(-0.45, -0.5);
            else {
                d->SetLeftRightMotorOutputs(0.0, 0.0);
                autoStage = 1;
            }
        }
        else if(autoStage == 1) {
            // roughly 80" from starting point, now need to find
            // the visual targets to orient the bot correctly this
            // will depend on our starting position, left, right or
            // center.  Center should require no change as the 
            // target should be in view and ideally straight ahead.
        	// Starting on the left will require us to turn left and
        	// similarly starting on the right will require us to turn right.
        	// How much is a bit of a crap shoot.  If grip code is working
            // we can run this in a polling loop
        	d->ShiftGear(LOW_GEAR);
            targetAngle = 0.0; targetDistance = 0.0; target_acquired = 1;
            if(!target_acquired) {
                // make a call to see if target found here <TBD>
                // setting targetAngle and targetDistance
            	targeter->process();
            	targetAngle = targeter->targetAngle();
            	targetDistance = targeter->targetDistance();

                switch(startPosition) {
                    case 0: // left side
                        d->SetLeftRightMotorOutputs(-0.25, 0.25);
                        break;
                    case 1: // center
                        d->SetLeftRightMotorOutputs(0.25, 0.25);
                        break;
                    case 2: // right side
                        d->SetLeftRightMotorOutputs(0.25, -0.25);
                        break;
                    default:
                        d->SetLeftRightMotorOutputs(0.0, 0.0);
                        break;
                }
                if (abs(targetAngle) > 5){
                	target_acquired = true;
              	}
            }
            else
                autoStage = 2;
        }
        else if(autoStage == 2) {
            // We in theory have found/aligned with the target so
            // we can at this point trundle forward to the peg
            // We need to keep the target centered in our field of
            // vision, so again the grip code will be used to provide
            // the same 2 pieces of information angle to target and
            // distance to target
            // make call to get target distance and angle <TBD>
            targetAngle = 0.0; targetDistance = 0.0;
            if (targetDistance > 5) {
                if (targetAngle > 0)
                    // off to the right, speed should be set depending on
                    // off center we are
                    d->SetLeftRightMotorOutputs(0.25, -0.25);
                else if (targetAngle == 0)
                    d->SetLeftRightMotorOutputs(0.25, 0.25);
                else 
                    // off to the right, speed should be set depending on
                    // off center we are
                    d->SetLeftRightMotorOutputs(-0.25, 0.25);
            }
            else {
                d->SetLeftRightMotorOutputs(0.0, 0.0);
                autoStage = 3;
            }
        }
        else if(autoStage == 3) {
            // We are now at the peg or so we believe, so do the peg deployment
            // and backup
            claw->DeployMode();
            Wait(0.1);
            claw->OpenClaw();
            Wait(0.1);
            d->SetLeftRightMotorOutputs(0.25, 0.25);
            backup_distance = distance - 915;
            autoStage = 4;
        }
        else if(autoStage == 4) {
            // backup from the peg for about a foot
            if(backup_distance > distance)
                d->SetLeftRightMotorOutputs(0.25, 0.25);
            else {
                d->SetLeftRightMotorOutputs(0.0, 0.0);
                autoStage = 5;
            }
        }
        else
            d->SetLeftRightMotorOutputs(0.0, 0.0);
    }

	void
	TeleopInit()
	{
		c->Start();
		claw->TravelMode();
		d->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void
	TeleopPeriodic()
	{
		static bool sawButtonRelease = true;
		static cs::VideoSource nullV;
		static long teleopCnt;
		bool useArcade, toggleClaw;
		double climbvalue;

		useArcade = (leftJoystick->GetZ() == -1.0);

		// Drive controls
		if (useArcade)
			d->ArcadeDrive(leftJoystick);
		else
			d->TankDrive(leftJoystick, rightJoystick);

		if(leftJoystick->GetTrigger())
			d->ShiftGear(LOW_GEAR);
		if(rightJoystick->GetTrigger())
			d->ShiftGear(HIGH_GEAR);

		// Gear controls
		if(xbox->GetAButton())
			claw->DeployMode();
		if(xbox->GetBButton())
			claw->GroundMode();
		if(xbox->GetXButton())
			claw->TravelMode();

		claw->CheckForGear();

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
		if(xbox->GetStartButton()) {
			climb->MoveToIndex();
			climb->GrabRope();
		}
		if(xbox->GetBackButton())
			climb->ReleaseRope();

		if(climbvalue > 0.3)
			climb->ClimbRope(0.9);
		else
			climb->Stop();

		// Camera controls
		/*if((xbox->GetBumper(frc::GenericHID::JoystickHand::kRightHand)) ||
		   (leftJoystick->GetTop())) {
			targeter->switchCam0();
		}
		
		if((xbox->GetBumper(frc::GenericHID::JoystickHand::kLeftHand)) ||
		   (rightJoystick->GetTop())) {
			targeter->switchCam1();
		}*/

		targeter->process();
		//r1 = targeter->getR1();
		//r2 = targeter->getR2();
		++teleopCnt;
		if((teleopCnt % 10) == 0)
			DashboardUpdates();
	}

	void
	TestInit()
	{
		c->Start();
		claw->TravelMode();
		d->SetLeftRightMotorOutputs(0.0, 0.0);

	}

	void
	TestPeriodic()
	{
		lw->Run();
	}

	void
	DashboardUpdates()
	{
		frc::SmartDashboard::PutBoolean("Compressor ", !(c->GetPressureSwitchValue()));
		frc::SmartDashboard::PutNumber("Left Encoder", leftMotor->GetSpeed());
		frc::SmartDashboard::PutNumber("Right Encoder", rightMotor->GetSpeed());
		frc::SmartDashboard::PutBoolean("Gear Switch", claw->GearPresent());
		frc::SmartDashboard::PutBoolean("Drum Switch", climb->IsIndexed());
		frc::SmartDashboard::PutBoolean("Claw Open", claw->IsOpen());
		frc::SmartDashboard::PutBoolean("At Top", climb->IsAtTop());
		/*frc::SmartDashboard::PutNumber("Contour 1X", r1.br().x-r1.width);
		frc::SmartDashboard::PutNumber("Contour 1Y", r1.br().y-r1.height);
		frc::SmartDashboard::PutNumber("Contour 2X", r2.br().x-r2.width);
		frc::SmartDashboard::PutNumber("Contour 2Y", r2.br().y-r2.height);
		frc::SmartDashboard::PutBoolean("cam0", isCam0);
		if(r1.br().x-(r1.width/2) + r2.br().x-(r2.width/2) / 2 < 180){
			frc::SmartDashboard::PutString("Direction", "Go Left");
		}
		else{
			frc::SmartDashboard::PutString("Direction", "Go Right");
		}*/
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();

};

START_ROBOT_CLASS(Robot)
