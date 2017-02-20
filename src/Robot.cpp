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
	cs::UsbCamera *usbCamera0, *usbCamera1;
	cs::MjpegServer *mjpegServer0;
	cs::CvSink *cvSink0;

	void
	RobotInit()
	{
		leftMotor   = new CANTalon(LEFT_DRIVEMOTOR);
		leftSlave   = new CANTalon(LEFT_SLAVEMOTOR);
		rightMotor  = new CANTalon(RIGHT_DRIVEMOTOR);
		rightSlave  = new CANTalon(RIGHT_SLAVEMOTOR);
		climbMotor  = new CANTalon(CLIMB_MOTOR);
		climbPiston = new Solenoid(PCM_ID, CLIMB);

		usbCamera0  = new cs::UsbCamera("USB Camera 0", 0);
		if(usbCamera0) {
			mjpegServer0 = new cs::MjpegServer("serve_USB Camera 0", 1181);
			cvSink0      = new cs::CvSink("opencv_USB Camera 0");
			mjpegServer0->SetSource(*usbCamera0);
			cvSink0->SetSource(*usbCamera0);
		}
		usbCamera1 = new cs::UsbCamera("USB Camera 1", 1);

		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		xbox          = new XboxController(XBOX_CONTROLS);

		c = new Compressor(PCM_ID);
		d = new DalekDrive(leftMotor, leftSlave, rightMotor, rightSlave, SHIFTER);
		claw  = new Claw(PISTON, PIVOT, ARM, GEAR_SWITCH, PEG_SWITCH);
		climb = new Climber(climbMotor, climbPiston, DRUM_SWITCH, CLIMB_SWITCH);
	}

	void
	AutonomousInit()
	{
		c->Start();
		claw->TravelMode();
		d->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void
	AutonomousPeriodic()
	{
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

			if ((outputMagnitude + curve) <= 1 && (outputMagnitude - curve) >= -1)
				d->TankDrive(outputMagnitude + curve, outputMagnitude - curve);
			else {
				if (outputMagnitude > 0) {
					if (curve > 0)
						d->TankDrive(1, (2*outputMagnitude - 1));
					else
						d->TankDrive((2*outputMagnitude - 1), 1);
				}
				else {
					if (curve > 0)
						d->TankDrive(-1, (2*outputMagnitude + 1));
					else
						d->TankDrive((2*outputMagnitude + 1), -1);
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

		if(claw->IsOpen() && claw->GearPresent())
			claw->CloseClaw();

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
			climb->ClimbRope(climbvalue);
		else
			climb->Stop();

		// Camera controls
		if (xbox->GetBumper(frc::GenericHID::JoystickHand::kRightHand)) {
			mjpegServer0->SetSource(nullV);
			cvSink0->SetSource(nullV);
			mjpegServer0->SetSource(*usbCamera0);
			cvSink0->SetSource(*usbCamera0);
		}
		
		if (xbox->GetBumper(frc::GenericHID::JoystickHand::kLeftHand)) {
			if(usbCamera1) {
				mjpegServer0->SetSource(nullV);
				cvSink0->SetSource(nullV);
				mjpegServer0->SetSource(*usbCamera1);
				cvSink0->SetSource(*usbCamera1);
			}
		}

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
		frc::SmartDashboard::PutBoolean("Claw Open", claw->IsOpen());
		frc::SmartDashboard::PutBoolean("At Top", climb->IsAtTop());
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();

};

START_ROBOT_CLASS(Robot)
