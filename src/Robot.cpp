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
#include <Target.h>
#include <IRsensor.h>
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
	Relay *lightswitch;
	Target *targeter;
	IRsensor *irdst;
	cv::Rect r1, r2;
	int startPosition;
	bool autoGear;
	bool testFinal;
	int autoStage;
	bool forwardDone, turnDone, pegDone, clawDone, backupDone, midTurn, midGo;
	int clawMode, autonomousAlign;
	double exposure;
	int autoPos;
	void
	RobotInit()
	{
		exposure = 1;
		leftMotor     = new CANTalon(LEFT_DRIVEMOTOR);
		leftSlave     = new CANTalon(LEFT_SLAVEMOTOR);
		rightMotor    = new CANTalon(RIGHT_DRIVEMOTOR);
		rightSlave    = new CANTalon(RIGHT_SLAVEMOTOR);
		climbMotor    = new CANTalon(CLIMB_MOTOR);
		climbPiston   = new Solenoid(PCM_ID, CLIMB_SOLENOID);
		targeter      = new Target(FRONT_CAMERA, REAR_CAMERA);
		leftJoystick  = new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		xbox          = new XboxController(XBOX_CONTROLS);
		c             = new Compressor(PCM_ID);
		lightswitch   = new Relay(LIGHT_SWITCH);
		d             = new DalekDrive(leftMotor, leftSlave, rightMotor, rightSlave, SHIFTER_SOLENOID);
		claw          = new Claw(PISTON_SOLENOID, PIVOT_SOLENOID, ARM_SOLENOID, GEAR_SWITCH, CAMERA_SERVO);
		climb         = new Climber(climbMotor, climbPiston, DRUM_SWITCH, CLIMB_SWITCH);
		irdst         = new IRsensor(IR_SENSOR_LEFT, IR_SENSOR_RIGHT);

		autoLocation.AddObject("Center", CENTER_POSITION);
		autoLocation.AddDefault("Left", LEFT_POSITION);
		autoLocation.AddObject("Right", RIGHT_POSITION);
		frc::SmartDashboard::PutData("Autonomous Starting Location", &autoLocation);

		autoLocation2.AddObject("Boiler Side", 1);
		autoLocation2.AddDefault("Gear Side", 0);
		frc::SmartDashboard::PutData("Autonomous Starting Location 2", &autoLocation2);
		autoMode.AddObject("Gear Placement", GEAR_HANDLING);
		autoMode.AddDefault("Mobility", MOBILITY);
		frc::SmartDashboard::PutData("Autonomous Mode", &autoMode);

		autoClaw.AddObject("Close Claw", 1);
		autoClaw.AddDefault("Open Claw", 0);
		frc::SmartDashboard::PutData("Autonomous Claw Mode", &autoClaw);

		autoAlign.AddObject("With Camera", 1);
		autoAlign.AddDefault("Dead Reckoning", 0);
		frc::SmartDashboard::PutData("Align Mode", &autoAlign);
		lw->AddActuator("light switch", "light switch", lightswitch);
	}

	void
	RobotPeriodic()
	{
	}

	void
	DisabledInit()
	{
		d->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void
	DisabledPeriodic()
	{
		autoStage = 0;
		d->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void
	AutonomousInit()
	{
	    // get autonomous start position
		forwardDone	 = false;
		turnDone	 = true;
		pegDone  	 = true;
		clawDone	 = true;
		backupDone	 = true;
		midTurn = true;
		midGo = true;
		clawMode 	 = autoClaw.GetSelected();
		autonomousAlign = autoAlign.GetSelected();
		std::string loc = autoLocation.GetSelected();
		autoPos = autoLocation2.GetSelected();
		if(loc == LEFT_POSITION)
			startPosition = 0;
		else if (loc == CENTER_POSITION)
			startPosition = 1;
		else if (loc == RIGHT_POSITION)
			startPosition = 2;
		else
			startPosition = 1;

		// get autonomous mode of operation
		autoGear = (autoMode.GetSelected() == GEAR_HANDLING);
		if(!autoGear && (loc == CENTER_POSITION))
			autoGear = true;

		autoStage = 0;

		// make sure robot is properly setup
		c->Start();
		claw->TravelMode();
		d->SetLeftRightMotorOutputs(0.0, 0.0);
		leftMotor->SetPosition(0); rightMotor->SetPosition(0);
		d->ShiftGear(HIGH_GEAR);
		lightswitch->Set(Relay::kReverse);
        targeter->switchCam(FRONT_CAMERA);
        targeter->autoExposure();
	}

	bool
	AutonomousMobility(double dist, double lSpeed, double rSpeed)
	{
		// Thing 1
		double travel_distance = dist;
		// Thing 2
		// static double travel_distance = XXXXX;
		double ldist, rdist, distance;

        ldist = abs(leftMotor->GetEncPosition());
        rdist = abs(rightMotor->GetEncPosition());
        distance = (ldist >  rdist) ? ldist : rdist;

        if(distance < travel_distance){
        	d->SetLeftRightMotorOutputs(lSpeed, rSpeed);
        	return false;
        }
        else {
        	d->SetLeftRightMotorOutputs(0.0, 0.0);
        	rightMotor->SetPosition(0);
        	leftMotor->SetPosition(0);
        	return true;
        }


	}

	void
	AutonomousGearPlacement()
	{
		// Thing 1 - note center distance is unknown at this point
		static double arc_distance = 12300, center_distance = 8000;
		// Thing 2
		//static double arc_distance = 15800, center_distance = 10000;
		static bool target_acquired = false;
		static double backup_distance;
		double ldist, rdist, distance, req_distance;
		// Thing 1
		double arc_speed = -0.66, arc_curve = 0.56, center_speed = -0.5;
		// Thing 2
		//double arc_speed = -0.6, arc_curve = 0.75, center_speed = -0.5;
		double target_angle, target_dist;

		ldist = abs(leftMotor->GetEncPosition());
		rdist = abs(rightMotor->GetEncPosition());
		distance = (ldist >  rdist) ? ldist : rdist;

		req_distance = arc_distance;
		if(startPosition == 1)
			req_distance = center_distance;

		if(autoStage == 0) {
			// Make the initial arc or travel straight to get close
			if(distance < req_distance) {
				if(startPosition == 1)
					d->SetLeftRightMotorOutputs(center_speed, center_speed);
				else if(startPosition == 0)
					d->Drive(arc_speed, -arc_curve);
				else if(startPosition == 2)
					d->Drive(arc_speed, arc_curve);
			}
			else {
				//HACK: skip to stage 5 until we get this working
				autoStage = 5;
				d->SetLeftRightMotorOutputs(0.0, 0.0);
				d->ShiftGear(LOW_GEAR);
			}
		}
		else if(autoStage == 1) {
			// do we have a target
			target_acquired = targeter->isAquired();
			target_angle = targeter->targetAngle();
			if(!target_acquired) {
				if(target_angle < 0.0)
					d->SetLeftRightMotorOutputs(-0.25, 0.25);
				else if(target_angle > 0.0)
					d->SetLeftRightMotorOutputs(0.25, -0.25);
				else {
					d->SetLeftRightMotorOutputs(0.0, 0.0);
					target_acquired = true;
					autoStage = 2;
				}
			}
			else {
				d->SetLeftRightMotorOutputs(0.0, 0.0);
				autoStage = 2;
			}
		}
		else if(autoStage == 2) {
			target_dist = targeter->targetDistance();
			if(target_dist)
				d->SetLeftRightMotorOutputs(-0.25, -0.25);
			else {
				d->SetLeftRightMotorOutputs(0.0, 0.0);
				autoStage = 3;
			}
		}
		else if(autoStage == 3) {
            claw->DeployMode();
            Wait(0.1);
            claw->OpenClaw();
            Wait(0.1);
            d->SetLeftRightMotorOutputs(0.25, 0.25);
            backup_distance = distance - 2000;
            autoStage = 4;
        }
        else if(autoStage == 4) {
            // backup from the peg for about a foot
            if(distance > backup_distance)
                d->SetLeftRightMotorOutputs(0.25, 0.25);
            else {
                d->SetLeftRightMotorOutputs(0.0, 0.0);
                autoStage = 5;
            }
        }
        else
        	d->SetLeftRightMotorOutputs(0.0, 0.0);
	}
	bool
	CameraAlignForward(double areaDone, double lSpeed, double rSpeed){
		double l = lSpeed;
		double r = rSpeed;
		int done = 1000;
		int area = targeter->getArea();
		int dev = 95 - targeter->getMidX();
		frc::SmartDashboard::PutNumber("Area", targeter->getArea());
		frc::SmartDashboard::PutNumber("Midpoint", targeter->getMidX());

		/*if(dev>0){
			r+=-((double)(dev/10))/100;
		}
		else if(dev<0){
			l+=-((double)(dev/10))/100;
		}*/
		if(!(area>areaDone)){
			d->SetLeftRightMotorOutputs(l, r);
			return false;
		}
		else{
			return true;
		}

	}
	bool
	CameraAlignRotate(double lSpeed, double rSpeed){
		double l = abs(lSpeed);
		double r = abs(rSpeed);
		int done = 1000;
		int dev = 160 - targeter->getMidX();
		if(dev>50){
			d->SetLeftRightMotorOutputs(l, -r);
			return false;
		}
		else if(dev<-50){
			d->SetLeftRightMotorOutputs(-l, r);
			return false;
		}
		else{
			return true;
		}

	}

	void
	AutonomousPeriodic()
	{
		frc::SmartDashboard::PutNumber("Start Position", startPosition);
		frc::SmartDashboard::PutNumber("claw mode", clawMode);
		frc::SmartDashboard::PutNumber("auto align", autonomousAlign);
		frc::SmartDashboard::PutNumber("Right Encoder", rightMotor->GetEncPosition());
		frc::SmartDashboard::PutNumber("Left Encoder", leftMotor->GetEncPosition());

		if(!autoGear){
			AutonomousMobility(10600, -.5, -.5);
		}
		else{
			if(startPosition == 1){
				if(!forwardDone){
					if(autonomousAlign == 0){
						forwardDone = AutonomousMobility(9000, -.5, -.5);
					}
					else{
						forwardDone = CameraAlignForward(5500, -.5, -.5);
					}
					if (forwardDone){
						clawDone = false;
					}
				}
				if (!clawDone){
					if(clawMode == 0){
						Wait(.2);
						claw->DeployMode();
						Wait(.5);
						claw->OpenClaw();
						rightMotor->SetPosition(0);
						leftMotor->SetPosition(0);
						Wait(1);
						backupDone = false;

					}
					clawDone = true;

				}
				if (!backupDone){

					backupDone = AutonomousMobility(3000, 1, 1);
				}
			}
			else{
				if(!forwardDone){

					if(autoPos==0){
						//gear chute
						forwardDone = AutonomousMobility(8000, -1, -1);
						//forwardDone = AutonomousMobility(8800, -.5, -.5);
					}
					else{
						//boiler
						forwardDone = AutonomousMobility(7800, -1, -1);
					}
					if (forwardDone){
						Wait(.2);
						turnDone = false;

					}

				}
				if(!turnDone){
					if(startPosition == 2){
						//right
						if(autonomousAlign == 0){
							turnDone = AutonomousMobility(2100, 1, -1);
							//turnDone = AutonomousMobility(2400, 1, -1);
						}
						else{
							turnDone = CameraAlignRotate(0.5, -0.5);
						}
					}
					else{
						//left
						if(autonomousAlign == 0){
							turnDone = AutonomousMobility(2400, -.5, .5);
						}
						else{
							turnDone = CameraAlignRotate(-0.5, 0.5);
						}
					}
					if (turnDone){
						Wait(.1);
						pegDone = false;
					}
				}

				if (!pegDone){

					if(autonomousAlign == 0){
						pegDone = AutonomousMobility(6500, -1, -1);
					}
					else{
						pegDone = CameraAlignForward(2000, -.5, -.5);
					}
					if (pegDone){
						clawDone = false;
					}
				}
				if(!clawDone){
					if(clawMode == 0){
						claw->DeployMode();
						Wait(.2);
						claw->OpenClaw();
						backupDone = false;

					}
					Wait(.5);
					clawDone = true;
				}
				if(!backupDone){
					backupDone = AutonomousMobility(5000, 1, 1);
					if(backupDone){
						midTurn = false;
						claw->TravelMode();
						Wait(.1);
					}
				}
				if(!midTurn){
					if(startPosition == 2){
						//right

							midTurn = AutonomousMobility(2700, -1, 1);

					}
					else{
						//left

							midTurn = AutonomousMobility(2700, 1, -1);


					}
					if (midTurn){
						Wait(.1);
						midGo = false;
						d->ShiftGear(LOW_GEAR);
					}
				}
				if(!midGo){

					midGo = AutonomousMobility(14000, -1, -1);
				}


			}
		}

		return;
    }

	void
	TeleopInit()
	{
		c->Start();
		claw->TravelMode();
		d->SetLeftRightMotorOutputs(0.0, 0.0);
		targeter->autoExposure();
		lightswitch->Set(Relay::kForward);
		testFinal = false;

	}

	void
	TeleopPeriodic()
	{
		static bool sawButtonRelease = true;
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
		if(xbox->GetStartButton()) {
			climb->MoveToIndex();
			climb->GrabRope();
		}
		if(xbox->GetBackButton())
			climb->ReleaseRope();

		climbvalue = fabs(xbox->GetY(frc::GenericHID::JoystickHand::kLeftHand));
		if(climbvalue > 0.3)
			climb->ClimbRope();
		else
			climb->Stop();

		// Camera controls
		if((xbox->GetBumper(frc::GenericHID::JoystickHand::kRightHand)) ||
		   (leftJoystick->GetTop())) {
			targeter->switchCam(FRONT_CAMERA);
		}
		
		if((xbox->GetBumper(frc::GenericHID::JoystickHand::kLeftHand)) ||
		   (rightJoystick->GetTop())) {
			targeter->switchCam(REAR_CAMERA);
		}
		if(leftJoystick->GetRawButton(3)){
			rightMotor->SetPosition(0);
		}
		if(rightJoystick->GetRawButton(3)){
			leftMotor->SetPosition(0);
		}

		if(leftJoystick->GetRawButton(2))
			claw->SetCameraView(GROUND_VIEW_POSITION);
		if(rightJoystick->GetRawButton(2))
			claw->SetCameraView(FRONT_VIEW_POSITION);

		if(xbox->GetPOV(0)==0){
			claw->SetCameraView(FRONT_VIEW_POSITION);




			//claw->SetCameraView(claw->GetCameraView()-.01);

		}
		if(xbox->GetPOV(0)==180){
			claw->SetCameraView(GROUND_VIEW_POSITION);


			//claw->SetCameraView(claw->GetCameraView()+.01);
		}

		if(xbox->GetPOV(0)==90 && !testFinal){
			//targeter->autoExposure();
			//lightswitch->Set(Relay::kForward);
			//d->ShiftGear(LOW_GEAR);
			//testFinal = AutonomousMobility(10000, -1, -1);

		}
		if(xbox->GetPOV(0)==270){

				testFinal = false;


		}

		if(claw->GearPresent()){
			lightswitch->Set(Relay::kReverse);
		}
		else{
			lightswitch->Set(Relay::kForward);
		}
		++teleopCnt;
		if((teleopCnt % 10) == 0)
			DashboardUpdates();
	}

	void
	TestInit()
	{
		c->Start();
		claw->TravelMode();
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
		frc::SmartDashboard::PutBoolean("Gear Switch", claw->GearPresent());
		frc::SmartDashboard::PutBoolean("Drum Switch", climb->IsIndexed());
		frc::SmartDashboard::PutBoolean("Claw Open", claw->IsOpen());
		frc::SmartDashboard::PutBoolean("At Top", climb->IsAtTop());
		frc::SmartDashboard::PutNumber("Left Encoder", leftMotor->GetSpeed());
		frc::SmartDashboard::PutNumber("Right Encoder", rightMotor->GetSpeed());
		frc::SmartDashboard::PutNumber("Distance", rightMotor->GetEncPosition());
		frc::SmartDashboard::PutNumber("Servo Angle", claw->GetCameraView());
		frc::SmartDashboard::PutNumber("Area", targeter->getArea());
		frc::SmartDashboard::PutNumber("Midpoint", targeter->getMidX());
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> autoLocation;
	frc::SendableChooser<int> autoLocation2;
	frc::SendableChooser<std::string> autoMode;
	frc::SendableChooser<int> autoClaw;
	frc::SendableChooser<int> autoAlign;

};

START_ROBOT_CLASS(Robot)
