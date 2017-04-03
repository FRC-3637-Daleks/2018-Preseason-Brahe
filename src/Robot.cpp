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
	int autoStage;
	bool mobilityDone, turnStart, turnDone, clawDone, BACKUP;

	void
	RobotInit()
	{
		BACKUP = false;
		mobilityDone = false;
		clawDone     = false;
		turnDone	 = false;
		turnStart	 = false;
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

		autoMode.AddObject("Gear Placement", GEAR_HANDLING);
		autoMode.AddDefault("Mobility", MOBILITY);
		frc::SmartDashboard::PutData("Autonomous Mode", &autoMode);

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
		mobilityDone = false;
		clawDone     = false;
		turnDone	 = false;
		mobilityDone = false;
		BACKUP		 = false;
		turnStart	 = false;
		std::string loc = autoLocation.GetSelected();
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
	}

	void
	AutonomousMobility()
	{
		// Thing 1
		static double travel_distance = 10600;
		// Thing 2
		// static double travel_distance = XXXXX;
		double ldist, rdist, distance;

        ldist = abs(leftMotor->GetEncPosition());
        rdist = abs(rightMotor->GetEncPosition());
        distance = (ldist >  rdist) ? ldist : rdist;

        if(distance < travel_distance){
        	d->SetLeftRightMotorOutputs(-0.5, -0.5);
        	mobilityDone = false;
        }
        else {
        	d->SetLeftRightMotorOutputs(0.0, 0.0);
        	mobilityDone = true;
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

	void
	AutonomousPeriodic()
	{
		if(!autoGear){
			AutonomousMobility();
		}
		else{
			if(startPosition == 1){
				if(!clawDone){
					AutonomousMobility();
				}
				if (mobilityDone){
					Wait(.2);
					claw->DeployMode();
					Wait(.2);
					claw->OpenClaw();
					Wait(1);
					clawDone = true;
					rightMotor->SetEncPosition(0);
					leftMotor->SetEncPosition(0);
				}
				if (clawDone){
					// Thing 1
					//adding 1345
					static double travel_distance = 1000;
					// Thing 2
					// static double travel_distance = XXXXX;
					double ldist, rdist, distance;

					ldist = abs(leftMotor->GetEncPosition());
					rdist = abs(rightMotor->GetEncPosition());
					distance = (ldist >  rdist) ? ldist : rdist;

					if(distance < travel_distance){
						d->SetLeftRightMotorOutputs(0.3, 0.3);
					}
					else{
						d->SetLeftRightMotorOutputs(0, 0);
					}
					mobilityDone = false;
				}
			}
			else{
				if(!mobilityDone){
					// Thing 1
					static double travel_distance = 8341;
					// Thing 2
					// static double travel_distance = XXXXX;
					double ldist, rdist, distance;

					ldist = abs(leftMotor->GetEncPosition());
					rdist = abs(rightMotor->GetEncPosition());
					distance = (ldist >  rdist) ? ldist : rdist;

					if(distance < travel_distance){
						d->SetLeftRightMotorOutputs(-0.5, -0.5);
						mobilityDone = false;
					}
					else {
						d->SetLeftRightMotorOutputs(0.0, 0.0);
						turnStart = true;
						mobilityDone = true;
					}
				}
				if(turnStart){
					// Thing 1
					//adding 1345
					static double travel_distance = 8901;
					// Thing 2
					// static double travel_distance = XXXXX;
						double ldist, rdist, distance;

						ldist = abs(leftMotor->GetEncPosition());
						rdist = abs(rightMotor->GetEncPosition());
						distance = (ldist >  rdist) ? ldist : rdist;

						if(distance < travel_distance){
							if(startPosition == 2){
								d->SetLeftRightMotorOutputs(0.5, -0.5);
							}
							else{
								d->SetLeftRightMotorOutputs(-0.5, 0.5);
							}
							turnStart = true;
						}
						else {
							d->SetLeftRightMotorOutputs(0.0, 0.0);
							turnStart = false;
							turnDone = true;
						}
				}

				if (turnDone){
					// Thing 1
					//adding 1345
					static double travel_distance = 13717;
					// Thing 2
					// static double travel_distance = XXXXX;
					double ldist, rdist, distance;

					ldist = abs(leftMotor->GetEncPosition());
					rdist = abs(rightMotor->GetEncPosition());
					distance = (ldist >  rdist) ? ldist : rdist;

					if(distance < travel_distance){
						d->SetLeftRightMotorOutputs(-0.5, -0.5);
						turnDone = true;
					}
					else {
						d->SetLeftRightMotorOutputs(0.0, 0.0);
						turnDone = false;
						clawDone = true;
					}

				}
				if(clawDone){
					claw->DeployMode();
					Wait(.2);
					claw->OpenClaw();
					Wait(1);
					BACKUP = true;
					clawDone = false;

				}
				if(BACKUP){
					// Thing 1
					//adding 1345
					static double travel_distance = 8901;
					// Thing 2
					// static double travel_distance = XXXXX;
					double ldist, rdist, distance;

					ldist = abs(leftMotor->GetEncPosition());
					rdist = abs(rightMotor->GetEncPosition());
					distance = (ldist >  rdist) ? ldist : rdist;

					if(distance > travel_distance){
						d->SetLeftRightMotorOutputs(0.3, 0.3);
					}
					else {
						d->SetLeftRightMotorOutputs(0.0, 0.0);
						BACKUP = false;
					}

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
		lightswitch->Set(Relay::kForward);
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

		if(leftJoystick->GetRawButton(2))
			claw->SetCameraView(GROUND_VIEW_POSITION);
		if(rightJoystick->GetRawButton(2))
			claw->SetCameraView(FRONT_VIEW_POSITION);

		if(xbox->GetPOV(0)==0){

			claw->SetCameraView(claw->GetCameraView()-.01);
		}
		if(xbox->GetPOV(0)==180){
			claw->SetCameraView(claw->GetCameraView()+.01);
		}
		if(xbox->GetPOV(0)==90){
			d->SetLeftRightMotorOutputs(-.5, .5);
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
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> autoLocation;
	frc::SendableChooser<std::string> autoMode;

};

START_ROBOT_CLASS(Robot)
