/*
 * Climber.cpp
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */
#include <CANTalon.h>;
#include <DoubleSolenoid.h>
#include <WPILib.h>

class Climber{
private:
	CANTalon *Climb;
	DoubleSolenoid *Piston;
	XboxController *xbox;
	bool midair;

public:
	Climber(int c, int p, int f, int r, XboxController xb){
		xbox = xb;
		Climb = new CANTalon(c);
		Piston = new DoubleSolenoid(p, f, r);
		Piston->Set(DoubleSolenoid::Value::kOff);
		midair = false;
	}
	void Switch(){
		if (Piston->Get() == (DoubleSolenoid::Value::kForward)){
			Piston->Set(DoubleSolenoid::Value::kOff);
			Piston->Set(DoubleSolenoid::Value::kReverse);
		}
		else if (Piston->Get() == (DoubleSolenoid::Value::kReverse)){
			Piston->Set(DoubleSolenoid::Value::kOff);
			Piston->Set(DoubleSolenoid::Value::kForward);
		}
		else{
			Piston->Set(DoubleSolenoid::Value::kForward);
		}

	}
	void Up(double speed){
		Climb -> Set(speed);
	}
	void Stop(){
		Climb -> Set(0);
	}
	void Play(){
		if (xbox->GetYButton()){
			//Up(5);
			//midair = true;
		}
		if (xbox->GetBumper(GenericHID::JoystickHand::kRightHand) && !midair){
			void Switch();
		}
		if(xbox->GetY(GenericHID::JoystickHand::kRightHand) != 0 && Piston->Get() == (DoubleSolenoid::Value::kForward)){
			Up(xbox->GetY(GenericHID::JoystickHand::kRightHand));
		}
		if(xbox->GetAButton()){
			Stop();
		}
	}
};



