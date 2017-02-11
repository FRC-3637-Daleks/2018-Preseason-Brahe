/*
 * Climber.cpp
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */
#include <CANTalon.h>
#include <Solenoid.h>
#include <WPILib.h>

class Climber{
private:
	CANTalon *Climb;
	Solenoid *Piston;
	XboxController *xbox;
	bool midair;

public:
	Climber(int c, int p, int xb){
		xbox = new XboxController(xb);
		Climb = new CANTalon(c);
		Piston = new Solenoid(p);
		Piston->Set(false);
		midair = false;
	}
	void Switch(){
		Piston->Set(!Piston->Get());

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
		if(xbox->GetY(GenericHID::JoystickHand::kRightHand) != 0 && Piston->Get()){
			Up(xbox->GetY(GenericHID::JoystickHand::kRightHand));
		}
		if(xbox->GetAButton()){
			Stop();
		}
	}
};



