/*
 * Climber.cpp
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */
#include <CANTalon.h>
#include <Solenoid.h>
#include <WPILib.h>
#include <Climber.h>


	CANTalon *Climb;
	Solenoid *Piston;
	XboxController *xbox;
	bool midair;



	Climber::Climber(int c, int p, int xb){
		xbox = new XboxController(xb);
		Climb = new CANTalon(c);
		Piston = new Solenoid(p);
		Piston->Set(false);
		midair = false;
	}
	void Climber::Switch(){
		Piston->Set(!Piston->Get());

	}
	void Climber::Up(double speed){
		Climb -> Set(speed);
	}
	void Climber::Stop(){
		Climb -> Set(0);
	}
	void Climber::Play(){
		/*if (xbox->GetYButton()){
			Up(5);
			midair = true;
		}*/
		if (xbox->GetBumper(GenericHID::JoystickHand::kLeftHand) && !midair){
			void Switch();
		}
		if(Piston->Get()){
			Up(xbox->GetY(GenericHID::JoystickHand::kLeftHand));
		}
		else{
			Stop();
		}

	}




