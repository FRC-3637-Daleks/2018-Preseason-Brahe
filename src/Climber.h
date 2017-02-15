/*
 * Climber.h
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */

#pragma once
#include <Brahe.h>

class Climber
{
    public:
		Climber(int climbMotor, int piston);
		Climber(CANTalon *climbMotor, Solenoid *piston);
		Climber(CANTalon &climbMotor, Solenoid &piston);
		~Climber();
		void SwitchOn();
		void SwitchOff();
		void Up(double speed);
		void Stop();
		void Play();


    private:
		CANTalon *m_climb;
		Solenoid *m_piston;
		bool m_needFree;
};

