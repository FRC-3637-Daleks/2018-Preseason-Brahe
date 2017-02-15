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
		Climber(int climbMotor, int piston, int magswitch);
		Climber(CANTalon *climbMotor, Solenoid *piston, int magswitch);
		Climber(CANTalon &climbMotor, Solenoid &piston, int magswitch);
		~Climber();
		void GrabRope();
		void ReleaseRope();
		void ClimbRope(double speed);
		bool IsIndexed();
		void MoveToIndex();
		void Stop();

    private:
		CANTalon *m_climb;
		Solenoid *m_piston;
		DigitalInput *m_index;
		bool m_needFree;
};

