/*
 * Climber.h
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */

#pragma once
#include <Brahe.h>

#define INDEX_SPEED 0.2
class Climber
{
    public:
		Climber(int climbMotor, int piston, int magswitch, int climbswitch);
		Climber(CANTalon *climbMotor, Solenoid *piston, int magswitch, int climbswitch);
		Climber(CANTalon &climbMotor, Solenoid &piston, int magswitch, int climbswitch);
		~Climber();
		void GrabRope();
		void ReleaseRope();
		void ClimbRope(double speed);
		bool IsIndexed();
		bool IsAtTop();
		void MoveToIndex();
		void Stop();

    private:
		CANTalon *m_climb;
		Solenoid *m_piston;
		DigitalInput *m_index;
		DigitalInput *m_kswitch;
		bool m_needFree;
        frc::LiveWindow* lw = LiveWindow::GetInstance();
        void climberInit();
};

