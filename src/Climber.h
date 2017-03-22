/*
 * Climber.h
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */

#pragma once
#include <Brahe.h>

#define INDEX_SPEED -0.3
#define CLIMB_SPEED -0.9

class Climber
{
    public:
		enum climbState { INDEXING, INDEXED, CLIMBING, AT_TOP, STOP, NUM_CLIMB_STATES };
		Climber(int climbMotor, int piston, int magswitch, int climbswitch);
		Climber(CANTalon *climbMotor, Solenoid *piston, int magswitch, int climbswitch);
		Climber(CANTalon &climbMotor, Solenoid &piston, int magswitch, int climbswitch);
		~Climber();
		void GrabRope();
		void ReleaseRope();
		void ClimbRope();
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
		enum climbState m_state;

        frc::LiveWindow* lw = LiveWindow::GetInstance();
        void climberInit();
        static void climbIndexer(void *c);
};

