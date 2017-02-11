/*
 * Brahe.h
 *
 *  Created on: Jan 20, 2017
 *      Author: Michael
 */
#ifndef SRC_BRAHE_H_
#define SRC_BRAHE_H_

#include <WPILib.h>
#include <CANTalon.h>

enum DriveMotors { LEFT_DRIVEMOTOR = 1, LEFT_SLAVEMOTOR, RIGHT_DRIVEMOTOR, RIGHT_SLAVEMOTOR, NUM_DRIVE_MOTORS };
enum Joysticks   { LEFT_JOYSTICK = 0, RIGHT_JOYSTICK, NUM_JOYSTICKS };
enum Gears       { LOW_GEAR, HIGH_GEAR, NUM_GEARS };

// Solenoids
#define SHIFT_A 1
#define SHIFT_B 2
#define PIVOT_A 3
#define PIVOT_B 4
#define PISTON_A 5
#define PISTON_B 6
#define ARM 7

// Digital inputs
#define GEAR_SWITCH 1
#define PEG_SWITCH  2

#define ENCODER_TICKS_PER_REV 360
#define WHEEL_DIAMETER_INCHES 6.25
#define LOW_GEAR_RATIO (17.0/25.9)
#define HIGH_GEAR_RATIO (17.0/6.96)

#endif /* SRC_BRAHE_H_ */
