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

enum Motors      { LEFT_DRIVEMOTOR = 1, LEFT_SLAVEMOTOR, RIGHT_DRIVEMOTOR, RIGHT_SLAVEMOTOR, CLIMB_MOTOR, NUM_MOTORS };
enum Joysticks   { LEFT_JOYSTICK = 0, RIGHT_JOYSTICK, XBOX_CONTROLS, NUM_JOYSTICKS };
enum Gears       { LOW_GEAR, HIGH_GEAR, NUM_GEARS };

#define CLIMB_MOTOR 5

// Compressor
#define PCM_ID 14

// Solenoids
#define ARM 0
#define PIVOT 1
#define PISTON 2
#define CLIMB 3
#define SHIFTER 4

// Digital inputs
#define GEAR_SWITCH 1
#define PEG_SWITCH  2
#define DRUM_SWITCH 3
#define CLIMB_SWITCH 4

#define ENCODER_TICKS_PER_REV 360
#define WHEEL_DIAMETER_INCHES 6.25
#define LOW_GEAR_RATIO (17.0/25.9)
#define HIGH_GEAR_RATIO (17.0/6.96)

// #define CAMERA_INUSE

#endif /* SRC_BRAHE_H_ */
