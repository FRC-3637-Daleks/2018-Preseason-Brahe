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
enum Cameras	 { FRONT_CAMERA = 0, REAR_CAMERA, NUM_CAMERAS };
enum Gears       { LOW_GEAR, HIGH_GEAR, NUM_GEARS };
enum Solenoids	 { ARM_SOLENOID, PIVOT_SOLENOID, PISTON_SOLENOID, CLIMB_SOLENOID, SHIFTER_SOLENOID };
enum DigitalIO   { GEAR_SWITCH = 1, DRUM_SWITCH, CLIMB_SWITCH = 4 };
enum PWMs		 { CAMERA_SERVO = 0 };
enum Relays		 { LIGHT_SWITCH = 0 };

// Compressor
#define PCM_ID 14

// Camera servo positions
#define FRONT_VIEW_POSITION 0.0 // 0.14
#define GROUND_VIEW_POSITION 0.18 // 0.33

#define ENCODER_TICKS_PER_REV 360
#define WHEEL_DIAMETER_INCHES 6.0
#define AT_WHEEL_RATIO (26.0/22.0)

#define RPM_THRESHOLD 300
#define RAMP_RATE 50.0

#endif /* SRC_BRAHE_H_ */
