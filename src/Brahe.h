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

// Compressor
#define PCM_ID 14

// Servo
#define CAMERASERVO 0

#define ENCODER_TICKS_PER_REV 360
#define WHEEL_DIAMETER_INCHES 6.0
#define AT_WHEEL_RATIO (26.0/22.0)

#define RPM_THRESHOLD 300
#define RAMP_RATE 50.0

#endif /* SRC_BRAHE_H_ */
