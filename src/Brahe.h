/*
 * Brahe.h
 *
 *  Created on: Jan 20, 2017
 *      Author: Michael
 */

#ifndef SRC_BRAHE_H_
#define SRC_BRAHE_H_

enum DriveMotors { LEFT_DRIVEMOTOR = 1, LEFT_SLAVEMOTOR, RIGHT_DRIVEMOTOR, RIGHT_SLAVEMOTOR, NUM_DRIVE_MOTORS };
enum Joysticks   { LEFT_JOYSTICK = 0, RIGHT_JOYSTICK, NUM_JOYSTICKS };
enum Gears       { LOW_GEAR, HIGH_GEAR, NUM_GEARS };

#define SHIFT_FORWARD 5
#define SHIFT_REVERSE 6

#define ENCODER_TICKS_PER_REV 1440
#define WHEEL_DIAMETER_INCHES 6.25
#define LOW_GEAR_RATIO (17.0/25.9)
#define HIGH_GEAR_RATIO (17.0/6.96)


#endif /* SRC_BRAHE_H_ */
