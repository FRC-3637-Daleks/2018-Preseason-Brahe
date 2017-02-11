/*
 * Climber.h
 *
 *  Created on: Feb 8, 2017
 *      Author: Poofi
 */

#ifndef SRC_CLIMBER_H_
#define SRC_CLIMBER_H_
#include <CANTalon.h>;
#include <Solenoid.h>
class Climber {
 public:
  Climber(int c, int p, int f, int r, XboxController xb);
  void Switch();
  void Up(double speed);
  void Stop();
  void Play();

 private:
  CANTalon *Climb;
  Solenoid *Piston;
  XboxController *xbox;
};



#endif /* SRC_CLIMBER_H_ */
