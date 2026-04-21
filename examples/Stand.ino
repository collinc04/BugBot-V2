#include "QuadrupedRobot.h"

void setup() {
  //initialize robot
  QuadrupedRobot robot;

  //Calibrate and stand up
  robot.calibrate();
  robot.initialize();
}

// the loop function runs over and over again forever
void loop() {
  
}
