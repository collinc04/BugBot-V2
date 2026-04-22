#include "QuadrupedRobot.h"

void setup() {
  //initialize robot
  QuadrupedRobot robot;

  //calibrate
  robot.calibrate();

  //Move servos into position to screw in horns over 5 seconds
  robot.moveHips(90, 1000);
  delay(1000);
  robot.moveKnees(90, 1000);
  delay(1000);
  robot.moveAnkles(90, 1000);
}

// the loop function runs over and over again forever
void loop() {
  
}
