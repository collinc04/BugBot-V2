# Bug bot-V2
[![Project stack](https://skillicons.dev/icons?i=cpp,c,arduino)](https://skillicons.dev)

This project is a more polished fork of an original<br>
Original Bug Bot can be found here: https://github.com/bmck039/Robot_King_Creativity. 

## methodology:
This project was started withGrant money obtained as a part of Southwestern University's King Creativity Fund https://www.southwestern.edu/undergraduate-research/king-creativity-fund/ The goal of the project has been to create a fully functional and customizable platform for beginners to learn about robotics and the math that governs modern robotic systems on cheap and reliable hardware. This is not intended as a standalone product but rather a jumping off point for users to play and expirement with. The library included with this project contains settings for the lengths 

## resources:
You will need a 3D printer to assemble this project, as well as an Arduino nano, a nano servo expansion board and 12 hobby servos. Tower pro 9g plastic and metal variants work for this project but the metal gear servos are highly recommended.
https://tinyurl.com/ykw8mreh
https://tinyurl.com/yfc457us

## library use:
download this repo as a .zip and include it in your Arduino IDE for your bugbot-v2 project
```cpp
//Initializer
QuadrupedRobot();

//Initializer
QuadrupedRobot(int legLength, int baseLength, int clawLength);

//attaches motors and stands
void initialize();

//moves the motors to stand
void stand();

//Executes an input function incrementally over the course of moveTime milliseconds. the input function f should 
//be a lambda function that takes an integer parameter that controls the execution progress
template<typename T> void executeFunctionOverTime(int moveTime, T&& f);

//moves a particular joint to a specified angle
void moveJoint(int legNum, int jointNum, int angle);

//moves all hips to the specified angle over the course of moveTime ms
void moveHips(int angle, int moveTime);

//moves all knees to the specified angle over the course of moveTime ms
void moveKnees(int angle, int moveTime);

//moves all ankles to the specified angle over the course of moveTime ms
void moveAnkles(int angle, int moveTime);

//moves hips and knees to the specified angles and moves the ankles to a calculated 
//angle to keep them pointing straight down
void moveAligned(int hipAngle, int kneeAngle);

//checks to see if a particular joint type is at a specified angle
bool indexIsAtAngle(int i, int angle);

//moves a particular leg to the specified angles over the course of moveTime ms.
void moveLeg(int i, int legAngles[], int moveTime);

//moves all joints to the specified angles over the course of moveTime ms.
void moveJoints(int moveAngles[4][3], int moveTime);

//moves the joints to a safe position
void safePosition();

//sets the internal calibration array
void setCalibration(int offsetArray[4][3]);

//runs a Serial-interactive loop to adjust the calibration array. Sets the zero-point of each motor. Hips should
//point at right-angles to the body, knees should point fully vertical, and ankles should be at right-angles 
//to the legs
void calibrate();

//positions the leg in such a way that the tip of the leg occupies the coordinates (x,y,z) 
//millimeters in space in reference to the leg joint
void positionFromCoordinates(int legNum, int x, int y, int z);

//returns all 12 leg servo angles in an array
AngleArray getCurrentPosition();

//time between motor moves
int delayTime;

//default time that a move takes
int defaultMoveTime;
```

## mathematics:
Implementation: 
```cpp
void QuadrupedRobot::inverseKinematics(int legNum, int x, int y, int z,
int &hipAngle, int &kneeAngle, int &ankleAngle) {
    float pi = 3.14159;

    if(x == QuadrupedRobot::same) {
        x = QuadrupedRobot::getXCoord(QuadrupedRobot::setAngles[legNum]);
    }

    if(y == QuadrupedRobot::same) {
        y = QuadrupedRobot::getYCoord(QuadrupedRobot::setAngles[legNum]);
    }

    if(z == QuadrupedRobot::same) {
        z = QuadrupedRobot::getZCoord(QuadrupedRobot::setAngles[legNum]);
    }

    hipAngle = atan2(x, y) * 180 / pi;
    if(hipAngle < 0) { hipAngle += 360; }

    float r = sqrt(pow(x, 2) + pow(y, 2)) - QuadrupedRobot::segmentBLength;
    int &c = QuadrupedRobot::segmentCLength;
    int &l = QuadrupedRobot::segmentLLength;

    float theta = atan2(z, r) * 180 / pi;
    int length = sqrt(pow(r, 2) + pow(z, 2));
    float gamma = acos((pow(length, 2) - pow(l, 2) - pow(c, 2))/(2*c*l)) * 180 / pi;
    float phi = asin(c * sin(gamma * pi / 180) / length) * 180 / pi; 

    kneeAngle = 90 - theta - phi;
    ankleAngle = 90 - gamma;
}
```
