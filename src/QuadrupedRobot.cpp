#include "Arduino.h"
#include "QuadrupedRobot.h"
#include <Servo.h>
#include <math.h>

// QuadrupedRobot::width = 70;
// QuadrupedRobot::

//Initializer
QuadrupedRobot::QuadrupedRobot() {

    QuadrupedRobot::delayTime = 10;
    QuadrupedRobot::defaultMoveTime = 250;
    //Standardized in millimeters
    
    QuadrupedRobot::segmentLLength = 65; //Leg
    QuadrupedRobot::segmentBLength = 33; //Base
    QuadrupedRobot::segmentCLength = 63; //Claw

    QuadrupedRobot::zDown = -QuadrupedRobot::segmentCLength;
    QuadrupedRobot::zUp = zDown / 2;
    QuadrupedRobot::xDefault = (QuadrupedRobot::segmentBLength + QuadrupedRobot::segmentLLength) / sqrt(2);
    QuadrupedRobot::yDefault = (QuadrupedRobot::segmentBLength + QuadrupedRobot::segmentLLength) / sqrt(2);;
    QuadrupedRobot::stepSize = 30;

    QuadrupedRobot::same = -255;
}

QuadrupedRobot::QuadrupedRobot(int legLength, int baseLength, int clawLength, int stepSize) {

    QuadrupedRobot::QuadrupedRobot();
    //Standardized in millimeters
    QuadrupedRobot::segmentLLength = legLength; //Leg
    QuadrupedRobot::segmentBLength = baseLength; //Base
    QuadrupedRobot::segmentCLength = clawLength; //Claw

    QuadrupedRobot::zDown = -QuadrupedRobot::segmentCLength;
    QuadrupedRobot::zUp = zDown / 2;
    QuadrupedRobot::xDefault = (QuadrupedRobot::segmentBLength + QuadrupedRobot::segmentLLength) / sqrt(2);
    QuadrupedRobot::yDefault = (QuadrupedRobot::segmentBLength + QuadrupedRobot::segmentLLength) / sqrt(2);;
    QuadrupedRobot::stepSize = stepSize;
}

int findInArray(int arr[], int element) {
    for(int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
        if(arr[i] == element) return i;
    }
    return -1;
}

//attaches motors and stands
void QuadrupedRobot::initialize() {
    int overrideArray[] = {0};
    QuadrupedRobot::moveHips(90, 1);
    QuadrupedRobot::moveKnees(0, 1);
    QuadrupedRobot::moveAnkles(90, 1);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            int index = findInArray(overrideArray, 4*j + i);
            if(index >= 0) {
                QuadrupedRobot::motors[i][j].attach(12 + index);
            } else {
                QuadrupedRobot::motors[i][j].attach(4*j + i);
            }
        }
    }
    // delay(1000);
    QuadrupedRobot::stand();
}

//moves the motors to stand
void QuadrupedRobot::stand() {

    QuadrupedRobot::safePosition();
    delay(defaultMoveTime);
    QuadrupedRobot::moveKnees(90, 1000);
    delay(defaultMoveTime);
}

//moves a particular leg to the specified angles over the course of moveTime ms.
void QuadrupedRobot::moveLeg(int i, int legAngles[], int moveTime) {
    int moveAngles[4][3];
    memcpy(&moveAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    for(int j = 0; j < 3; j++) {
        moveAngles[i][j] = legAngles[j];
    }

    QuadrupedRobot::moveJoints(moveAngles, moveTime);
}

//checks to see if a particular joint type is at a specified angle
bool QuadrupedRobot::indexIsAtAngle(int i, int angle) {
    bool isAtAngle = true;
    for (int j = 0; j < 4 && isAtAngle; j++) {
        int setAngle = QuadrupedRobot::motors[j][i].read();
        int correctedAngle = QuadrupedRobot::correctAngle(j, i, angle);
        isAtAngle = isAtAngle && (setAngle == correctedAngle);
    }
    return isAtAngle;
    
}


//internal function to transform an angle into local coordinates for a particular motor
int QuadrupedRobot::correctAngle(int legNum, int jointNum, int angle) {
    int correctedAngle;
    angle += QuadrupedRobot::calibrationArray[legNum][jointNum];
    if(angle > 135 && jointNum == 0) {
        angle = 135; // safeguard to prevent legs crashing into the body
    }
    if(legNum == 0 or legNum == 2) {
        correctedAngle = 180 - angle;
    } else {
        correctedAngle = angle;
    }
    switch (jointNum) {
        case 1:
        case 2:
            correctedAngle = 180 - correctedAngle; //for knees: 90 is horizontal
            break;
        
        default:
            break;
    }
    correctedAngle = constrain(correctedAngle, 0, 180);
    return correctedAngle;
}

//sets the internal calibration array
void QuadrupedRobot::setCalibration(int offsetArray[4][3]) {
    memcpy(&QuadrupedRobot::calibrationArray[0][0], &offsetArray[0][0], sizeof(offsetArray[0][0]) * 4 * 3);
}

//runs a Serial-interactive loop to adjust the calibration array. Sets the zero-point of each motor. Hips should point at right-angles to the body, knees should point fully vertical, and ankles should be at right-angles to the legs
void QuadrupedRobot::calibrate() {
    Serial.begin(9600);
    String arrayResult = String("{");
    for(int i = 0; i < 4; i++) {
        arrayResult.concat("{");
        for(int j = 0; j < 3; j++) {
            int currentOffset = 0;
            bool notEmpty = true;
            Serial.print("Calibrating Leg Number ");
            Serial.print(i);
            Serial.print(" and Motor Number ");
            Serial.println(j);
            QuadrupedRobot::moveJoint(i, j, 0);
            while(notEmpty) {
                int result = Serial.parseInt();
                notEmpty = !(result == -1);
                if(notEmpty) {
                    currentOffset += result;
                }
                QuadrupedRobot::calibrationArray[i][j] = currentOffset;
                QuadrupedRobot::moveJoint(i, j, 0);
            }
            arrayResult.concat(currentOffset);
            if(j != 2) {
                arrayResult.concat(", ");
            }
        }
        arrayResult.concat("}");
        if(i != 3) {
            arrayResult.concat(", ");
        }
    }
    arrayResult.concat("}");
    Serial.print("Calibration Array: ");
    Serial.println(arrayResult);
    Serial.end();
}

//moves a particular joint to a specified angle
void QuadrupedRobot::moveJoint(int legNum, int jointNum, int angle) {
    setAngles[legNum][jointNum] = angle;
    int correctedAngle = QuadrupedRobot::correctAngle(legNum, jointNum, angle);
    QuadrupedRobot::motors[legNum][jointNum].write(correctedAngle);
    // delay(delayTime);
}

//Executes an input function incrementally over the course of moveTime milliseconds. the input function f should be a lambda function that takes an integer parameter that controls the execution progress
template<typename T> void QuadrupedRobot::executeFunctionOverTime(int moveTime, T&& f) {
    int moveStartTime = millis();
    int currentTime = 0;
    while(currentTime <= moveTime) {
        currentTime = millis() - moveStartTime;
        f(currentTime);
    }
    f(moveTime);
}

//internal function to move a particular joint index to an angle over moveTime ms.
void QuadrupedRobot::moveIndexOverTime(int i, int angle, int moveTime) {
    int moveAngles[4][3];
    memcpy(&moveAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    for(int j = 0; j < 4; j++) {
        moveAngles[j][i] = angle;
    }
    if(! QuadrupedRobot::indexIsAtAngle(i, angle)) {
        QuadrupedRobot::moveJoints(moveAngles, moveTime);
    }
    
    // delay(moveTime);
}

//moves all joints to the specified angles over the course of moveTime ms.
void QuadrupedRobot::moveJoints(int moveAngles[4][3], int moveTime) {
    int startAngles[4][3];
    memcpy(&startAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    auto function = [=](int t) {
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 3; j++) {
                int angle = moveAngles[i][j];
                int moveAngle = map(t, 0, moveTime, startAngles[i][j], angle);
                if(startAngles[i][j] < angle) {
                        moveAngle = constrain(moveAngle, startAngles[i][j], angle);
                } else {
                        moveAngle = constrain(moveAngle, angle, startAngles[i][j]);
                }
                QuadrupedRobot::moveJoint(i, j, moveAngle);
            }
        }
    };
    executeFunctionOverTime(moveTime, function);
}

//internal function to move a particular joint index to an angle immediately
void QuadrupedRobot::moveIndex(int i, int angle) {
    for(int j = 0; j < 4; j++) {
        QuadrupedRobot::moveJoint(j, i, angle);
    }
    // delay(QuadrupedRobot::delayTime);
}

//moves all hips to the specified angle over the course of moveTime ms
void QuadrupedRobot::moveHips(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(0, angle, moveTime);
}

//moves all knees to the specified angle over the course of moveTime ms
void QuadrupedRobot::moveKnees(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(1, angle, moveTime);
}

//moves all ankles to the specified angle over the course of moveTime ms
void QuadrupedRobot::moveAnkles(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(2, angle, moveTime);
}

//moves hips and knees to the specified angles and moves the ankles to a calculated angle to keep them pointing straight down
void QuadrupedRobot::moveAligned(int hipAngle, int kneeAngle) {
    int moveAngles[4][3];
    for(int i = 0; i < 4; i++) {
        moveAngles[i][0] = hipAngle;
        moveAngles[i][1] = kneeAngle;
        moveAngles[i][2] = kneeAngle - 90;
    }
    QuadrupedRobot::moveJoints(moveAngles, QuadrupedRobot::defaultMoveTime);
}

void QuadrupedRobot::moveLegAligned(int legNum, int hipAngle, int kneeAngle) {
    int moveAngles[4][3];
    memcpy(&moveAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    moveAngles[legNum][0] = hipAngle;
    moveAngles[legNum][1] = kneeAngle;
    moveAngles[legNum][2] = kneeAngle - 90;
    QuadrupedRobot::moveJoints(moveAngles, QuadrupedRobot::defaultMoveTime);
}

//moves the joints to a safe position
void QuadrupedRobot::safePosition() {
    int setAngles[4][3] = {{45, 0, 0}, {45, 0, 0}, {45, 0, 0}, {45, 0, 0}};
    QuadrupedRobot::moveJoints(setAngles, 1000);
}

int QuadrupedRobot::getXCoord(int (&angles)[3]) {
    float pi = 3.14159;

    int &b = QuadrupedRobot::segmentBLength;
    int &c = QuadrupedRobot::segmentCLength;
    int &l = QuadrupedRobot::segmentLLength;

    return (l*sin(angles[1] * pi / 180) + c*cos((angles[1] - angles[2]) * pi / 180) + b) * sin(angles[0] * pi / 180);
}

int QuadrupedRobot::getYCoord(int (&angles)[3]) {
    float pi = 3.14159;

    int &b = QuadrupedRobot::segmentBLength;
    int &c = QuadrupedRobot::segmentCLength;
    int &l = QuadrupedRobot::segmentLLength;

    return (l*sin(angles[1] * pi / 180) + c*cos((angles[1] - angles[2]) * pi / 180) + b) * cos(angles[0] * pi / 180);
}

int QuadrupedRobot::getZCoord(int (&angles)[3]) {
    float pi = 3.14159;

    int &b = QuadrupedRobot::segmentBLength;
    int &c = QuadrupedRobot::segmentCLength;
    int &l = QuadrupedRobot::segmentLLength;

    return (l*cos(angles[1] * pi / 180) - c*sin((angles[1] - angles[2]) * pi / 180) - b);
}

void QuadrupedRobot::inverseKinematics(int legNum, int x, int y, int z, int &hipAngle, int &kneeAngle, int &ankleAngle) {
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

void QuadrupedRobot::positionFromCoordinates(int legNum, int x, int y, int z) {
    int hipAngle;
    int kneeAngle;
    int ankleAngle;
    inverseKinematics(legNum, x, y, z, hipAngle, kneeAngle, ankleAngle);

    int angles[] = {hipAngle, kneeAngle, ankleAngle};
    QuadrupedRobot::moveLeg(legNum, angles, QuadrupedRobot::defaultMoveTime);
}

void QuadrupedRobot::moveToPositions(int (&positions)[4][3]) {
    int angles[4][3];
    for(int i = 0; i < 4; i++) {
        int hipAngle;
        int kneeAngle;
        int ankleAngle;
        QuadrupedRobot::inverseKinematics(i, positions[i][0], positions[i][1], positions[i][2], hipAngle, kneeAngle, ankleAngle);
        angles[i][0] = hipAngle;
        angles[i][1] = kneeAngle;
        angles[i][2] = ankleAngle;
    }

    QuadrupedRobot::moveJoints(angles, QuadrupedRobot::defaultMoveTime);
}

QuadrupedRobot::AngleArray QuadrupedRobot::getCurrentPosition() {
    struct QuadrupedRobot::AngleArray angles;
    // angles.array = QuadrupedRobot::setAngles;
    memcpy(&angles.array[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    return angles;
}

void rotateCoordinates(int angle, int startX, int startY, int &endX, int &endY) {
    float pi = 3.14159;
    endX = startX * cos(angle * pi / 180) - startY * sin(angle * pi / 180);
    endY = startX * sin(angle * pi / 180) + startY * cos(angle * pi / 180);
}

void QuadrupedRobot::forward (int repetitions) {
    for (int i = 0; i < repetitions; i++) {

        QuadrupedRobot::positionFromCoordinates(2, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(2, QuadrupedRobot::xDefault + 20, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(2, QuadrupedRobot::same, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);
        
        QuadrupedRobot::positionFromCoordinates(3, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(3, QuadrupedRobot::xDefault + 20, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(3, QuadrupedRobot::same, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);

        QuadrupedRobot::positionFromCoordinates(0, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(0, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(0, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);
        
        QuadrupedRobot::positionFromCoordinates(1, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(1, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(1, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);
        


        int coordinates[4][3] = {{QuadrupedRobot::same, QuadrupedRobot::yDefault - QuadrupedRobot::stepSize, QuadrupedRobot::zDown}, {QuadrupedRobot::same, QuadrupedRobot::yDefault - QuadrupedRobot::stepSize, QuadrupedRobot::zDown}, {QuadrupedRobot::same, QuadrupedRobot::yDefault + QuadrupedRobot::stepSize, QuadrupedRobot::zDown}, {QuadrupedRobot::same, QuadrupedRobot::yDefault + QuadrupedRobot::stepSize, QuadrupedRobot::zDown}};
        QuadrupedRobot::moveToPositions(coordinates);

    }
    for(int i = 0; i < 4; i++) {
        QuadrupedRobot::positionFromCoordinates(i, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(i, QuadrupedRobot::xDefault, QuadrupedRobot::yDefault, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(i, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zDown);
    }
}

void QuadrupedRobot::backward (int repetitions) {
    for (int i = 0; i < repetitions; i++) {

        QuadrupedRobot::positionFromCoordinates(0, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(0, QuadrupedRobot::xDefault + 20, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(0, QuadrupedRobot::same, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);
        
        QuadrupedRobot::positionFromCoordinates(1, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(1, QuadrupedRobot::xDefault + 20, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(1, QuadrupedRobot::same, QuadrupedRobot::yDefault - 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);

        QuadrupedRobot::positionFromCoordinates(2, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(2, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(2, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);
        
        QuadrupedRobot::positionFromCoordinates(3, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(3, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(3, QuadrupedRobot::same, QuadrupedRobot::yDefault + 2*QuadrupedRobot::stepSize, QuadrupedRobot::zDown);
        


        int coordinates[4][3] = {{QuadrupedRobot::same, QuadrupedRobot::yDefault + QuadrupedRobot::stepSize, QuadrupedRobot::zDown}, {QuadrupedRobot::same, QuadrupedRobot::yDefault + QuadrupedRobot::stepSize, QuadrupedRobot::zDown}, {QuadrupedRobot::same, QuadrupedRobot::yDefault - QuadrupedRobot::stepSize, QuadrupedRobot::zDown}, {QuadrupedRobot::same, QuadrupedRobot::yDefault - QuadrupedRobot::stepSize, QuadrupedRobot::zDown}};
        QuadrupedRobot::moveToPositions(coordinates);

    }
    for(int i = 0; i < 4; i++) {
        QuadrupedRobot::positionFromCoordinates(i, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zUp);
        QuadrupedRobot::positionFromCoordinates(i, QuadrupedRobot::xDefault, QuadrupedRobot::yDefault, QuadrupedRobot::same);
        QuadrupedRobot::positionFromCoordinates(i, QuadrupedRobot::same, QuadrupedRobot::same, QuadrupedRobot::zDown);
    }
}

void QuadrupedRobot::rotateByAngle(int angle) {
    int repetitions = abs(angle / 30);
    for (int i = 0; i < repetitions; i++) {
        int stepAngle = angle > 30 ? 30 : angle;
        int rotatedX[] = {0, 0, 0, 0};
        int rotatedY[] = {0, 0, 0, 0};
        for(int i = 0; i < 4; i++) {
            int sign;
            switch (i) {
            case 0:
            case 2:
                sign = 1;
                break;
            case 1:
            case 3:
                sign = -1;
                break;
            }
            rotateCoordinates(stepAngle * sign, QuadrupedRobot::xDefault, QuadrupedRobot::yDefault, rotatedX[i], rotatedY[i]);
            angle -= stepAngle;
        }
        for(int j = 0; j < 4; j++) {
            int index = (j < 2) ? 2 * j : (j - 2) * 2 + 1;
            QuadrupedRobot::positionFromCoordinates(index, QuadrupedRobot::xDefault, QuadrupedRobot::yDefault, QuadrupedRobot::zUp);
            QuadrupedRobot::positionFromCoordinates(index, rotatedX[index], rotatedY[index], QuadrupedRobot::zUp);
            QuadrupedRobot::positionFromCoordinates(index, rotatedX[index], rotatedY[index], QuadrupedRobot::zDown);
        }
        QuadrupedRobot::moveHips(45, QuadrupedRobot::defaultMoveTime);
    }
}

void QuadrupedRobot::right (int repetitions) {
    for(int i = 0; i < repetitions; i++) {
        QuadrupedRobot::rotateByAngle(-30);
    }
}

void QuadrupedRobot::left (int repetitions) {
    for(int i = 0; i < repetitions; i++) {
        QuadrupedRobot::rotateByAngle(30);
    }
}
