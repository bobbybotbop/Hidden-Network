#include "vex.h"

// CONSTANTS & VARIABLES
// Encoder values
double currentL;
double currentR;
double currentB;
// Change in encoder values
double deltaLeft;
double deltaRight;
double deltaBack;
// Encoder value before last reset
double prevLeft;
double prevRight;
double prevBack;
// Robot orientation/heading/angle
double angleDeg;
double angleRad;
double deltaAngle;
double prevAngleRad;
// Change in x and y coordinates
double xCoordinateChange;
double yCoordinateChange;
// Polar coordinates
double polarCoordinateAngle;
double polarRadius;
// Absolute x and y coordinates
double finalGlobalX;
double finalGlobalY;
double prevGlobalX;
double prevGlobalY;
// PD loop constants & variables
double turnError;
double driveError;
double prevTurnError;
double prevDriveError;
double turnDerivative;
double driveDerivative;

double IMUAngleRad;
double IMUAngle;
double deltaIMUAngle;
double prevImuAngleRad;

double LEFT_TRACK = 5.5;  // Left tracking wheel distance from tracking center
double RIGHT_TRACK = 5.5; // right tracking wheel distance from tracking center
double BACK_TRACK = 1;  // back tracking wheel distance from tracking center

double inchPerTick = 0.02415;
double inchPerTickBack = 0.0252;
void getEncValue() {  //Tuning encoder ticks to inches. You will have to tune these
  currentL = (encL.rotation(rotationUnits::deg) * inchPerTick) * -1;
  currentR = encR.rotation(rotationUnits::deg) * inchPerTick * -1;
  currentB = encB.rotation(rotationUnits::deg) * inchPerTickBack* -1;
  // Get change in encoder values
  deltaLeft = currentL - prevLeft;
  deltaRight = currentR - prevRight;
  deltaBack = currentB - prevBack;
  // Store previous encoder value
  prevLeft = currentL;
  prevRight = currentR;
  prevBack = currentB;
}

void positionUpdate() {
  angleRad = (currentL - currentR) / (LEFT_TRACK + RIGHT_TRACK);//Angle in radians
  angleDeg = keepInRange360(angleRad * (180 / pi)); // Angle value in degrees
  deltaAngle = (deltaLeft - deltaRight) / (LEFT_TRACK + RIGHT_TRACK);//Angle change from last loop

  IMUAngle = keepInRange360(Inertial.rotation());//Angle using IMU instead of tracking wheels
  IMUAngleRad = IMUAngle * (pi / 180);
  deltaIMUAngle = IMUAngleRad - prevImuAngleRad;

  if (deltaAngle == 0) { // Avoid dividing by 0 error
    xCoordinateChange = deltaBack;
    yCoordinateChange = (deltaLeft+deltaRight)/2;
  } else {//Calculates the X, Y position change
    xCoordinateChange = (2 * sin(deltaAngle / 2)) * ((deltaBack / deltaAngle) + BACK_TRACK);
    yCoordinateChange = (2 * sin(deltaAngle / 2)) * ((deltaLeft / deltaAngle) - LEFT_TRACK);
  }

  double averagePosition = prevImuAngleRad + deltaAngle / 2; // Previous orientation plus average change in orientation
  // Converting to polar coordinates to change angle the robot is facing
  if (xCoordinateChange == 0 && yCoordinateChange == 0) { // Prevent 0 from going to atan2
    polarCoordinateAngle = 0;
    polarRadius = 0;
  } else {
    polarCoordinateAngle = atan2(yCoordinateChange, xCoordinateChange) - averagePosition;
    polarRadius = sqrt(pow(xCoordinateChange, 2) + pow(yCoordinateChange, 2));
  }

  // Converting from polar coordinates back to cartesian
  double trueChangeX = polarRadius * cos(polarCoordinateAngle);
  double trueChangeY = polarRadius * sin(polarCoordinateAngle);

  finalGlobalX = prevGlobalX + trueChangeX;//Adding the change in position to the total position
  finalGlobalY = prevGlobalY + trueChangeY;

  prevGlobalX = finalGlobalX;
  prevGlobalY = finalGlobalY;

  prevAngleRad = angleRad;
  prevImuAngleRad = IMUAngleRad;
}

int statePosition() {
  Brain.Screen.clearScreen();
  while (1) {
    getEncValue();
    positionUpdate();

    Brain.Screen.printAt(20, 20, "Global X: %f", finalGlobalX);
    Brain.Screen.printAt(20, 40, "Global Y: %f", finalGlobalY);
    Brain.Screen.printAt(20, 60, "Theta: %f", Inertial.rotation());
    Brain.Screen.printAt(20, 80, "Left Wheel: %f", currentL);
    Brain.Screen.printAt(20, 100, "Right Wheel: %f", currentR);
    Brain.Screen.printAt(20, 120, "Back Wheel: %f", currentB);
  }
  return 0;
}

int runOdomData() {
  while (1) {
    getEncValue();
    positionUpdate();
  }
  return 0;
}


void setPositionOdom(double X, double Y,double theta) { // Set position at match start
  encR.setRotation(0, rotationUnits::deg);
  encL.setRotation((LEFT_TRACK + RIGHT_TRACK) * (theta) / (inchPerTick) * (pi / 180), rotationUnits::deg);
  encB.setRotation(0, rotationUnits::deg);

  Inertial.setRotation(theta, rotationUnits::deg);

  prevLeft = 0;
  prevRight = 0;
  prevBack = 0;

  getEncValue();
  positionUpdate();

  prevGlobalX = 0;
  prevGlobalY = 0;

  prevGlobalX = X;
  prevGlobalY = Y;
}

double increase_max(double num1, double num2, double max_speed){
  double max_value;

  if (num1 > num2){
    max_value = num1;
  }
  else{
    max_value = num2;
  }

  double multiplier = 1/max_value;

  return multiplier;
}

void moveToDrive(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin) {
  
  // DTtimer = 0;
  Brain.resetTimer();

  while (Brain.timer(timeUnits::msec) < timeout) {
    // PD Control
    double driveError = PYTHAG_THEOREM(coorX -  finalGlobalX, coorY - finalGlobalY);

    turnDerivative = turnError - prevTurnError;
    driveDerivative = driveError - prevDriveError;
    prevTurnError = turnError;
    prevDriveError = driveError;
    // Basic derivative concepts


    double driveSpeed = driveSpeedCalc(driveError, driveMax, 0, driveDerivative, drivekp, 0, drivekd);
    if (driveSpeed > driveMax) {driveSpeed = driveMax;}
    if (driveSpeed < -driveMax) {driveSpeed = -driveMax;}

    float DriveMulti1 = cos(IMUAngleRad + atan2(coorY - finalGlobalY, coorX - finalGlobalX) - pi/4);

    float DriveMulti2 = cos(-IMUAngleRad - atan2(coorY - finalGlobalY, coorX - finalGlobalX) + 3*pi/4);

    left_drive.spin(fwd, driveSpeed * DriveMulti1 /*+ turnSpeed*/, velocityUnits::pct);
    right_drive.spin(fwd, driveSpeed * DriveMulti2 /*- turnSpeed*/, velocityUnits::pct);

    if (fabs(driveError) < driveErrorMargin) {
      break;
    }
    wait(5);
  }
  Drivetrain.stop(brake);
}

void moveToDriveMax(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin) {
  
  // DTtimer = 0;
  Brain.resetTimer();

  while (Brain.timer(timeUnits::msec) < timeout) {
    // PD Control
    double driveError = PYTHAG_THEOREM(coorX - finalGlobalX, coorY - finalGlobalY);

    turnDerivative = turnError - prevTurnError;
    driveDerivative = driveError - prevDriveError;
    prevTurnError = turnError;
    prevDriveError = driveError;
    // Basic derivative concepts


    double driveSpeed = driveSpeedCalc(driveError, driveMax, 0, driveDerivative, drivekp, 0, drivekd);
    if (driveSpeed > driveMax) {driveSpeed = driveMax;}
    if (driveSpeed < -driveMax) {driveSpeed = -driveMax;}

    float DriveMulti1 = cos(IMUAngleRad + atan2(coorY - finalGlobalY, coorX - finalGlobalX) - pi/4);

    float DriveMulti2 = cos(-IMUAngleRad - atan2(coorY - finalGlobalY, coorX - finalGlobalX) + 3*pi/4);

    double multiplier = increase_max(DriveMulti1, DriveMulti2, driveMax);
    DriveMulti1 *= multiplier;
    DriveMulti2 *= multiplier;

    left_drive.spin(fwd, driveSpeed * DriveMulti1 /*+ turnSpeed*/, velocityUnits::pct);
    right_drive.spin(fwd, driveSpeed * DriveMulti2 /*- turnSpeed*/, velocityUnits::pct);

    if (fabs(driveError) < driveErrorMargin) {
      break;
    }
    wait(5);
  }
  Drivetrain.stop(brake);
}

void moveToDriveBackMax(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin) {
  
  DTtimer = 0;
  Brain.resetTimer();
  while (Brain.timer(timeUnits::msec) < timeout) {
    double flipAngleRad = IMUAngleRad + pi;
    // PD Control
    double driveError = PYTHAG_THEOREM(coorX - finalGlobalX, coorY - finalGlobalY);

    turnDerivative = turnError - prevTurnError;
    driveDerivative = driveError - prevDriveError;
    prevTurnError = turnError;
    prevDriveError = driveError;
    // Basic derivative concepts


    double driveSpeed = driveSpeedCalc(driveError, driveMax, 0, driveDerivative, drivekp, 0, drivekd);
    if (driveSpeed > driveMax) {driveSpeed = driveMax;}
    if (driveSpeed < -driveMax) {driveSpeed = -driveMax;}

    float DriveMulti1 = cos(flipAngleRad + atan2(coorY - finalGlobalY, coorX - finalGlobalX) - pi/4);

    float DriveMulti2 = cos(-flipAngleRad - atan2(coorY - finalGlobalY, coorX - finalGlobalX) + 3*pi/4);
    
    double multiplier = increase_max(DriveMulti1, DriveMulti2, driveMax);
    DriveMulti1 *= multiplier;
    DriveMulti2 *= multiplier;

    left_drive.spin(reverse, driveSpeed * DriveMulti2 /*+ turnSpeed*/, velocityUnits::pct);
    right_drive.spin(reverse, driveSpeed * DriveMulti1 /*- turnSpeed*/, velocityUnits::pct);

    if (fabs(driveError) < driveErrorMargin) {
      break;
    }
    wait(5);
  }
  Drivetrain.stop(brake);
}


void moveToDriveBack(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin) {
  
  DTtimer = 0;
  Brain.resetTimer();
  while (Brain.timer(timeUnits::msec) < timeout) {
    double flipAngleRad = IMUAngleRad + pi;
    // PD Control
    double driveError = PYTHAG_THEOREM(coorX - finalGlobalX, coorY - finalGlobalY);

    turnDerivative = turnError - prevTurnError;
    driveDerivative = driveError - prevDriveError;
    prevTurnError = turnError;
    prevDriveError = driveError;
    // Basic derivative concepts


    double driveSpeed = driveSpeedCalc(driveError, driveMax, 0, driveDerivative, drivekp, 0, drivekd);
    if (driveSpeed > driveMax) {driveSpeed = driveMax;}
    if (driveSpeed < -driveMax) {driveSpeed = -driveMax;}

    float DriveMulti1 = cos(flipAngleRad + atan2(coorY - finalGlobalY, coorX - finalGlobalX) - pi/4);

    float DriveMulti2 = cos(-flipAngleRad - atan2(coorY - finalGlobalY, coorX - finalGlobalX) + 3*pi/4);

    left_drive.spin(reverse, driveSpeed * DriveMulti2 /*+ turnSpeed*/, velocityUnits::pct);
    right_drive.spin(reverse, driveSpeed * DriveMulti1 /*- turnSpeed*/, velocityUnits::pct);

    if (fabs(driveError) < driveErrorMargin) {
      break;
    }
    wait(5);
  }
  Drivetrain.stop(brake);
}

void moveToDrive_limit(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin) {
  
  // DTtimer = 0;
  Brain.resetTimer();

  while (Brain.timer(timeUnits::msec) < timeout) {
    // PD Control
    double driveError = PYTHAG_THEOREM(coorX - finalGlobalX, coorY - finalGlobalY);

    turnDerivative = turnError - prevTurnError;
    driveDerivative = driveError - prevDriveError;
    prevTurnError = turnError;
    prevDriveError = driveError;
    // Basic derivative concepts


    double driveSpeed = driveSpeedCalc(driveError, driveMax, 0, driveDerivative, drivekp, 0, drivekd);
    if (driveSpeed > driveMax) {driveSpeed = driveMax;}
    if (driveSpeed < -driveMax) {driveSpeed = -driveMax;}

    float DriveMulti1 = cos(IMUAngleRad + atan2(coorY - finalGlobalY, coorX - finalGlobalX) - pi/4);

    float DriveMulti2 = cos(-IMUAngleRad - atan2(coorY - finalGlobalY, coorX - finalGlobalX) + 3*pi/4);
    
    double multiplier = increase_max(DriveMulti1, DriveMulti2, driveMax);
    DriveMulti1 *= multiplier;
    DriveMulti2 *= multiplier;

    left_drive.spin(fwd, driveSpeed * DriveMulti1 /*+ turnSpeed*/, velocityUnits::pct);
    right_drive.spin(fwd, driveSpeed * DriveMulti2 /*- turnSpeed*/, velocityUnits::pct);

    if (fabs(driveError) < driveErrorMargin || frontLimit.pressing() == 1) {
      break;
    }
    wait(5);
  }
  // front_claw.set(true);
  Drivetrain.stop(brake);
}