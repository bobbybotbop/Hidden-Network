#include "vex.h"

int driveSpeedCalc(double distError, double maxSpeed, double totalError, double deltaError, double kp, double ki, double kd){
  double proportion = distError * kp;
  
  double integral = totalError * ki;

  double derivative = deltaError * kd;

  double motorSpeed = proportion + integral + derivative;

  if(motorSpeed > maxSpeed){
    motorSpeed = maxSpeed;
  }
  if(motorSpeed < -maxSpeed){
    motorSpeed = -maxSpeed;
  }

  return motorSpeed;
}

int turnSpeedCalc(double angleError, double maxSpeed, double turnTotalError, double turnDeltaError, 
                  double turnKp, double turnKi, double turnKd)
{
  double turnProportion = angleError * turnKp;
  double turnIntegral = turnTotalError * turnKi;
  double turnDerivative = turnDeltaError * turnKd;
  double motorSpeed = turnProportion + turnIntegral + turnDerivative;

  if(motorSpeed > maxSpeed){
    motorSpeed = maxSpeed;
  }
  if(motorSpeed < -maxSpeed){
    motorSpeed = -maxSpeed;
  }

  return motorSpeed;
}

void PIDTurn(float rotateToAngle, float maxSpeed, double timeLimit, double turnErrorMargin, double kp, double ki, double kd){
  //double error = keepInNeg180to180(rotateToAngle - Inertial.rotation());
  double totalError = 0;
  double integral = 0;
  double derivative = 0;
  double prevError = 0;
  double motorSpeed = 0;

  Brain.resetTimer();

  while (Brain.timer(timeUnits::msec) < timeLimit) {
    double error = keepInNeg180to180(rotateToAngle - Inertial.rotation());

    if(fabs(error) < 50 && error != 0 && DTtimer > 0){
      totalError += error;
    }
    else {
      totalError = 0;
    }
    integral = totalError;

    derivative = error - prevError;
    prevError = error;

    motorSpeed = (kp*error) + (ki*integral) + (kd*derivative);
    if(motorSpeed>maxSpeed){
      motorSpeed=maxSpeed;
    }
    if(motorSpeed<-maxSpeed){
      motorSpeed=-maxSpeed;
    }

    left_drive.spin(fwd, motorSpeed, velocityUnits::pct);
    right_drive.spin(fwd, -motorSpeed, velocityUnits::pct);
    if(fabs(error)<turnErrorMargin){
      break;
    }
    wait(5);
  }
  Drivetrain.stop(brake);
}

void PIDMove(double dist, double maxSpeed, double timeLimit, double driveErrorMargin, double kp, double ki, double kd){
  Drivetrain.resetRotation();
  trackingWheelReset();
  int dir = 1;
  if(dist<0){
    dir = -1;
  }
  double totalError = 0;
  double prevError = 0;

  double inchesToTick = 38; //Need to tune
  double moveCount = fabs(dist*inchesToTick);

  float leftCount = encL.rotation(rotationUnits::deg);
  float rightCount = encL.rotation(deg) * -1;
  double initialCount = (fabs(leftCount + rightCount)/2)/inchesToTick;
  double goalCount = moveCount + initialCount;

  double error = (goalCount - fabs(leftCount+rightCount)/2)/inchesToTick;
  DTtimer = 0;
  while(DTtimer < timeLimit){
    leftCount = encL.rotation(rotationUnits::deg) * -1;
    rightCount = encL.rotation(deg) * -1;
    //Proportion
    error = (moveCount - fabs(leftCount+rightCount)/2)/inchesToTick;
    double proportion = kp*error;

    //Integral
    if(error < 50 && error != 0){
      totalError += error;
    }
    else{
      totalError = 0;
    }
    double integral = ki*totalError;

    //Derivative
    double deltaError = error - prevError;
    prevError = error;
    double derivative = kd * deltaError;

    double motorSpeed = proportion + integral + derivative;

    if(motorSpeed>maxSpeed){
      motorSpeed = maxSpeed;
    }
    else if(motorSpeed<-maxSpeed){
      motorSpeed = -maxSpeed;
    }
    Drivetrain.spin(fwd, dir*motorSpeed, velocityUnits::pct);
    wait(5);
    if(fabs(error)<driveErrorMargin){
      break;
    }
  }
  Drivetrain.stop(brake);
}

void PIDMove2(double dist, double maxSpeed, double timeLimit, double driveErrorMargin, double kp, double ki, double kd){
  double totalError = 0;
  double prevError = 0;


  float leftCount = encL.rotation(rotationUnits::deg);
  float rightCount = encR.rotation(deg);

  double inchesToTick = 37; 
  double moveCount = dist*inchesToTick; // in terms of ticks
  double initialCount = (leftCount + rightCount)/2; // in terms of ticks


  double goalCount = initialCount + moveCount; // in terms of ticks

  double error = (goalCount - initialCount)/inchesToTick; // in terms of inches
  Brain.resetTimer();

  while (Brain.timer(timeUnits::msec) < timeLimit) {
    leftCount = encL.rotation(rotationUnits::deg);
    rightCount = encR.rotation(deg);
    double currentCount = (leftCount + rightCount)/2; // in terms of ticks

    //Proportion
    error = (goalCount - currentCount)/inchesToTick; // in terms of inches
    double proportion = kp*error;

    //Integral
    if(error < 50 && error != 0){
      totalError += error;
    }
    else{
      totalError = 0;
    }
    double integral = ki*totalError;

    //Derivative
    double deltaError = error - prevError;
    prevError = error;
    double derivative = kd * deltaError;

    double motorSpeed = proportion + integral + derivative;

    if(motorSpeed>maxSpeed){
      motorSpeed = maxSpeed;
    }
    else if(motorSpeed<-maxSpeed){
      motorSpeed = -maxSpeed;
    }
    Drivetrain.spin(fwd, motorSpeed, velocityUnits::pct);
    wait(5);


    if(fabs(error)<driveErrorMargin){
      break;
    }
  }
  Drivetrain.stop(brake);
}

void PIDMove_limit(double dist, double maxSpeed, double timeLimit, double driveErrorMargin, double kp, double ki, double kd){
  double totalError = 0;
  double prevError = 0;


  float leftCount = encL.rotation(rotationUnits::deg);
  float rightCount = encR.rotation(deg);

  double inchesToTick = 37; 
  double moveCount = dist*inchesToTick; // in terms of ticks
  double initialCount = (leftCount + rightCount)/2; // in terms of ticks


  double goalCount = initialCount + moveCount; // in terms of ticks

  double error = (goalCount - initialCount)/inchesToTick; // in terms of inches
  Brain.resetTimer();

  while (Brain.timer(timeUnits::msec) < timeLimit) {
    leftCount = encL.rotation(rotationUnits::deg);
    rightCount = encR.rotation(deg);
    double currentCount = (leftCount + rightCount)/2; // in terms of ticks

    //Proportion
    error = (goalCount - currentCount)/inchesToTick; // in terms of inches
    double proportion = kp*error;

    //Integral
    if(error < 50 && error != 0){
      totalError += error;
    }
    else{
      totalError = 0;
    }
    double integral = ki*totalError;

    //Derivative
    double deltaError = error - prevError;
    prevError = error;
    double derivative = kd * deltaError;

    double motorSpeed = proportion + integral + derivative;

    if(motorSpeed>maxSpeed){
      motorSpeed = maxSpeed;
    }
    else if(motorSpeed<-maxSpeed){
      motorSpeed = -maxSpeed;
    }
    Drivetrain.spin(fwd, motorSpeed, velocityUnits::pct);
    wait(5);


    if(fabs(error)<driveErrorMargin || frontLimit.pressing()){
      break;
    }
  }
  // front_claw.set(true);
  Drivetrain.stop(brake);
}

void vision_move(char Color, double drive_max, double drivekp, double drivekd, double turn_max, double turnkp, double turnkd, double time_limit, bool limit_stop){
  int center_y = 200; // pixel value we want the center of the mobile goal to be at
  int center_x = 200;

  double prev_drive_error = 0;
  double prev_turn_error = 0;

  DTtimer = 0;

  while (DTtimer < time_limit){
    if (Color == 'r'){ Vision.takeSnapshot(RED); } // select the color to focus on
    else if (Color == 'b') { Vision.takeSnapshot(BLUE); }
    else{ Vision.takeSnapshot(YELLOW); }

    // takes in difference between object's center and desired center and does PID calculation
    double turn_error = center_x - Vision.largestObject.centerX;
    double drive_error = Vision.largestObject.centerY - center_y;

    double delta_turn_error = drive_error - prev_turn_error;
    prev_turn_error = drive_error;

    double delta_drive_error = drive_error - prev_drive_error;
    prev_drive_error = drive_error;

    double turn_speed = turnSpeedCalc(turn_error, turn_max, 0, delta_turn_error, turnkp, 0, turnkd);
    double drive_speed = driveSpeedCalc(drive_error, drive_max, 0, delta_drive_error, drivekp, 0, drivekd);

    if (drive_speed > drive_max) {drive_speed = drive_max;}
    if (drive_speed < -drive_max) {drive_speed = -drive_max;}
    if (turn_speed > turn_max) {turn_speed = turn_max;}
    if (turn_speed < -turn_max) {turn_speed = -turn_max;}

    right_drive.spin(fwd, drive_speed - turn_speed, velocityUnits::pct);
    left_drive.spin(fwd, drive_speed + turn_speed, velocityUnits::pct);    

    // if limit switch is pressed, stop the function
    if (limit_stop && frontLimit.value() == 1){
      break;
    }

  }

}


void auton_climb(int timeout, double target_pitch, double max_pitch, double pitch_kp, double pitch_kd, double target_yaw, double max_yaw, double yaw_kp, double yaw_kd){
  double start_time = Brain.timer(msec);
  double pitch_error = 0;
  double yaw_error = 0;
  double pitch_error_prev = 0;
  double yaw_error_prev = 0;
  double delta_pitch = 0;
  double delta_yaw = 0;
  double pitch_speed = 0;
  double yaw_speed = 0;
  Drivetrain.setStopping(hold);

  while(Inertial.pitch() < 5){
    PIDMove2(15, 100, 1000, 2, 10, 0, 0);
  }

  while (Brain.timer(msec) - start_time < timeout){
    pitch_error = Inertial.pitch() - target_pitch;
    yaw_error = Inertial.rotation() - target_yaw;

    delta_pitch = pitch_error_prev - pitch_error;
    pitch_error_prev = pitch_error;
    
    delta_yaw = yaw_error_prev - yaw_error;
    yaw_error_prev = yaw_error;

    pitch_speed = pitch_error * pitch_kp + delta_pitch * pitch_kd;
    yaw_speed = yaw_error * yaw_kp + delta_yaw * yaw_kd;

    if (pitch_speed > max_pitch) {pitch_speed = max_pitch;}
    if (pitch_speed < -max_pitch) {pitch_speed = -max_pitch;}
    if (yaw_speed > max_yaw) {yaw_speed = max_yaw;}
    if (yaw_speed < -max_yaw) {yaw_speed = -max_yaw;}

    right_drive.spin(fwd, pitch_speed + yaw_speed, velocityUnits::pct);
    left_drive.spin(fwd, pitch_speed - yaw_speed, velocityUnits::pct);    

    // if(pitch_error < 2){
    //   break;
    // }

  }
}
