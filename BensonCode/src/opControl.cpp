#include "vex.h"
#include <vector>

double TURN_MULTI=0.60;
double MOVE_MULTI=1.0;
void Drive(){
	if(abs(ch3)>5 || abs(ch1)>10){
    if(abs(ch3) > 80 || abs(ch1) > 80){
      left_drive.spin(fwd, (ch3)*MOVE_MULTI + (ch1)*TURN_MULTI, velocityUnits::pct);
	    right_drive.spin(fwd, (ch3)*MOVE_MULTI - (ch1)*TURN_MULTI, velocityUnits::pct);
    }
	  left_drive.spin(fwd, (ch3)*MOVE_MULTI + (ch1)*TURN_MULTI, velocityUnits::pct);
	  right_drive.spin(fwd, (ch3)*MOVE_MULTI - (ch1)*TURN_MULTI, velocityUnits::pct);
  }
  else{
    Drivetrain.stop();
  }
}

void DriveLimit(){
  double motorSpeed1 = ch3;
  double motorSpeed2 = ch1;
  if(abs(ch3)>5 || abs(ch1)>10){
    if(abs(ch3) > 95){
      if(ch3 > 95){
        motorSpeed1 = 95;
      }
      else if(ch3 < -95){
        motorSpeed1 = -95;
      }
      else{
        motorSpeed1 = ch3;
      }
    }
    if (abs(ch1) > 95){
      if(ch1 > 95){
        motorSpeed2 = 95;
      }
      else if(ch1 < -95){
        motorSpeed2 = -95;
      }
      else{
        motorSpeed2 = ch1;
      }
    }
      left_drive.spin(fwd, (motorSpeed1)*MOVE_MULTI + (motorSpeed2)*TURN_MULTI, velocityUnits::pct);
      right_drive.spin(fwd, (motorSpeed1)*MOVE_MULTI - (motorSpeed2)*TURN_MULTI, velocityUnits::pct);
  }
  else {
    Drivetrain.stop();
  }
}

// void Lift(){
//   if(bR1){
//     fourbar.spin(fwd, 100, velocityUnits::pct);
//     fourbar.setStopping(hold);
//     if(bR2){
//       fourBarAid.set(true);
//     }
//   }
//   else if(bR2){
//     fourbar.spin(reverse, 100, velocityUnits::pct);
//     fourbar.setStopping(hold);
//     fourBarAid.set(false);
//   }
//   else{
//     fourbar.stop();
//   }
// }

void spinFlyWheel1(){
    // if (flywheel.velocity(pct) < 45){
      flywheel_one.spin(fwd, 5, voltageUnits::volt);
      flywheel_two.spin(fwd, 5, voltageUnits::volt);    
    // }
    // else{
    //   flywheel_one.spin(fwd, 50, velocityUnits::pct);
    //   flywheel_two.spin(fwd, 50, velocityUnits::pct);
    // }
  }

void spinFlyWheel2(){
    // if (flywheel.velocity(pct) < 70){
      flywheel_one.spin(fwd, 7, voltageUnits::volt);
      flywheel_two.spin(fwd, 7, voltageUnits::volt);    
    // }
    // else{
    //   flywheel_one.spin(fwd, 75, velocityUnits::pct);
    //   flywheel_two.spin(fwd, 75, velocityUnits::pct);
    // }
  }


void spinFlyWheel3(){
    // if (flywheel.velocity(pct) < 95){
      flywheel_one.spin(fwd, 9.5, voltageUnits::volt);
      flywheel_two.spin(fwd, 9.5, voltageUnits::volt);    
    // }
    // else{
    //   flywheel_one.spin(fwd, 100, velocityUnits::pct);
    //   flywheel_two.spin(fwd, 100, velocityUnits::pct);
    // }
  }



void manualBB(){
  while(1){
    flywheel_one.stop(brake);
    flywheel_two.stop(brake);
    int setSpeed = 0;

    if (bL1){
      setSpeed = 1;
    }
    else if (bL2){
      setSpeed = 2;
    }
    else if (bR1){
      setSpeed = 3;
    }
    else{
      setSpeed = 0;
    }
    flywheel_one.spin(fwd, setSpeed, velocityUnits::pct);
    flywheel_two.spin(fwd, setSpeed, velocityUnits::pct);
    if (flywheel_one.velocity(pct) != setSpeed){
      flywheel_one.spin(fwd, 8, voltageUnits::volt);
      flywheel_two.spin(fwd, 50, voltageUnits::volt);
    }
  }
}

int shootingSpeed = 0;

void midLineShooting(){
  shootingSpeed = 3000;
}

void midLineShooting2(){
  shootingSpeed = 5000;
}

int shootingUrMother(){
  double threshold = 0.30;
  while(1){
    flywheel_one.spin(fwd, shootingSpeed, velocityUnits::rpm);
    flywheel_two.spin(fwd, shootingSpeed, velocityUnits::rpm);
       
    }
  }


double baseVoltage = 7;
void baseVoltageAdd(){
  baseVoltage += 0.01;
}

void baseVoltageSub(){
  baseVoltage -= 0.01;
}

void velocityAndVoltage(){
  flywheel_one.spin(fwd, baseVoltage, voltageUnits::volt);
  flywheel_two.spin(fwd, baseVoltage, voltageUnits::volt);
}

void turnOff(){
  flywheel_one.stop(coast);
  flywheel_two.stop(coast);
  // flywheel_three.stop(coast);
}


int intakeCount = 0;
void intakeAdd(){
  intakeCount += 1;
  waitUntil(!Controller.ButtonY.pressing());
}
  
void intakeReverse(){
  if(intakeCount >= 9999){
    intakeCount = 0;
  }
  else if(intakeCount != 10000){
    intakeCount = 10000;
  }
}
// double prev_rotation;
void intakeSpin(){
  // double error_speed = 0;
  // double desired_speed = 550;
  if(intakeCount %2 == 1){
    // error_speed = desired_speed - conv.velocity(rpm);
    // conv.spin(fwd, desired_speed + error_speed, velocityUnits::rpm);
    intake.spin(fwd, 100, pct);
    
  }
  else if(intakeCount == 10000){
    intake.spin(reverse, 100, velocityUnits::pct);
  }
  else if(intakeCount%2 == 0){
    intake.stop(coast);
  }
}

// void Front_Claw(){
//   if (front_claw.value() == 1){
//     front_claw.set(false);
//   }
//   else{
//     front_claw.set(true);
//   }
// }

// void claw_close(){
//   front_claw.set(true);
// }

// void Back_Claw_Up(){
//   back_claw.set(true);
//   wait(200);
//   back_tilter.set(true);
// }

// void Back_Claw_Down(){
//   back_tilter.set(false);
//   wait(300);
//   back_claw.set(false);
// }

// void tilter_move(){
//   if(back_tilter.value() == 1){
//     back_tilter.set(false);
//   }
//   else{
//     back_tilter.set(true);
//   }
// }

// void Goal_Cover(){
//   if (goal_cover.value() == 1){
//     goal_cover.set(false);
//   }
//   else{
//     goal_cover.set(true);
//   }
// }

// void FlyWheelLift2(){
//   if (flyWheelLift.value() == 1){
//     flyWheelLift.set(0);
//   }
//   else{
//     flyWheelLift.set(1);
//   }
// }

/*void IntakeLift(){
  if (intakeLift.value() == 1){
    intakeLift.set(false);
  }
  else{
    intakeLift.set(true);
  }
}*/


int brakeCount = 0;
void Drivetrain_brake_type(){
  if(brakeCount % 2 == 0){
    Drivetrain.setStopping(hold);
    Controller.rumble("-");
    brakeCount += 1;
  } else{
    Drivetrain.setStopping(brake);
    Controller.rumble("- - -");
    brakeCount += 1;
  }
}


void indexerSpin(){
    if(bR1){
        indexer.rotateFor(reverse, 360, rotationUnits::deg, 100, velocityUnits::pct, false);
    }
}


void intakeSpin2(){
  if (intake.velocity(pct) != 0){
    intake.stop(brake);
  }
  else {
    intake.spin(fwd, 100, velocityUnits::pct);
  }
}

int spinFlywheel;
void flyWheelSpin2(){
  if (flywheel_one.velocity(pct) != 0){
    flywheel.stop(brake);
    spinFlywheel = 0;
  }
  else{
    flywheel_one.spin(fwd, 9, voltageUnits::volt);
    flywheel_two.spin(fwd, 9, voltageUnits::volt);
    spinFlywheel = 1;
  }
}


void opControl(){
  // dtask = vex::task(manualBB);
  // task sideTaskOne = task(sideTaskSlotOne);
  Drivetrain.setStopping(brake);
  // ShootingMode = 1;
  //intakeLift.set(false);
  flyWheelLift.set(false);

  while(1){
    // shootingMode();
    // manualBB();
    // velocityAndVoltage();
    DriveLimit();
    indexerSpin();
    intakeSpin();
    Controller.ButtonL1.pressed(spinFlyWheel1);
    Controller.ButtonL2.pressed(spinFlyWheel2);  
    Controller.ButtonR2.pressed(spinFlyWheel3);  
    Controller.ButtonDown.pressed(baseVoltageAdd);
    // Controller.ButtonUp.pressed(baseVoltageSub);
    Controller.ButtonLeft.pressed(IntakeLift2);
    Controller.ButtonX.pressed(intakeReverse);
    Controller.ButtonY.pressed(intakeAdd);

    if (bRight){
      btask = vex::task(bangbang);
    }

    if (bUp){
      // autoAim(5000, 100);////
      rollerAuton();
      // PIDMove2(-5, 100, 5000, 2, 3, 0, 0);
    }

    if(bB){
      vexcodeInit();
    }
  }   
}