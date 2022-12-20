#include "vex.h"

void rollerAuton(){
  setPositionOdom(30, 13, 180);
  flyWheelSpin(400);
  // btask = vex::task(bangbang);
  PIDMove2(2, 75, 3000, 1, 3, 0, 0);
  spinIntake();
  wait(75);
  moveToDriveBackMax(30, 15, 100, 3, 0, 2000, 1);
  PIDTurn(175, 100, 4000, 1);
  autoBangBang(390, 7.5, 2, 5000);
  indexer.stop(brake);
  intake.stop(brake);

  // Moves to 3 Stack
  // moveToDriveMax(50, 30, 5000, 3, 0, 5000, 2);
  // IntakeLift2();
  // spinIntake();
  // moveToDriveMax(58, 36, 30, 3, 0, 5000, 1);
  // IntakeLift2();
  // moveToDriveMax(62, 40, 30, 3, 0, 5000, 1);
  // wait(1000);


  moveToDriveMax(60, 37, 100, 10, 0, 5000, 1);
  moveToDriveBackMax(52, 30, 100, 3, 0, 3000, 3);
  spinIntake();
  moveToDriveMax(74, 46, 20, 3, 0, 5000, 2);
  wait(500);
  PIDTurn(120, 50, 5000, 2);
  autoBangBang(340, 7, 3, 5000); /* needs changing */
  // Shoots 3 stack

  moveToDriveMax(84, 60, 50, 3, 0, 5000, 2);
  moveToDriveMax(96, 72, 50, 3, 0, 3000, 2);
  moveToDriveMax(110, 86, 50, 3, 0, 50, 2);
  PIDTurn(110, 50, 5000, 2);
  // Scores individual disks

  moveToDriveMax(132, 116, 100, 3, 0, 5000, 2);
  PIDTurn(270, 100, 3000, 2);
}

void matchLoadAuto(){
  setPositionOdom(72, 13, 180);
  flyWheelSpin(400);
  spinIntake();
  wait(500);
  moveToDriveBackMax(72, 16, 100, 3, 0, 1500, 1);
  PIDTurn(140, 100, 2000, 1);
  autoBangBang(400, 7.6, 3, 5000);
  indexer.stop(brake);
  // Preloads and 1 matchload scored

  PIDTurn(180, 100, 2000, 1);
  PIDMove2(3, 100, 1500, 1, 3, 0, 0);
  wait(3000);
  //Obtained 3 matchloads

  moveToDriveBackMax(72, 16, 100, 3, 0, 1500, 1);
  PIDTurn(150, 100, 2000, 1);
  autoBangBang(400, 7.6, 3, 5000);
  //Scored 3 matchloads

  moveToDriveMax(68, 40, 100, 10, 0, 5000, 1);
  moveToDriveBackMax(72, 20, 100, 3, 0, 3000, 2);
  moveToDriveMax(50, 42, 30, 3, 0, 5000, 2);
  PIDTurn(140, 100, 2000, 1);
  autoBangBang(400, 7.6, 3, 5000);
  // Scores 3 disks

  moveToDriveBackMax(30, 13, 100, 3, 1, 5000, 2);
  PIDTurn(180, 100, 1000, 1);
  PIDMove2(3, 100, 1500, 2, 3, 0, 0);
  wait(200);
  moveToDriveBackMax(30, 15, 100, 3, 0, 1000, 2);
}
