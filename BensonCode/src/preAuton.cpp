#include "vex.h"

int DTtimer = 0;
int Timer1() {
  while (1) {
    DTtimer += 1;
    wait(1);
  }
}
int T2 = 0;
int Timer2() {
  while (1) {
    T2 += 1;
    wait(1);
  }
}
int absTimer = 0;
int absTimer1() {
  while (1) {
    absTimer += 1;
    wait(1);
  }
}


float motorValue(motor motorname) {return motorname.rotation(rotationUnits::deg);}
float opticValue(optical sensor) { return sensor.hue(); }
float lineValue(line sensor) { return sensor.value(analogUnits::mV); }
float lineValuePCT(line sensor) { return sensor.value(analogUnits::pct); }
float encValue(encoder sensor) { return sensor.rotation(rotationUnits::deg); }
float potValue(pot sensor) { return sensor.value(rotationUnits::deg); }
float clawValue(pneumatics claw) { return claw.value(); }

double pi = 3.14159265;

double RADIAN_TO_DEGREES(double radian) { return radian * 180 / pi; }
double DEGREES_TO_RADIAN(double degrees) { return degrees * pi / 180; }



double TICKS_PER_INCH = 90.33;
double getInches(double num) { return num / TICKS_PER_INCH; }

double keepInRange360(double num) {
  while (num >= 360 || num < 0) {
    if (num < 0) {
      num += 360;
    } else {
      num -= 360;
    }
  }
  return num;
}

double keepInNeg180to180(double num) {
  while (num >= 180 || num < -180) {
    if (num >= 180) {
      num -= 360;
    } else {
      num += 360;
    }
  }
  return num;
}
double PYTHAG_THEOREM(double num1, double num2) {
  return sqrt(pow(num1, 2) + pow(num2, 2));
}

void statePositionOdom();

int DTtemp() {
  Brain.Screen.clearScreen();
  while (1) {
    Brain.Screen.printAt(20, 150, "Left front %f ",
                         left_top.temperature(percentUnits::pct));
    Brain.Screen.printAt(20, 170, "Left back %f ",
                         left_bottom.temperature(percentUnits::pct));
    Brain.Screen.printAt(40, 150, "Right front %f ",
                         right_top.temperature(percentUnits::pct));
    Brain.Screen.printAt(40, 170, "Right back %f ",
                         right_bottom.temperature(percentUnits::pct));
    wait(10);
  }
  return 0;
}

int FWstatus(){
  Brain.Screen.clearScreen();
  while(1){
    // Brain.Screen.printAt(20, 150, "1 %f ",
    //                      flywheel_one.power(powerUnits::watt));
    // Brain.Screen.printAt(20, 170, "FW 2 %f ",
    //                      flywheel_two.power(powerUnits::watt));
    // Brain.Screen.printAt(100, 150, "FW 3 %f ",
    //                      flywheel_three.power(powerUnits::watt));
    Brain.Screen.printAt(20, 150, "FW 1 Voltage %f ",
                         flywheel_one.voltage(volt));
    Brain.Screen.printAt(20, 170, "FW 2 Voltage %f ",
                         flywheel_two.voltage(volt));
    Brain.Screen.printAt(20, 190, "FW 1 Velocity %f ",
                         flywheel_one.velocity(rpm));
    Brain.Screen.printAt(20, 210, "FW 2 Velocity %f ",
                         flywheel_two.velocity(rpm));
    wait(10);
  }
  return 0;
}

// void clawCheck(){
//   Controller.Screen.setCursor(3, 1);
//   if (front_claw.value() == 1){
//     Controller.Screen.print("Claw Closed");
//   }
//   else if (front_claw.value() == 0){
//     Controller.Screen.print("Claw Open");
//   }
// }

int controllerPrint(){
  Controller.Screen.clearScreen();
  while(true){
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("D: %f", sqrt(pow(finalGlobalX - 12, 2) + pow(finalGlobalY - 130, 2)));
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("FW V: %f", flywheel.velocity(rpm));
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("FW Volt: %f", flywheel.voltage(volt));
    // clawCheck();


  }
}

int DTVelocity(){
  Brain.Screen.clearScreen();
  while (1) {
    Brain.Screen.printAt(20, 150, "Left front %f ",
                         left_top.velocity(percentUnits::pct));
    Brain.Screen.printAt(20, 170, "Left back %f ",
                         left_bottom.velocity(percentUnits::pct));
    Brain.Screen.printAt(250, 150, "Right front %f ",
                         right_top.velocity(percentUnits::pct));
    Brain.Screen.printAt(250, 170, "Right back %f ",
                         right_bottom.velocity(percentUnits::pct));
    wait(10);
  }
  return 0;
}

void preauton(){
  vexcodeInit();//Inertial calibration

  //task BrainUI(showSortColor);
  task ConGUI(controllerPrint);
  task timer1(Timer1);//Timer
  task timer2(Timer2);
  task absoluteTimer(absTimer1);
  task speedTest(DTVelocity);
  //task tempDisplay(DTtemp);
  task odoData(statePosition);//Odometry Data
}



void trackingWheelReset(){
  encB.resetRotation();
  encL.resetRotation();
  encR.resetRotation();
}