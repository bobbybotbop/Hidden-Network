#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller = controller(primary);
triport Expander = triport(PORT10);

//  Motors
//  Drivetrain
motor left_top = motor(PORT2, ratio6_1, false);
motor left_bottom = motor(PORT3, ratio6_1, true);
motor right_top = motor(PORT1, ratio6_1, true);
motor right_bottom = motor(PORT11, ratio6_1, false);

motor_group left_drive = motor_group(left_top, left_bottom);
motor_group right_drive = motor_group(right_top, right_bottom);
motor_group Drivetrain = motor_group(left_top, left_bottom, right_top, right_bottom);

// Flywheel

motor flywheel_one(PORT15, ratio6_1, true);
motor flywheel_two(PORT16, ratio6_1, false);
motor_group flywheel(flywheel_one, flywheel_two);

motor intake(PORT13, true);
motor indexer(PORT4, false);

//Sensors
// Inertial
inertial Inertial = inertial(PORT12);

// Optical shaft encoders
encoder encB = encoder(Expander.E); //E
encoder encR = encoder(Brain.ThreeWirePort.A); //G
encoder encL = encoder(Expander.A); //A

// Color sensor
//color back_optic = color(Brain.ThreeWirePort.D);

// Pneumatics
digital_out flyWheelLift = pneumatics(Brain.ThreeWirePort.C);
digital_out intakeLift = pneumatics(Brain.ThreeWirePort.D);
// pneumatics flyWheelLift = pneumatics(Brain.ThreeWirePort.A);
// pneumatics intakeLift = pneumatics(Brain.ThreeWirePort.F);

// Distance
vex::distance distSensorL = vex::distance(PORT1);
vex::distance distSensorR = vex::distance(PORT2);
vex::distance front_distance = vex::distance(PORT11);

limit frontLimit = limit(Expander.C);

bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Controller.Screen.print("Device initialization...");
  Controller.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200);
  Inertial.calibrate();
  Controller.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (Inertial.isCalibrating()) {
    wait(25);
  }
  // reset the screen now that the calibration is complete
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1,1);
  wait(50);
  Controller.Screen.clearScreen();
}