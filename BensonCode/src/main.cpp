/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\zhenb                                            */
/*    Created:      Mon Dec 06 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

void preauton();
void autonomous(){
  // leftAuto();
}

void opControl();

int main() {
  // Competition Callbacks for autonomous and drive control periods
  Competition.autonomous(autonomous);
  Competition.drivercontrol(opControl);

  preauton();
  // Prevent main from exiting with an infinite loop.
  while (1) {

    wait(100); // Sleep the task to prevent wasted resources.
  }
}
