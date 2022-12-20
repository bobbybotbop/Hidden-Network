/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include <vector>

using namespace vex;
using namespace std;


#include "robot-config.h"
#include "odometry.h"
#include "PID.h"
#include "miscFunctions.h"
#include "preAuton.h"
#include "auton.h"
#include "PP.h"



#define ch4 Controller.Axis4.position(percentUnits::pct)
#define ch3 Controller.Axis3.position(percentUnits::pct)
#define ch1 Controller.Axis1.position(percentUnits::pct)
#define ch2 Controller.Axis2.position(percentUnits::pct)
#define bA Controller.ButtonA.pressing() // Right
#define bAA Controller.ButtonA.pressed()
#define bB Controller.ButtonB.pressing() // Down
#define bX Controller.ButtonX.pressing() // Up
#define bY Controller.ButtonY.pressing()
#define bDown Controller.ButtonDown.pressing()
#define bUp Controller.ButtonUp.pressing()
#define bLeft Controller.ButtonLeft.pressing()
#define bRight Controller.ButtonRight.pressing()
#define bL1 Controller.ButtonL1.pressing()
#define bL2 Controller.ButtonL2.pressing()
#define bR2 Controller.ButtonR2.pressing()
#define bR1 Controller.ButtonR1.pressing()
#define wait vex::task::sleep

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5);                                                                   \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)


