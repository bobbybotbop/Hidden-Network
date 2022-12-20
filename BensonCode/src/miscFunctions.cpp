#include "vex.h"

void autoClimbCheck(){

  while(true){
    if(Inertial.pitch() > 1 && Inertial.pitch() ){
      PIDMove2(1.2, 90, 1000, 1, 6, 0, 2);
    }
    else if(Inertial.pitch() < -1){
      PIDMove2(-1.6, 90, 1000, 1, 6, 0, 1);
    }
    else if(Inertial.pitch() > -.5 && Inertial.pitch() < .5){
      Drivetrain.stop();
    }
  }
    
}

void FlyWheelLift(){
  if (flyWheelLift.value() == true){
    flyWheelLift.set(false);
  }
  else{
    flyWheelLift.set(true);
  }
}

void IntakeLift2(){
  if (intakeLift.value() == 1){
    intakeLift.set(0);
  }
  else{
    intakeLift.set(1);
  }
}

void autoAim(double timeLimit, double maxSpeed){
  double goalX = 13;
  double goalY = 130;
  double currentY = finalGlobalY;
  double currentX = finalGlobalX;

  double dy = goalY - currentY;
  double dx = goalX - currentX;

  double degreesTurn = atan2(dy, dx)*180/pi;
  PIDTurn(degreesTurn+180, maxSpeed, timeLimit, 3);
}

// void bb(){
//   flywheel_one.spin(fwd, 10, voltageUnits::volt);
//   flywheel_two.spin(fwd, 10, voltageUnits::volt);
// }

// int ShootingMode;
// int shootingMode(){
//   while(1){
//     double Distance = sqrt(pow(finalGlobalX - 12, 2) + pow(finalGlobalY - 130, 2));
//   // double Distance = 40;
//     // double startTime = Brain.timer(msec);
//     if (Distance > 120){
//       ShootingMode = 3;
//     }
//     else if (Distance > 80){
//       ShootingMode = 2;
//     }
//     else if (Distance > 30){
//       ShootingMode = 1;
//     }
//     else{
//       ShootingMode = 0;
//     }
//     wait(10);
//   }

//   return (0);
// }


int runBangBang;
void checkBangBang(){
  if (runBangBang == 0){
    runBangBang = 1;
  }
  else{
    runBangBang = 0;
  }
}

double distanceFromGoal = sqrt(pow(finalGlobalX - 12, 2) + pow(finalGlobalY - 130, 2));

int bangbang(){
  int ShootingMode;
  double TargetSpeed;
  double BasePower;
  double CurrentSpeed = flywheel_one.velocity(pct);
	int MotorPower = 0;
	while(1)
	{
    double Distance = sqrt(pow(finalGlobalX - 12, 2) + pow(finalGlobalY - 130, 2));
  // double Distance = 40;
    // double startTime = Brain.timer(msec);
    if (Distance > 120){
      ShootingMode = 3;
    }
    else if (Distance > 80){
      ShootingMode = 2;
    }
    else if (Distance > 30){
      ShootingMode = 1;
    }
    else{
      ShootingMode = 0;
    }
    wait(10);
		//////SELECT SPEED
		if(ShootingMode == 1)//FULL COURT
		{
			TargetSpeed = 80;
			BasePower = 6;
		}else if(ShootingMode == 2)//MID FIELD
		{
			TargetSpeed = 50;
			BasePower = 7.5; //35
		}else if(ShootingMode == 3)//Barrier Shots
		{
			TargetSpeed = 30;//2150
			BasePower = 2;
		}else if(ShootingMode == 5)//SKILLS
		{
			TargetSpeed = 100;
			BasePower = 8;
		}else if(ShootingMode == 0)//LAUNCHER OFF
		{
			TargetSpeed = 0;
			BasePower = 0;
		}
		///////CONTROL
		if (CurrentSpeed > TargetSpeed){
			MotorPower = BasePower;
		}
		else if (CurrentSpeed < TargetSpeed){
			MotorPower = 8; 
		}
    flywheel_one.spin(fwd, MotorPower, voltageUnits::volt);
    flywheel_two.spin(fwd, MotorPower, voltageUnits::volt);
		wait(20); //OR DELAY 20 MILLI
		////ERROR CALCULATION//
	}
  
  // if (flywheel_one.velocity(pct) != 0){
  //   flywheel.stop(brake);
  //   }
  // else {
  //   flywheel_one.spin(fwd, MotorPower, voltageUnits::volt);
  //   flywheel_two.spin(fwd, MotorPower, voltageUnits::volt);    
  // }
}


void autoBangBang(double BasePower, double TargetSpeed, int numOfDisks, double timeout){
  Brain.resetTimer();
  double threshold = 0.9;
  int n = 0;
  // double TargetSpeed;
  // double BasePower;
  // 	if(ShootingMode == 1)//FULL COURT
	// 	{
	// 		TargetSpeed = 430;
	// 		BasePower = 8;
	// 	}else if(ShootingMode == 2)//MID FIELD
	// 	{
	// 		TargetSpeed = 340;
	// 		BasePower = 7; //35
	// 	}else if(ShootingMode == 3)//Barrier Shots
	// 	{
	// 		TargetSpeed = 360;//2150
	// 		BasePower = 7.4;
	// 	}else
	// 	{
	// 		TargetSpeed = 0;
	// 		BasePower = 0;
	// 	}
  flywheel_one.spin(fwd, BasePower, voltageUnits::volt);
  flywheel_two.spin(fwd, TargetSpeed, volt);
    while (Brain.timer(timeUnits::msec) < timeout){
      while (n < numOfDisks){
          while(flywheel_one.velocity(rpm) < TargetSpeed * threshold){
              flywheel_one.spin(fwd, 10.5, voltageUnits::volt);
              flywheel_two.spin(fwd, 10.5, voltageUnits::volt);
          }
          indexer.spinFor(fwd, 360, deg);
          n += 1;
          wait(25);
        }
  }
}
void flyWheelSpin(double speed){
  flywheel_one.spin(fwd, speed, rpm);
  flywheel_two.spin(fwd, speed, rpm);
}


void spinIntake(double speed){
  intake.spin(fwd, speed, velocityUnits::pct);
}

void shootDisk(double diskCount){

}

vex::task dtask;
vex::task btask;

#define FREE 0
#define SHOOTINGMODE 1
// #define BANGBANG 2


int sideTaskOneHolder = FREE;
int sideTaskTwoHolder = FREE;


int sideTaskSlotOne(){
  while(true){
    if(sideTaskOneHolder == SHOOTINGMODE){
      shootingMode();
      sideTaskOneHolder = FREE;
    }
    wait(25);
  }
  return 0;
}

// int sideTaskSlotTwo(){
//   while(true){
//     if(sideTaskTwoHolder == BANGBANG){
//       bangbang();
//       sideTaskTwoHolder = FREE;
//     }
//     else if (sideTaskTwoHolder == SHOOTINGMODE){

//     }
//     wait(25);
//   }
//   return 0;
// }

void sideTask(int Task){
  if (sideTaskOneHolder == FREE){sideTaskOneHolder = Task;}
  else if(sideTaskTwoHolder == FREE){sideTaskTwoHolder = Task;}
}