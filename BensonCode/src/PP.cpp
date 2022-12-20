#include "vex.h"

// vector<double> arrayToVector(double x[]){
//     // int n = sizeof(src) / sizeof(src[0]);
 
//     // std::vector<double> dest(src, src + n);

//     // return vector<double> dest();

//     return(std::vector<double>(x, x + sizeof x / sizeof x[0]));
// }

int sgn (double num){
  if (num > 0) {
    return 1;
  }
  else{
    return -1;
  }
}

vector<vector<double>> lineCircleIntersection(double currentPos[], double pt1[], double pt2[], double lookAheadDis){

  double currentX = currentPos[0];
  double currentY = currentPos[1];
  double x1 = pt1[0];
  double y1 = pt1[1];
  double x2 = pt1[0];
  double y2 = pt1[1];

  bool intersectionFound = false;

  double x1_offset = x1 - currentX;
  double y1_offset = y1 - currentY;
  double x2_offset = x2 - currentX;
  double y2_offset = y2 - currentY;

  double dx = x2_offset - x1_offset;
  double dy = y2_offset - y1_offset;
  double dr = sqrt(pow(dx, 2) + pow(dy, 2));
  double D = x1_offset*y2_offset - x2_offset*y1_offset;
  double discriminant = ((pow(lookAheadDis, 2)*pow(dr,2)) - pow(D, 2));

  vector<vector<double>> solutions;

  if (discriminant >= 0){
    intersectionFound = true;

    double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant));
    // Check if sqrt vs the np.sqrt difference

    double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant));

    double sol_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
    double sol_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);    


    double sol1[2] = {sol_x1 + currentX, sol_y1 + currentY};
    double sol2[2] = {sol_x2 + currentX, sol_y2 + currentY};


    vector<double> solVec1;
    vector<double> solVec2;  



    double minX = min(x1, x2);
    double maxX = max(x1, x2);
    double minY = min(y1, y2);
    double maxY = max(y1, y2);


    if (minX <= sol1[0] <= maxX && minY <= sol1[1] <= maxY){
      solVec1.push_back(sol1[0]);
      solVec1.push_back(sol1[1]);
    }
    if (minX <= sol2[0] <= maxX && minY <= sol2[1] <= maxY){
      solVec2.push_back(sol2[0]);
      solVec2.push_back(sol2[1]);
    }    

    solutions.push_back(solVec1);
    solutions.push_back(solVec2);
  }
  return solutions;
}


double pt_to_pt_distance(double pt1[2], double pt2[2]){
  double Distance = sqrt(pow(pt2[0] - pt1[0], 2) + (pow(pt2[1] - pt1[1], 2)));
  return Distance;
}

// vector<double> goal_pt_search(vector<vector<double>> path, double currentPos[2], double lookAheadDis, double lastFoundIndex){
  
//   double currentX = currentPos[0];
//   double currentY = currentPos[1];

//   double goalPoint[2];

//   double intersectFound = false;
//   double startingIndex = lastFoundIndex;

//   for (int i = 0; startingIndex < i && i <= path.size(); i++){
//     double x1 = path[i][0] - currentX;
//     double y1 = path[i][0] - currentY;
//     double x2 = path[i+1][0] - currentX;
//     double y2 = path[i+1][0] - currentY;
//     double dx = x2 - x1;
//     double dy = y2 - y1;
//     double dr = sqrt(pow(dx, 2) + pow(dx, 2));
//     double D = x1*x2 - x2*y1;
//     double discriminant = ((pow(lookAheadDis, 2)*pow(dr,2)) - pow(D, 2));

//     if (discriminant >= 0){
//       double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant));
//       // Check if sqrt vs the np.sqrt difference

//       double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant));

//       double sol_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
//       double sol_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);    


//       double sol_pt1[2] = {sol_x1 + currentX, sol_y1 + currentY};
//       double sol_pt2[2] = {sol_x2 + currentX, sol_y2 + currentY};

//       double minX = min(path[i][0], path[i+1][0]);
//       double minY = min(path[i][1], path[i+1][1]);
//       double maxX = max(path[i][0], path[i+1][0]);
//       double maxY = max(path[i][1], path[i+1][1]);

//       if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY))||((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))){

//         intersectFound = true;
//         double pathPoint[2];
//         pathPoint[0] = path[i+1][0];
//         pathPoint[1] = path[i+1][1];

//         if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)) && ((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))){

//           if (pt_to_pt_distance(sol_pt1, pathPoint) < pt_to_pt_distance(sol_pt2, pathPoint)){
//             goalPoint[0] = sol_pt1[0];
//             goalPoint[1] = sol_pt1[1];
//           }
//           else {
//             goalPoint[0] = sol_pt2[0];
//             goalPoint[1] = sol_pt2[1];
//           }

//         }
//         else{

//           if ((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)){
//             goalPoint[0] = sol_pt1[0];
//             goalPoint[1] = sol_pt1[1];         
//           }
//           else{
//             goalPoint[0] = sol_pt2[0];
//             goalPoint[1] = sol_pt2[1];
//           }
//         }
        
//         if (pt_to_pt_distance(goalPoint, pathPoint) < pt_to_pt_distance(currentPos, pathPoint)){
//           lastFoundIndex = i;
//           break;
//         }
//         else{
//           lastFoundIndex = i+1;
//           break;
//         }
//       }
//       else{
//         intersectFound = false;
//         goalPoint[0] = path[lastFoundIndex][0];
//         goalPoint[1] = path[lastFoundIndex][1];
//       }
//     }
//   }
// }


double robotWidth = 0;

double findMinAngle(double absTargetAngle, double currentHeading){
  double minAngle = absTargetAngle - currentHeading;

  if (minAngle > 180 || minAngle < -180){
    minAngle = -(sgn(minAngle)) * (360 - fabs(minAngle));
  }

  return minAngle;
}

vector<double> move_to_point(double targetPoint[2]){
  double kp_lin = 20;
  double kp_turn = 1;

  double linearError = sqrt(pow(targetPoint[1]-finalGlobalY, 2) + pow(targetPoint[0] - finalGlobalX, 2));
  double absTargetAngle = atan2(targetPoint[1] - finalGlobalY, targetPoint[0] - finalGlobalX) * 180/M_PI;
  if (absTargetAngle < 0){
    absTargetAngle += 360;
  }
  double turnError = absTargetAngle - angleRad;
  if (turnError > 180 || turnError < -180){
    turnError = -(sgn(turnError) * (360 - fabs(turnError)));
  }

  double linearVel = kp_lin * linearError;
  double turnVel = kp_turn * turnError;

  vector<double> velocities = {linearVel, turnVel};
  return velocities;
}




void purePursuit(const vector<vector<double>> path, double lookAheadDis, double LFIndex, double driveMax, double drivekp, double drivekd, double timeout, double driveErrorMargin){

  double currentPos[2] = {finalGlobalX, finalGlobalY};
  double lastFoundIndex = LFIndex;
  double goalPoint[2];

  double intersectFound = false;
  double startingIndex = lastFoundIndex;

  for (int i = 0; startingIndex < i && i <= path.size(); i++){
    double x1 = path[i][0] - finalGlobalX;
    double y1 = path[i][0] - finalGlobalY;
    double x2 = path[i+1][0] - finalGlobalX;
    double y2 = path[i+1][0] - finalGlobalY;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = sqrt(pow(dx, 2) + pow(dx, 2));
    double D = x1*x2 - x2*y1;
    double discriminant = ((pow(lookAheadDis, 2)*pow(dr,2)) - pow(D, 2));

    if (discriminant >= 0){
      double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant));
      // Check if sqrt vs the np.sqrt difference

      double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant));

      double sol_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
      double sol_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);    


      double sol_pt1[2] = {sol_x1 + finalGlobalX, sol_y1 + finalGlobalY};
      double sol_pt2[2] = {sol_x2 + finalGlobalX, sol_y2 + finalGlobalY};

      double minX = min(path[i][0], path[i+1][0]);
      double minY = min(path[i][1], path[i+1][1]);
      double maxX = max(path[i][0], path[i+1][0]);
      double maxY = max(path[i][1], path[i+1][1]);

      if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY))||((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))){

        intersectFound = true;
        double pathPoint[2];
        pathPoint[0] = path[i+1][0];
        pathPoint[1] = path[i+1][1];

        if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)) && ((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))){

          if (pt_to_pt_distance(sol_pt1, pathPoint) < pt_to_pt_distance(sol_pt2, pathPoint)){ 
            goalPoint[0] = sol_pt1[0];
            goalPoint[1] = sol_pt1[1];
          }
          else {
            goalPoint[0] = sol_pt2[0];
            goalPoint[1] = sol_pt2[1];
          }

        }
        else{

          if ((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)){
            goalPoint[0] = sol_pt1[0];
            goalPoint[1] = sol_pt1[1];         
          }
          else{
            goalPoint[0] = sol_pt2[0];
            goalPoint[1] = sol_pt2[1];
          }
        }
        
        if (pt_to_pt_distance(goalPoint, pathPoint) < pt_to_pt_distance(currentPos, pathPoint)){
          lastFoundIndex = i;
          break;
        }
        else{
          lastFoundIndex = i+1;
          break;
        }
      }
      else{
        intersectFound = false;
        goalPoint[0] = path[lastFoundIndex][0];
        goalPoint[1] = path[lastFoundIndex][1];
      }
    }

    vector<double> velocities = move_to_point(goalPoint);
    double turnVel =  ((robotWidth * sin(turnError)/lookAheadDis)*velocities[0]);

    double rightSpeed = velocities[0] + turnVel;
    double leftSpeed = velocities[0] - turnVel;

    if (rightSpeed > driveMax){
      rightSpeed = driveMax;
    }
    if (rightSpeed < -driveMax){
      rightSpeed = -driveMax;
    }

    if (leftSpeed > driveMax){
      leftSpeed = driveMax;
    }
    if (leftSpeed < -driveMax){
      leftSpeed = -driveMax;
    }

    right_drive.spin(fwd, rightSpeed,velocityUnits::pct);
    left_drive.spin(fwd, leftSpeed, velocityUnits::pct);



    // moveToDriveMax(goalPoint[0], goalPoint[1], driveMax, drivekp, drivekd, timeout, driveErrorMargin);
  }
}

void purePursuitOdom(const vector<vector<double>> path, double lookAheadDis, double LFIndex, double driveMax, double drivekp, double drivekd, double timeout, double driveErrorMargin){

  double currentPos[2] = {finalGlobalX, finalGlobalY};
  double lastFoundIndex = LFIndex;
  double goalPoint[2];

  double intersectFound = false;
  double startingIndex = lastFoundIndex;

  for (int i = 0; startingIndex < i && i <= path.size(); i++){
    double x1 = path[i][0] - finalGlobalX;
    double y1 = path[i][0] - finalGlobalY;
    double x2 = path[i+1][0] - finalGlobalX;
    double y2 = path[i+1][0] - finalGlobalY;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = sqrt(pow(dx, 2) + pow(dx, 2));
    double D = x1*x2 - x2*y1;
    double discriminant = ((pow(lookAheadDis, 2)*pow(dr,2)) - pow(D, 2));

    if (discriminant >= 0){
      double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant));
      // Check if sqrt vs the np.sqrt difference

      double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant));

      double sol_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
      double sol_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);    


      double sol_pt1[2] = {sol_x1 + finalGlobalX, sol_y1 + finalGlobalY};
      double sol_pt2[2] = {sol_x2 + finalGlobalX, sol_y2 + finalGlobalY};

      double minX = min(path[i][0], path[i+1][0]);
      double minY = min(path[i][1], path[i+1][1]);
      double maxX = max(path[i][0], path[i+1][0]);
      double maxY = max(path[i][1], path[i+1][1]);

      if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY))||((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))){

        intersectFound = true;
        double pathPoint[2];
        pathPoint[0] = path[i+1][0];
        pathPoint[1] = path[i+1][1];

        if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)) && ((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))){

          if (pt_to_pt_distance(sol_pt1, pathPoint) < pt_to_pt_distance(sol_pt2, pathPoint)){ 
            goalPoint[0] = sol_pt1[0];
            goalPoint[1] = sol_pt1[1];
          }
          else {
            goalPoint[0] = sol_pt2[0];
            goalPoint[1] = sol_pt2[1];
          }

        }
        else{

          if ((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)){
            goalPoint[0] = sol_pt1[0];
            goalPoint[1] = sol_pt1[1];         
          }
          else{
            goalPoint[0] = sol_pt2[0];
            goalPoint[1] = sol_pt2[1];
          }
        }
        
        if (pt_to_pt_distance(goalPoint, pathPoint) < pt_to_pt_distance(currentPos, pathPoint)){
          lastFoundIndex = i;
          break;
        }
        else{
          lastFoundIndex = i+1;
          break;
        }
      }
      else{
        intersectFound = false;
        goalPoint[0] = path[lastFoundIndex][0];
        goalPoint[1] = path[lastFoundIndex][1];
      }
    }

    // vector<double> velocities = move_to_point(goalPoint);
    // double turnVel =  ((robotWidth * sin(turnError)/lookAheadDis)*velocities[0]);

    // double rightSpeed = velocities[0] + turnVel;
    // double leftSpeed = velocities[0] - turnVel;

    // if (rightSpeed > driveMax){
    //   rightSpeed = driveMax;
    // }
    // if (rightSpeed < -driveMax){
    //   rightSpeed = -driveMax;
    // }

    // if (leftSpeed > driveMax){
    //   leftSpeed = driveMax;
    // }
    // if (leftSpeed < -driveMax){
    //   leftSpeed = -driveMax;
    // }

    // right_drive.spin(fwd, rightSpeed,velocityUnits::pct);
    // left_drive.spin(fwd, leftSpeed, velocityUnits::pct);
    moveToDriveMax(goalPoint[0], goalPoint[1], driveMax, drivekp, drivekd, timeout, driveErrorMargin);
  }
}