using namespace std;
#include <vector>

int sgn(double num);

vector<vector<double>> lineCircleIntersection(double currentPos[], double pt1[], double pt2[], double lookAheadDis);

double pt_to_pt_distance(double pt1[2], double pt2[2]);

double findMinAngle(double absTargetAngle, double currentHeading);

void purePursuit(const vector<vector<double>> path, double lookAheadDis, double LFIndex, double driveMax, double drivekp, double drivekd, double timeout, double driveErrorMargin);

void purePursuitOdom(const vector<vector<double>> path, double lookAheadDis, double LFIndex, double driveMax, double drivekp, double drivekd, double timeout, double driveErrorMargin);