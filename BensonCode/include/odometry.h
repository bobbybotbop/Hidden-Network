// CONSTANTS & VARIABLES
// Encoder values
extern double currentL;
extern double currentR;
extern double currentB;
// Change in encoder values
extern double deltaLeft;
extern double deltaRight;
extern double deltaBack;
// Encoder value before last reset
extern double prevLeft;
extern double prevRight;
extern double prevBack;
// Robot orientation/heading/angle
extern double angleDeg;
extern double angleRad;
extern double deltaAngle;
extern double prevAngleRad;
// Change in x and y coordinates
extern double xCoordinateChange;
extern double yCoordinateChange;
// Polar coordinates
extern double polarCoordinateAngle;
extern double polarRadius;
// Absolute x and y coordinates
extern double finalGlobalX;
extern double finalGlobalY;
extern double prevGlobalX;
extern double prevGlobalY;
// PD loop constants & variables
extern double turnError;
extern double driveError;
extern double prevTurnError;
extern double prevDriveError;
extern double turnDerivative;
extern double driveDerivative;

extern double IMUAngleRad;
extern double IMUAngle;
extern double deltaIMUAngle;
extern double prevImuAngleRad;

extern double LEFT_TRACK;  // Left tracking wheel distance from tracking center
extern double RIGHT_TRACK; // right tracking wheel distance from tracking center
extern double BACK_TRACK;  // back tracking wheel distance from tracking center

extern double inchPerTick;;
extern double inchPerTickBack;

void getEncValue();

void positionUpdate();

int statePosition();

void setPositionOdom(double X, double Y,double theta);

int driveSpeedCalc(double distError, double maxSpeed, double totalError, double deltaError, double kp, double ki, double kd);

int turnSpeedCalc(double angleError, double maxSpeed, double turnTotalError, double turnDeltaError, 
                  double turnKp, double turnKi, double turnKd);

double increase_max(double num1, double num2, double max_speed);

void moveToDrive(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin);
            
void moveToDriveMax(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin);

void moveToDriveBack(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin);

void moveToDriveBackMax(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin);

void moveToDrive_limit(double coorX, double coorY, double driveMax, double drivekp,
            double drivekd, double timeout, double driveErrorMargin);