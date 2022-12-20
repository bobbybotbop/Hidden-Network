using namespace vex;

extern int DTtimer;
int Timer1();
extern int T2;
int Timer2();
extern int absTimer;
int absTimer1();


float motorValue(motor motorname);
float opticValue(optical sensor);
float lineValue(line sensor);
float lineValuePCT(line sensor);
float encValue(encoder sensor);
float potValue(pot sensor);

extern double pi;

double RADIAN_TO_DEGREES(double radian);
double DEGREES_TO_RADIAN(double degrees);



extern double TICKS_PER_INCH;
double getInches(double num);

double keepInRange360(double num);

double keepInNeg180to180(double num);

double PYTHAG_THEOREM(double num1, double num2);

void statePositionOdom();

int BrainPrint();

int controllerPrint();

void trackingWheelReset();