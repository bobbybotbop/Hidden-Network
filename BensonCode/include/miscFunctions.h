int bangbang();

int shootingMode();

extern double distanceFromGoal;

void spinIntake(double speed = 100);

// extern int ShootingMode;

#define FREE 0
#define SHOOTINGMODE 1
#define BANGBANG 2

void FlyWheelLift();
void IntakeLift2();

extern int sideTaskOneHolder;
extern int sideTaskTwoHolder;

int sideTaskSlotOne();
int sideTaskSlotTwo();

void sideTask(int Task);

void autoBangBang(double BasePower, double TargetSpeed, int numOfDisks, double timeout);

void flyWheelSpin(double speed);

int check_conv();

void autoClimbCheck();

void autoAim(double timeLimit, double maxSpeed);

void bb();

extern int runBangBang;

void checkBangBang();

extern vex::task dtask;
extern vex::task btask;
