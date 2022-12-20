using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller;
extern triport Expander;

extern motor left_top;
extern motor left_bottom;
extern motor right_top;
extern motor right_bottom;
extern motor flywheel_one;
extern motor flywheel_two;

extern motor_group left_drive;
extern motor_group right_drive;
extern motor_group Drivetrain;
extern motor_group flywheel;
extern motor intake;
extern motor indexer;
extern inertial Inertial;

extern encoder encB;
extern encoder encL;
extern encoder encR;

extern digital_out flyWheelLift;
extern digital_out intakeLift;

extern vex::distance distSensorL;
extern vex::distance distSensorR;
extern vex::distance front_distance;

extern limit frontLimit;

extern vision Vision;

extern vision::signature RED;
extern vision::signature BLUE;
extern vision::signature YELLOW;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );