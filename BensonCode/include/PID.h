int driveSpeedCalc(double distError, double maxSpeed, double totalError, double deltaError, double kp, double ki, double kd);

int turnSpeedCalc(double angleError, double maxSpeed, double turnTotalError, double turnDeltaError, 
                  double turnKp, double turnKi, double turnKd);

void PIDTurn(float rotateToAngle, float maxSpeed, double timeLimit, double turnErrorMargin, double kp = 1.05, double ki = 0, double kd = 10);

void PIDMove(double dist, double maxSpeed, double timeLimit, double driveErrorMargin, double kp, double ki, double kd);

void PIDMove2(double dist, double maxSpeed, double timeLimit, double driveErrorMargin, double kp, double ki, double kd);

void vision_move(char Color, double drive_max, double drivekp, double drivekd, double turn_max, 
                  double turnkp, double turnkd, double time_limit, bool limit_stop);

void limit_move(int max_iterations, double max_speed, double kp, double kd);

void PIDMove_limit(double dist, double maxSpeed, double timeLimit, double driveErrorMargin, double kp, double ki, double kd);

void auton_climb(int timeout, double target_pitch, double max_pitch, double pitch_kp, double pitch_kd, double target_yaw, double max_yaw, double yaw_kp, double yaw_kd);
