using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern drivetrain Drivetrain;
extern motor_group Catapult;
extern motor Intake;
extern digital_out PneumaticE;
extern digital_out PneumaticH;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );