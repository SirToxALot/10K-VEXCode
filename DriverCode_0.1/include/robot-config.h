using namespace vex;

extern brain Brain;

// VEXcode devices
extern rotation rotation1;
extern motor intake;
extern motor cata;
extern controller Controller1;
extern digital_out booster;
extern digital_out lowerEX;
extern inertial inertial1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );