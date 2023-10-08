#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
rotation rotation1 = rotation(PORT5, false);
motor intake = motor(PORT16, ratio6_1, false);
motor cata = motor(PORT21, ratio18_1, false);
controller Controller1 = controller(primary);
digital_out booster = digital_out(Brain.ThreeWirePort.A);
digital_out lowerEX = digital_out(Brain.ThreeWirePort.C);
inertial inertial1 = inertial(PORT14);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}