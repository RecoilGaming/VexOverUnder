// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    2, 18, 1, 13    
// Catapult             motor_group   12, 19          
// Intake               motor         20              
// PneumaticE           digital_out   E               
// PneumaticH           digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;

// function definitions
void runIntake(void);
void fireCatapult(void);
void halfCatapult(void);
void resetAll(void);

void auton1(void);
void auton2(void);
void auton3(void);

// competition global
competition Competition;

// intake globals
bool isIntaking = false, isOuttaking = false;
bool wasPressed1 = false, isPressed1 = false;
bool wasPressed2 = false, isPressed2 = false;
int original = 0;

// catapult globals
bool up = false;

void reset() {
  double pos = Catapult.position(degrees);
  
  while (pos >= 360) pos -= 360;
  while (pos <= -360) pos += 360;

  if (Catapult.position(degrees) != pos) Catapult.setPosition(pos, degrees);
}

// setup robot
void preAuton(void) {
  // initialization
  vexcodeInit();
  Catapult.setStopping(hold);
  
  // setting velocities
  Intake.setVelocity(200, rpm);
  Catapult.setVelocity(100, rpm);

  // extending pneumatics
  PneumaticE.set(true);
  PneumaticH.set(true);

  // catapult down
  // 275 - 8 bands
  // 255 - 6 bands
  // 310 - sharvil
}

// autonomous function
void autonomous(void) {
  // reset catapult
  reset();
  Catapult.setPosition(0,degrees);

  // catapult down
  Catapult.spinToPosition(0, degrees);
  Catapult.spinToPosition(-350, degrees);
  
  // drivetrain settings
  Drivetrain.setDriveVelocity(175, rpm);
  Drivetrain.setStopping(hold);

  // auton function
  // auton1();
}

// driver controlled function
void usercontrol(void) {
  // drivetrain settings
  Drivetrain.setDriveVelocity(200, rpm);
  Drivetrain.setTurnVelocity(200, rpm);
  Drivetrain.setStopping(coast);

  // catapult down
  Catapult.spinToPosition(0, degrees);
  Catapult.spinToPosition(-350, degrees);

  while (1) {
    // reset catapult encoders
    reset();

    // robot functions
    runIntake();
    Controller1.ButtonL1.pressed(fireCatapult);
    Controller1.ButtonL2.pressed(halfCatapult);
    Controller1.ButtonX.pressed(resetAll);

    // waiting
    wait(50, msec);
  }
}

// main function
int main() {
  // setting up callbacks
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // initializing robot
  preAuton();

  // prevent exiting the loop
  while (true) wait(100, msec);
}

// intake function
void runIntake(void) {
  // detect controller input
  isPressed1 = Controller1.ButtonR2.pressing();
  isPressed2 = Controller1.ButtonR1.pressing();

  if (!isPressed1 || !isPressed2) {
    // toggle intake / outtake
    if (isPressed1 and !wasPressed1) {
      isIntaking = !isIntaking;
      if (isIntaking && original == 0) original = 1; 
    }

    if (isPressed2 and !wasPressed2) {
      isOuttaking = !isOuttaking;
      if (isOuttaking && original == 0) original = 2; 
    }

    // powering motors
    if (isIntaking && (!isOuttaking || original == 2)) { Intake.spin(reverse); isOuttaking = 0; original = 1; }
    else if (isOuttaking && (!isIntaking || original == 1)) { Intake.spin(forward); isIntaking = 0; original = 2; }
    else Intake.stop();

    // setting booleans
    wasPressed1 = isPressed1, wasPressed2 = isPressed2;
  } else { Intake.stop(); isIntaking = 0; isOuttaking = 0; }
}

// catapult function
// void fireCatapult(void) { Catapult.spinFor(reverse, 1, turns); }

void fireCatapult(void) {
  // if (up == true) {
  //   Catapult.spinToPosition(-365, degrees);
  //   up = false;
  // } else {
  //   Catapult.spinToPosition(-350, degrees);
  //   up = true;
  // }

  Catapult.spinToPosition(-365, degrees);
  reset();
  Catapult.spinToPosition(-350, degrees);
}

void halfCatapult(void) {
  // if (up == true) {
  //   Catapult.spinToPosition(-365, degrees);
  //   up = false;
  // } else {
  //   Catapult.spinToPosition(-250, degrees);
  //   up = true;
  // }

  Catapult.spinToPosition(-365, degrees);
  reset();
  Catapult.spinToPosition(-250, degrees);
}

void resetAll(void) {
  Catapult.spinToPosition(0, degrees);
  PneumaticE.set(false);
  PneumaticH.set(false);
}

// auton on same team side
void auton1(void) {
  // move to goal
  Drivetrain.driveFor(forward, 23.5, inches);
  Drivetrain.turnFor(left, 70, degrees);
  Drivetrain.driveFor(forward, 44, inches);
  Drivetrain.turnFor(right, 70, degrees);

  // drop triball
  Intake.spin(forward);
  wait(1, sec);
  Intake.stop();

  // push triball in
  Drivetrain.driveFor(reverse, 6, inches);
  Drivetrain.turnFor(right, 130, degrees);
  Drivetrain.setDriveVelocity(200, rpm);
  Drivetrain.driveFor(reverse, 15, inches);

  // drive out
  wait(100, msec);
  Drivetrain.driveFor(forward, 1, inches);
  Drivetrain.setDriveVelocity(175, rpm);

  // move toward triball
  Drivetrain.turnFor(left, 17, degrees);
  Intake.spin(reverse);
  Drivetrain.driveFor(forward, 29, inches);

  // aim catapult
  wait(500, msec);
  Drivetrain.turnFor(right, 130, degrees);
}

// auton on other team side
void auton2(void) {
  // move to goal
  Drivetrain.driveFor(forward, 24, inches);
  Drivetrain.turnFor(right, 70, degrees);
  Drivetrain.driveFor(forward, 45, inches);
  Drivetrain.turnFor(left, 70, degrees);

  // drop triball
  Intake.spin(forward);
  wait(1, sec);
  Intake.stop();

  // push triball in
  Drivetrain.driveFor(reverse, 6, inches);
  Drivetrain.turnFor(left, 130, degrees);
  Drivetrain.setDriveVelocity(200, rpm);
  Drivetrain.driveFor(reverse, 15, inches);

  // drive out
  wait(100, msec);
  Drivetrain.driveFor(forward, 1, inches);
  Drivetrain.setDriveVelocity(175, rpm);

  // move toward triball
  // Drivetrain.turnFor(left, 17, degrees);
  // Intake.spin(reverse);
  // Drivetrain.driveFor(forward, 29, inches);

  // aim catapult
  // wait(500, msec);
  // Drivetrain.turnFor(right, 130, degrees);
}
