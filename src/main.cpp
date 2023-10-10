#include "vex.h"
using namespace vex;

using signature = vision::signature;
using code = vision::code;

// constants
const gearSetting redCartridge = ratio36_1;
const gearSetting greenCartridge = ratio18_1;
const gearSetting blueCartridge = ratio6_1;

// robot configuration
brain robotBrain;
controller robotController = controller(primary);
motor leftMotorA = motor(PORT2, greenCartridge, false);
motor leftMotorB = motor(PORT18, greenCartridge, false);
motor_group leftDrive = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT1, greenCartridge, true);
motor rightMotorB = motor(PORT13, greenCartridge, true);
motor_group rightDrive = motor_group(rightMotorA, rightMotorB);
drivetrain robotDrive = drivetrain(leftDrive, rightDrive, 300, 324, 203.2, mm, 1);
motor catapultMotorA = motor(PORT12, redCartridge, false);
motor catapultMotorB = motor(PORT19, redCartridge, true);
motor_group robotCatapult = motor_group(catapultMotorA, catapultMotorB);
motor robotIntake = motor(PORT20, blueCartridge, false);
rotation catapultSensor = rotation(PORT15);
digital_out pneumaticE = digital_out(robotBrain.ThreeWirePort.E);
digital_out pneumaticH = digital_out(robotBrain.ThreeWirePort.H);

// controller input
bool stopLeft = true;
bool stopRight = true;

int controllerInput() {
  while (true) {
    int speedLeft = robotController.Axis3.position() + robotController.Axis1.position();
    int speedRight = robotController.Axis3.position() - robotController.Axis1.position();
    
    if (speedLeft < 5 && speedLeft > -5) {
        if (stopLeft) {
            leftDrive.stop();
            stopLeft = false;
        }
    } else stopLeft = true;

    if (speedRight < 5 && speedRight > -5) {
        if (stopRight) {
            rightDrive.stop();
            stopRight = false;
        }
    } else stopRight = true;
    
    if (stopLeft) {
        leftDrive.setVelocity(speedLeft, percent);
        leftDrive.spin(forward);
    }

    if (stopRight) {
        rightDrive.setVelocity(speedRight, percent);
        rightDrive.spin(forward);
    }
    
    wait(20, msec);
  } return 0;
}

// function definitions
void runIntake(void);
void fireCatapult(void);
void halfCatapult(void);
void resetAll(void);

void auton1(void);
void auton2(void);
void auton3(void);
void auton4(void);

// competition global
competition Competition;

// intake globals
bool isIntaking = false, isOuttaking = false;
bool wasPressed1 = false, isPressed1 = false;
bool wasPressed2 = false, isPressed2 = false;
int original = 0;

// modulo function
void resetSensor(void) {
  double pos = robotCatapult.position(deg);// catapultSensor.position(deg);
  
  while (pos >= 360) pos -= 360;
  while (pos <= -360) pos += 360;

  if (robotCatapult.position(deg) != pos) robotCatapult.setPosition(pos, deg);
  // if (catapultSensor.position(deg) != pos) catapultSensor.setPosition(pos, deg);
}

// spin catapult with sensor
void spinCatapultTo(int pos, directionType dir) {
  while (catapultSensor.position(degrees) != pos) {
    robotCatapult.spin(dir, 100, rpm);
  } robotCatapult.stop();
}

// setup robot
void preAuton(void) {
  // initialization
  task controllerInputTask(controllerInput);
  robotCatapult.setStopping(hold);

  robotCatapult.setPosition(0, deg);
  
  // setting velocities
  robotIntake.setVelocity(600, rpm);
  robotCatapult.setVelocity(100, rpm);
  catapultSensor.setPosition(0, deg);
}

// autonomous function
void autonomous(void) {
  // extending pneumatics
  pneumaticE.set(true);
  pneumaticH.set(true);

  // reset catapult
  resetSensor();

  // catapult down
  // spinCatapultTo(-90, fwd);
  robotCatapult.spinToPosition(0, deg);
  robotCatapult.spinToPosition(-172, deg);
  
  // drivetrain settings
  robotDrive.setDriveVelocity(175, rpm);
  robotDrive.setStopping(hold);

  // auton function
  auton1();
}

// driver controlled function
void usercontrol(void) {
  // extending pneumatics
  pneumaticE.set(true);
  pneumaticH.set(true);

  // drivetrain settings
  robotDrive.setDriveVelocity(200, rpm);
  robotDrive.setTurnVelocity(200, rpm);
  robotDrive.setStopping(coast);

  // catapult down
  robotCatapult.spinToPosition(0, deg);
  robotCatapult.spinToPosition(-172, deg);
  // spinCatapultTo(-90, fwd);

  while (1) {
    // reset catapult encoders
    resetSensor();

    // robot functions
    runIntake();
    robotController.ButtonL1.pressed(halfCatapult);
    robotController.ButtonL2.pressed(fireCatapult);
    robotController.ButtonX.pressed(resetAll);

    // catapult position
    // robotBrain.Screen.print(catapultSensor.position(deg));
    robotBrain.Screen.print(robotCatapult.position(deg));

    // waiting
    wait(50, msec);
    robotBrain.Screen.clearLine();
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
  isPressed1 = robotController.ButtonR2.pressing();
  isPressed2 = robotController.ButtonR1.pressing();

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
    if (isIntaking && (!isOuttaking || original == 2)) { robotIntake.spin(reverse); isOuttaking = 0; original = 1; }
    else if (isOuttaking && (!isIntaking || original == 1)) { robotIntake.spin(forward); isIntaking = 0; original = 2; }
    else robotIntake.stop();

    // setting booleans
    wasPressed1 = isPressed1, wasPressed2 = isPressed2;
  } else { robotIntake.stop(); isIntaking = 0; isOuttaking = 0; }
}

// catapult function
void fireCatapult(void) {
  robotCatapult.spinToPosition(-365, deg);
  resetSensor();
  robotCatapult.spinToPosition(-172, deg);
}

void halfCatapult(void) {
  // if (up == true) {
  //   robotCatapult.spinToPosition(-365, deg);
  //   up = false;
  // } else {
  //   robotCatapult.spinToPosition(-250, deg);
  //   up = true;
  // }

  robotCatapult.spinToPosition(-365, deg);
  resetSensor();
  robotCatapult.spinToPosition(-122, deg);
}

void resetAll(void) {
  robotCatapult.spinToPosition(0, deg);
  pneumaticE.set(false);
  pneumaticH.set(false);
}

// auton on same team side
void auton1(void) {
  // move to goal
  robotDrive.driveFor(forward, 44, inches);
  robotDrive.turnFor(right, 67, deg);

  // drop triball
  robotIntake.spin(forward);
  wait(500, msec);
  robotIntake.stop();

  // push triball in
  robotDrive.driveFor(reverse, 6, inches);
  robotDrive.turnFor(right, 130, deg);
  robotDrive.setDriveVelocity(200, rpm);
  robotDrive.driveFor(reverse, 14, inches);

  // drive out
  wait(100, msec);
  robotDrive.driveFor(forward, 3, inches);
  robotDrive.setDriveVelocity(175, rpm);

  // move toward triball
  robotDrive.turnFor(left, 13, deg);
  robotIntake.spin(reverse);
  robotDrive.driveFor(forward, 28, inches);

  // turn around
  wait(200, msec);
  robotIntake.stop();

  // move toward goal
  robotDrive.turnFor(right, 118, deg);
  robotDrive.driveFor(forward, 21, inches);
  robotDrive.turnFor(right, 25, deg);
  robotDrive.driveFor(forward, 3, inches);

  // drop triball
  robotIntake.spin(forward);
  wait(500, msec);
  robotIntake.stop();
}

// auton on other team side
void auton2(void) {
  // move to goal
  robotDrive.driveFor(forward, 42, inches);
  robotDrive.turnFor(left, 70, deg);

  // drop triball
  robotIntake.spin(forward);
  wait(500, msec);
  robotIntake.stop();

  // push triball in
  robotDrive.driveFor(reverse, 6, inches);
  robotDrive.turnFor(left, 133, deg);
  robotDrive.setDriveVelocity(200, rpm);
  robotDrive.driveFor(reverse, 14, inches);

  // drive out
  wait(100, msec);
  robotDrive.driveFor(forward, 3, inches);
  robotDrive.setDriveVelocity(175, rpm);

  // // turn to triball
  // robotDrive.turnFor(left, 35, deg);
  // robotDrive.driveFor(forward, 2.5, inches);

  // // intake triball
  // robotIntake.spin(reverse);
  // wait(1, sec);
  // robotIntake.stop();
}

// auton for skills
void auton3(void) {
  while (1) {
    robotCatapult.spinToPosition(-365, deg);
    resetSensor();
    robotCatapult.spinToPosition(-172, deg);
    wait(200, msec);
  }
}

void auton4(void) {
  robotDrive.setDriveVelocity(200, rpm);
  robotDrive.drive(reverse);
  wait(3, sec);
  robotDrive.stop();
  robotDrive.driveFor(fwd, 10, inches);
  robotDrive.drive(reverse);
  wait(3, sec);
  robotDrive.stop();
  robotDrive.driveFor(fwd, 10, inches);
}