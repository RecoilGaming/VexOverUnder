#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// gear ratios
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
drivetrain robotDrive = drivetrain(leftDrive, rightDrive, 4*M_PI, 11, 11, inches, 3/7);

motor robotIntake = motor(PORT7, blueCartridge, false);
motor robotCatapult = motor(PORT3, redCartridge, false);
rotation catapultSensor = rotation(PORT4);

motor climbMotorA = motor(PORT8, redCartridge, true);
motor climbMotorB = motor(PORT19, redCartridge, false);
motor_group robotClimb = motor_group(climbMotorA, climbMotorB);

digital_out leftPneumatic = digital_out(robotBrain.ThreeWirePort.E);
digital_out rightPneumatic = digital_out(robotBrain.ThreeWirePort.H);

// controller input
bool stopLeft = true;
bool stopRight = true;

int controllerInput() {
  while (true) {
    int speedLeft = robotController.Axis3.position() - robotController.Axis1.position();
    int speedRight = robotController.Axis3.position() + robotController.Axis1.position();
    
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
void toggleCatapult(void);
void toggleWings(void);
void toggleLeftWing(void);
void toggleRightWing(void);
void climbUp(void);
void climbDown(void);

void auton1(void);
void auton2(void);
void auton3(void);
void auton4(void);

// competition global
competition Competition;

// intake globals
bool intaking = false;
bool outtaking = false;
bool climbing = false;

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
  
  // initial settings
  robotIntake.setVelocity(600, rpm);
  robotCatapult.setVelocity(100, rpm);
  robotClimb.setStopping(hold);
}

// autonomous function
void autonomous(void) {
  // reset catapult
  resetSensor();
  
  // drivetrain settings
  robotDrive.setDriveVelocity(100, percent);
  robotDrive.setStopping(hold);

  // auton function
  auton1();
}

// driver controlled function
void usercontrol(void) {
  // drivetrain settings
  robotDrive.setDriveVelocity(100, percent);
  robotDrive.setTurnVelocity(100, percent);
  robotDrive.setStopping(brake);

  while (1) {
    // reset catapult encoders
    resetSensor();

    // robot functions
    runIntake();
    robotController.ButtonL1.pressed(toggleCatapult);
    robotController.ButtonA.pressed(toggleWings);
    robotController.ButtonLeft.pressed(toggleLeftWing);
    robotController.ButtonRight.pressed(toggleRightWing);
    robotController.ButtonUp.pressed(climbUp);
    robotController.ButtonDown.pressed(climbDown);

    robotBrain.Screen.print(robotClimb.position(degrees));

    // waiting
    wait(50, msec);
    robotBrain.Screen.clearScreen();
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
  bool intake = robotController.ButtonR1.pressing();
  bool outtake = robotController.ButtonR2.pressing();

  // intake / outtake toggle
  if (intake && !outtake && !intaking) {
    robotIntake.spin(forward);
    intaking = 1; outtaking = 0;
  } else if (outtake && !intake && !outtaking) {
    robotIntake.spin(reverse);
    intaking = 0; outtaking = 1;
  } else {
    robotIntake.stop();
    intaking = 0; outtaking = 0;
  }
}

// catapult function
void toggleCatapult(void) {
  // TODO: IMPLEMENT CATAPULT
}

// wings functions
void toggleWings(void) {
  leftPneumatic.set(!leftPneumatic.value());
  rightPneumatic.set(!rightPneumatic.value());
}

void toggleLeftWing(void) { leftPneumatic.set(!leftPneumatic.value()); }
void toggleRightWing(void) { rightPneumatic.set(!rightPneumatic.value()); }

// climbing function
void climbUp(void) { robotClimb.spinToPosition(robotClimb.position(degrees) - 90, degrees); }
void climbDown(void) { robotClimb.spinToPosition(robotClimb.position(degrees) + 90, degrees); }

// auton on same team side
void auton1(void) { }

// auton on other team side
void auton2(void) { }

// auton for skills
void auton3(void) { }

// auton for being carried
void auton4(void) { }