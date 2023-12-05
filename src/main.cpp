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

motor driveLF = motor(PORT19, greenCartridge, false);
motor driveLB = motor(PORT20, greenCartridge, false);
motor_group leftDrive = motor_group(driveLF, driveLB);
motor driveRF = motor(PORT17, greenCartridge, true);
motor driveRB = motor(PORT18, greenCartridge, true);
motor_group rightDrive = motor_group(driveRF, driveRB);
drivetrain robotDrive = drivetrain(leftDrive, rightDrive, 4*M_PI, 11, 11, inches, 3/7);

motor robotIntake = motor(PORT15 , blueCartridge, false);
motor leftCata = motor(PORT3, redCartridge, false);
motor rightCata = motor(PORT3, redCartridge, true);
motor_group robotCata = motor_group(leftCata, rightCata);
rotation cataSensor = rotation(PORT4);

digital_out leftWing = digital_out(robotBrain.ThreeWirePort.E);
digital_out rightWing = digital_out(robotBrain.ThreeWirePort.H);

// function definitions
void resetCatapult(void);
void increaseCounters(void);

int runDrivetrain(void);
void toggleIntake(void);
void toggleOuttake(void);
void toggleCatapult(void);
void toggleWings(void);
void toggleLeftWing(void);
void toggleRightWing(void);

// auton definition
void auton1(void);
void auton2(void);
void auton3(void);
void auton4(void);

// competition global
competition Competition;

// robot globals
bool stopLeft = 1;
bool stopRight = 1;
bool outtakeTime = 0;

// spin catapult with sensor
void spinCatapultTo(int pos, directionType dir) {
  while (cataSensor.position(degrees) != pos) {
    robotCata.spin(dir, 100, rpm);
  } robotCata.stop();
}

// robot setup
void preAuton(void) {
  task controllerInputTask(runDrivetrain);
  resetCatapult();
  
  robotDrive.setDriveVelocity(100, percent);
  robotDrive.setTurnVelocity(100, percent);
  robotDrive.setStopping(hold);
  robotIntake.setVelocity(100, percent);
}

// autonomous function
void autonomous(void) { auton1(); }

// driver control function
void usercontrol(void) {
  robotDrive.setDriveVelocity(100, percent);
  robotDrive.setTurnVelocity(100, percent);
  robotDrive.setStopping(brake);

  while (1) {
    increaseCounters();
    resetCatapult();

    robotController.ButtonL1.pressed(toggleCatapult);
    robotController.ButtonR2.pressed(toggleIntake);
    robotController.ButtonR2.released(toggleOuttake);
    robotController.ButtonA.pressed(toggleWings);
    robotController.ButtonL2.pressed(toggleLeftWing);
    robotController.ButtonR2.pressed(toggleRightWing);

    if (outtakeTime > 25) robotIntake.stop();

    wait(20, msec);
  }
}

// main function
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  preAuton();

  while (1) wait(100, msec);
}

// reset catapult
void resetCatapult(void) {
  double pos = robotCata.position(deg);// catapultSensor.position(deg);
  
  while (pos >= 360) pos -= 360;
  while (pos <= -360) pos += 360;

  if (robotCata.position(deg) != pos) robotCata.setPosition(pos, deg);
  // if (catapultSensor.position(deg) != pos) catapultSensor.setPosition(pos, deg);
}

// increase counters
void increaseCounters(void) {
  outtakeTime ++;
}

// drivetrain function
int runDrivetrain(void) {
  while (1) {
    int speedLeft = robotController.Axis3.position() - robotController.Axis1.position();
    int speedRight = robotController.Axis3.position() + robotController.Axis1.position();
    
    if (speedLeft < 5 && speedLeft > -5) {
        if (stopLeft) {
            leftDrive.stop();
            stopLeft = 0;
        }
    } else stopLeft = 1;

    if (speedRight < 5 && speedRight > -5) {
        if (stopRight) {
            rightDrive.stop();
            stopRight = 0;
        }
    } else stopRight = 1;
    
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

// intake function
void toggleIntake(void) { robotIntake.spin(forward); }

// outtake function
void toggleOuttake(void) {
  robotIntake.spin(reverse);
  outtakeTime = 0;
}

// catapult function
void toggleCatapult(void) {
  // TODO: IMPLEMENT CATAPULT
}

// wings functions
void toggleWings(void) {
  if (leftWing.value() || rightWing.value()) {
    leftWing.set(false);
    rightWing.set(false);
  } else {
    leftWing.set(true);
    rightWing.set(true);
  }
}

void toggleLeftWing(void) { leftWing.set(!leftWing.value()); }
void toggleRightWing(void) { rightWing.set(!rightWing.value()); }

// auton on same team side
void auton1(void) { }

// auton on other team side
void auton2(void) { }

// auton for skills
void auton3(void) { }

// auton for being carried
void auton4(void) { }