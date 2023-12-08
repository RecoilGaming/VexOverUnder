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

motor driveLF = motor(PORT19, greenCartridge, true);
motor driveLB = motor(PORT20, greenCartridge, true);
motor_group leftDrive = motor_group(driveLF, driveLB);
motor driveRF = motor(PORT17, greenCartridge, false);
motor driveRB = motor(PORT18, greenCartridge, false);
motor_group rightDrive = motor_group(driveRF, driveRB);
drivetrain robotDrive = drivetrain(leftDrive, rightDrive, 4*M_PI, 11, 11, inches, 3/7);

motor robotIntake = motor(PORT16 , blueCartridge, true);
motor leftCata = motor(PORT14, redCartridge, true);
motor rightCata = motor(PORT15, redCartridge, false);
motor_group robotCata = motor_group(leftCata, rightCata);
rotation cataSensor = rotation(PORT4);

digital_out leftWing = digital_out(robotBrain.ThreeWirePort.H);
digital_out rightWing = digital_out(robotBrain.ThreeWirePort.E);

// function definitions
void resetCata(void);
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
int outtakeTime = 0;

// spin catapult with sensor
void spinCatapultTo(int pos, directionType dir) {
  while (cataSensor.position(degrees) != pos) {
    robotCata.spin(dir, 100, rpm);
  } robotCata.stop();
}

// robot setup
void preAuton(void) {
  task controllerInputTask(runDrivetrain);
  resetCata();
  
  robotDrive.setDriveVelocity(100, percent);
  robotDrive.setTurnVelocity(100, percent);
  robotDrive.setStopping(hold);
  robotIntake.setVelocity(100, percent);
  robotCata.setVelocity(90, percent);
  robotCata.setStopping(hold);

  robotCata.setPosition(0, deg);
  robotCata.spinToPosition(60, deg);
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
    resetCata();

    robotController.ButtonL2.pressed(toggleCatapult);
    robotController.ButtonR2.pressed(toggleIntake);
    robotController.ButtonR2.released(toggleOuttake);
    robotController.ButtonA.pressed(toggleWings);
    robotController.ButtonL1.pressed(toggleLeftWing);
    robotController.ButtonR1.pressed(toggleRightWing);

    if (outtakeTime > 25) { robotIntake.stop(); outtakeTime = 0; }

    robotBrain.Screen.print(robotCata.position(deg));
    wait(50, msec);
    robotBrain.Screen.clearLine();
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
void resetCata(void) {
  double pos = robotCata.position(deg);// catapultSensor.position(deg);
  
  while (pos >= 360) pos -= 360;
  while (pos <= -360) pos += 360;

  if (robotCata.position(deg) != pos) robotCata.setPosition(pos, deg);
  // if (catapultSensor.position(deg) != pos) catapultSensor.setPosition(pos, deg);
}

// increase counters
void increaseCounters(void) {
  if (robotIntake.velocity(percent)) outtakeTime++;
}

// drivetrain function
int runDrivetrain(void) {
  while (1) {
    int speedLeft = robotController.Axis3.position() + robotController.Axis1.position();
    int speedRight = robotController.Axis3.position() - robotController.Axis1.position();
    
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
  outtakeTime = 0;
  robotIntake.spin(reverse);
}

// catapult function
void toggleCatapult(void) {
  robotCata.spinFor(360, deg);
  // robotCata.spinToPosition(-365, deg);
  // resetCata();
  // robotCata.spinToPosition(-20, deg);
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

// offensive side auton
void auton1(void) {
  robotDrive.driveFor(24, inches);
}

// defensive side auton
void auton2(void) { }

// skills test auton
void auton3(void) { }

// extra auton ;)
void auton4(void) { }