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
motor leftCata = motor(PORT15, redCartridge, false);
motor rightCata = motor(PORT13, redCartridge, true);
motor_group robotCata = motor_group(leftCata, rightCata);
rotation cataSensor = rotation(PORT4);

digital_out leftWing = digital_out(robotBrain.ThreeWirePort.H);
digital_out rightWing = digital_out(robotBrain.ThreeWirePort.E);

// function definitions
void resetCata(void);
void updateBools(void);

int runDrivetrain(void);
void toggleIntake(void);
void startOuttake(void);
void stopOuttake(void);
void toggleCatapult(void);
void saveCatapult(void);
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
bool stopIntake = 0;

// spin catapult with sensor
void spinCatapultTo(int pos, directionType dir) {
  while (cataSensor.position(degrees) != pos) {
    robotCata.spin(dir, 100, rpm);
  } robotCata.stop();
}

// auton functions
void drive(double dst) { robotDrive.driveFor(dst/16, mm); wait(100, msec); }
void turn(turnType dir, double rot) { robotDrive.turnFor(dir, rot/270, deg); wait(100, msec); }
void intake() {
  robotIntake.spin(forward);
  wait(1, sec);
  robotIntake.stop();
  wait(500, msec);
}
void outtake() {
  robotIntake.spin(reverse);
  wait(1, sec);
  robotIntake.stop();
  wait(500, msec);
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
}

// autonomous function
void autonomous(void) { auton2(); }

// driver control function
void usercontrol(void) {
  robotDrive.setDriveVelocity(100, percent);
  robotDrive.setTurnVelocity(100, percent);
  robotDrive.setStopping(brake);

  robotCata.spinFor(60, deg);

  while (1) {
    resetCata();

    robotController.ButtonL2.pressed(toggleCatapult);
    robotController.ButtonR2.pressed(toggleIntake);
    robotController.ButtonX.pressed(startOuttake);
    robotController.ButtonX.released(stopOuttake);
    robotController.ButtonA.pressed(toggleWings);
    robotController.ButtonB.pressed(saveCatapult);
    robotController.ButtonL1.pressed(toggleLeftWing);
    robotController.ButtonR1.pressed(toggleRightWing);

    updateBools();

    robotBrain.Screen.print(stopIntake);
    // robotBrain.Screen.print(robotCata.position(deg));
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
  double pos = robotCata.position(deg); // catapultSensor.position(deg);
  
  while (pos >= 360) pos -= 360;
  while (pos <= -360) pos += 360;

  if (robotCata.position(deg) != pos) robotCata.setPosition(pos, deg);
  // if (catapultSensor.position(deg) != pos) catapultSensor.setPosition(pos, deg);
}

// update booleans
void updateBools(void) { stopIntake = robotController.ButtonR2.pressing(); }

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
void toggleIntake(void) {
  if (!stopIntake) {
    if (robotIntake.velocity(percent) > 0) robotIntake.stop();
    else robotIntake.spin(forward);
  }
}

// outtake function
void startOuttake(void) { robotIntake.spin(reverse); }
void stopOuttake(void) { robotIntake.stop(); }

// catapult function
void toggleCatapult(void) { robotCata.spinFor(360, deg); }

// save catapult function
void saveCatapult(void) { robotCata.spinFor(60, deg); }

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

// defenseive side auton
void auton1(void) {
  // one ball in
  drive(80);
  turn(left, 60);
  robotIntake.spin(reverse);
  wait(1, sec);
  robotIntake.stop();
  wait(500, msec);
  drive(-10);
  turn(left, 120);
  drive(-30);
}

// offensive side auton
void auton2(void) {
  // preload in
  drive(82);
  turn(right, 75);
  drive(1);
  outtake();
  drive(-10);
  turn(right, 140);
  drive(-30);

  // second ball in
  drive(5);
  turn(left, 10);
  drive(50);
  intake();
  turn(right, 135);
  drive(65);
  turn(right, 10);
  outtake();
  drive(-10);
  turn(right, 140);
  drive(-30);
}

// skills test auton
void auton3(void) { }

// extra auton ;) (OLD AUTO ONE)
void auton4(void) {
  // setup
  leftWing.set(true);

  // one ball in
  drive(20);
  turn(right, 45);
  drive(22.5);
  drive(-20);
  drive(22.5);
  wait(500, msec);

  // touch pole
  turn(right, 110);
  drive(54);
  turn(left, 25);
  drive(80);
}