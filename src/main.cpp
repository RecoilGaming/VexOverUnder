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

motor driveLF = motor(PORT2, greenCartridge, true);
motor driveLB = motor(PORT1, greenCartridge, true);
motor_group leftDrive = motor_group(driveLF, driveLB);
motor driveRF = motor(PORT9, greenCartridge, false);
motor driveRB = motor(PORT10, greenCartridge, false);
motor_group rightDrive = motor_group(driveRF, driveRB);
drivetrain robotDrive = drivetrain(leftDrive, rightDrive, 319.19, 280, 280, mm, 2.33333333333333333);
motor robotIntake = motor(PORT5 , blueCartridge, true);
motor leftCata = motor(PORT3, redCartridge, false);
motor rightCata = motor(PORT8, redCartridge, true);
motor_group robotCata = motor_group(leftCata, rightCata);
rotation cataSensor = rotation(PORT12);

digital_out leftWing = digital_out(robotBrain.ThreeWirePort.B);
digital_out rightWing = digital_out(robotBrain.ThreeWirePort.A);

// function definitions
void updateVars(void);

int runDrivetrain(void);
void toggleIntake(void);
void startOuttake(void);
void stopOuttake(void);
void startCatapult(void);
void stopCatapult(void);
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
int cataTimer = 0;
int cataTarget = -1;

// spin catapult with sensor
void spinCatapultTo(int pos) {
    robotCata.spin(fwd, 90, percent);
    cataTarget = pos;
    cataTimer = 25;
}

// auton functions
void drive(double dst) { robotDrive.driveFor(dst, inches); wait(100, msec); }
void turn(turnType dir, double rot) { robotDrive.turnFor(dir, rot * 5/9, deg); wait(100, msec); } // 9/49, 72/379 ,    8/43

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
    
    robotDrive.setDriveVelocity(100, percent);
    robotDrive.setTurnVelocity(20, percent);
    robotDrive.setStopping(hold);
    robotIntake.setVelocity(100, percent);
    robotCata.setVelocity(100, percent);
    robotCata.setStopping(hold);
}

// autonomous function
void autonomous(void) { auton3(); }

// driver control function
void usercontrol(void) {
    robotDrive.setDriveVelocity(100, percent);
    robotDrive.setTurnVelocity(50, percent);
    robotDrive.setStopping(brake);

    leftWing.set(false);
    rightWing.set(false);
    spinCatapultTo(300);

    while (1) {
        robotController.ButtonL2.pressed(startCatapult);
        robotController.ButtonR2.pressed(toggleIntake);
        robotController.ButtonX.pressed(startOuttake);
        robotController.ButtonX.released(stopOuttake);
        robotController.ButtonA.pressed(toggleWings);
        robotController.ButtonB.pressed(saveCatapult);
        robotController.ButtonL1.pressed(toggleLeftWing);
        robotController.ButtonR1.pressed(toggleRightWing);

        updateVars();
        stopCatapult();

        // robotBrain.Screen.print(cataSensor.angle(deg));
        wait(20, msec);
        // robotBrain.Screen.clearLine();
    }
}

// main function
int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    preAuton();

    while (1) wait(100, msec);
}

// update booleans
void updateVars(void) {
    stopIntake = robotController.ButtonR2.pressing();
    cataTimer--;
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
void startCatapult(void) {
    spinCatapultTo(270);
    spinCatapultTo(300);
}

void stopCatapult(void) {
    if (cataTimer <= 0 && abs((int)cataSensor.angle(degrees)-cataTarget) <= 5)
        robotCata.stop();
}

// save catapult function
void saveCatapult(void) { spinCatapultTo(300); }

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

// defensive side auton
void auton1(void) {
    drive(17.5);
    turn(left, 52);
    wait(0.5, sec);
    drive(15);
    wait(0.5, sec);
    outtake();
    wait(0.5, sec);
    drive(-10);
    wait(0.5, sec);
    turn(left, 215);
    drive(-16);
    wait(0.5, sec);
    drive(10);
    wait(0.5, sec);
    drive(-15);
    wait(0.5, sec);
    drive(10);
}

// offensive side auton
void auton2(void) {
    drive(17.5);
    turn(right, 52);
    wait(0.5, sec);
    drive(15);
    wait(0.5, sec);
    outtake();
    wait(0.5, sec);
    drive(-10);
    wait(0.5, sec);
    turn(right, 215);
    drive(-16);
    wait(0.5, sec);
    drive(10);
    wait(0.5, sec);
    drive(-15);
    wait(0.5, sec);
    drive(10);
}

// skills test auton
void auton3(void) {
    while (1) {
        spinCatapultTo(270);
        wait(750, msec);
        spinCatapultTo(300);
    }
}

// empty auton
void auton4(void) { }