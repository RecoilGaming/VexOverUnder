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
inertial driveInertial = inertial(PORT7);
smartdrive robotDrive = smartdrive(leftDrive, rightDrive, driveInertial, 319.19, 280, 280, mm, 2.33333333333333333);
motor robotIntake = motor(PORT5 , blueCartridge, true);
motor robotHang = motor(PORT6 , greenCartridge, false);
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
void toggleHang(void);
void fixHang(void);
void startCatapult(void);
void stopCatapult(void);
void toggleWings(void);
void toggleLeftWing(void);
void toggleRightWing(void);

// auton definition
void auton0(void);
void auton1(void);
void auton2(void);
void auton3(void);
void auton4(void);
void auton5(void);
void auton6(void);
void auton7(void);
void auton8(void);
void auton9(void);

// competition global
competition Competition;

// robot globals
bool stopLeft = 1;
bool stopRight = 1;
bool prevIntake = 0;
bool hangDown = 0;
int cataTimer = 0;
int cataTarget = -1;

// spin catapult with sensor
void spinCatapultTo(int pos, int time=25) {
    robotCata.spin(fwd, 90, percent);
    cataTarget = pos;
    cataTimer = time;
}

// auton functions
void oldDrive(double dst, float cd = 0.1) { robotDrive.driveFor(dst, inches); wait(cd, sec); }
void oldTurn(turnType dir, double rot, float cd = 0.1) { robotDrive.turnFor(dir, rot * 5/9, deg); wait(cd, sec); } // 9/49, 72/379 ,    8/43
void oldIntake(float cd = 0.5) { robotIntake.spin(forward); wait(1, sec); robotIntake.stop(); wait(cd, sec); }
void oldOuttake(float cd = 0.5) { robotIntake.spin(reverse); wait(1, sec); robotIntake.stop(); wait(cd, sec); }

void drive(double dst, float cd = 0.1) { robotDrive.driveFor(dst, inches); wait(cd, sec); }
void turnTo(double rot, float cd = 0.1) { robotDrive.turnToHeading(rot, degrees); wait(cd, sec); }
void spinIntake(directionType dir = forward, float cd = 0.1) { robotIntake.spin(dir); wait(cd, sec); }
void spinIntakeFor(float dur, directionType dir = forward, float cd = 0.1) { robotIntake.spin(dir); wait(dur, sec); robotIntake.stop(); wait(cd, sec); }
void stopIntake(float cd = 0.1) { robotIntake.stop(); wait(cd, sec); }

// robot setup
void preAuton(void) {
    task controllerInputTask(runDrivetrain);
    
    robotDrive.setDriveVelocity(100, percent);
    robotDrive.setTurnVelocity(60, percent);
    robotDrive.setStopping(hold);
    robotIntake.setVelocity(100, percent);
    robotHang.setVelocity(50, percent);
    robotHang.setStopping(hold);

    robotHang.setPosition(0, degrees);

    robotCata.setVelocity(100, percent);
    robotCata.setStopping(hold);

    driveInertial.calibrate();
    while (driveInertial.isCalibrating()) wait(100, msec);
}

// autonomous function
void autonomous(void) { auton3(); }

// driver control function
void usercontrol(void) {
    robotDrive.setDriveVelocity(100, percent);
    robotDrive.setTurnVelocity(25, percent);
    robotDrive.setStopping(brake);

    leftWing.set(false);
    rightWing.set(false);
    spinCatapultTo(300);

    while (1) {
        robotController.ButtonL1.pressed(toggleLeftWing);
        robotController.ButtonR1.pressed(toggleRightWing);
        robotController.ButtonL2.pressed(startCatapult);
        robotController.ButtonR2.pressed(toggleIntake);
        robotController.ButtonA.pressed(toggleWings);
        robotController.ButtonB.pressed(fixHang);
        robotController.ButtonX.pressed(startOuttake);
        robotController.ButtonX.released(stopOuttake);
        robotController.ButtonY.pressed(toggleHang);

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
    prevIntake = robotController.ButtonR2.pressing();
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
    if (!prevIntake) {
        if (robotIntake.velocity(percent) > 0) robotIntake.stop();
        else robotIntake.spin(forward);
    }
}

// outtake function
void startOuttake(void) { robotIntake.spin(reverse); }
void stopOuttake(void) { robotIntake.stop(); }

// hang function
void toggleHang() {
    if (hangDown) {
        robotHang.spinFor(-90, degrees);
        hangDown = 0;
    } else {
        robotHang.spinFor(90, degrees);
        hangDown = 1;
    }
}

void fixHang() { hangDown = !hangDown; }

// catapult function
void startCatapult(void) {
    spinCatapultTo(270);
    spinCatapultTo(300);
}

void stopCatapult(void) {
    if (cataTimer <= 0 && abs((int)cataSensor.angle(degrees)-cataTarget) <= 5)
        robotCata.stop();
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

// empty auton
void auton0(void) { }

// three balls in (offensive)
void auton1(void) {
    // NOTE: Place robot with intake facing the ball under the hang bar with the intake half an inch from the ball.

    spinIntake();
    drive(2, 0.5);
    stopIntake();
    drive(-24);
    turnTo(315);
    drive(-33.94);
    turnTo(270);
    drive(-10);
    drive(5);
    turnTo(0);
    drive(36);
    turnTo(270);
    drive(24);
    turnTo(180);

}

// one ball in + block (defensive)
void auton2(void) {
    // NOTE: Place robot at a 45 degree angle parallel to match loading bar

    drive(-16.97);
    turnTo(45);
    drive(-16);
    drive(5);
    drive(-10);
    drive(5);
    drive(-10);
    drive(10);
    turnTo(135);
    drive(28);
    turnTo(45);
    drive(28);
}

// one ball in (either)
void auton3(void) {
    robotDrive.setDriveVelocity(100, percent);

    robotDrive.driveFor(-40, inches, false);
    wait(3, seconds);
    drive(10);
    drive(-15, 0.5);
    drive(10);
    drive(-15, 0.5);
    drive(10);
}

// old one ball in (offensive)
void auton7(void) {
    robotDrive.setTurnVelocity(20, percent);
    oldDrive(17.5);
    oldTurn(left, 52, 0.5);
    oldDrive(15, 0.5);
    oldOuttake(1);
    oldDrive(-10, 0.5);
    oldTurn(left, 215, 0.5);
    oldDrive(-16, 0.5);
    oldDrive(10, 0.5);
    oldDrive(-15, 0.5);
    oldDrive(10, 0.5);
}

// old one ball in (defensive)
void auton8(void) {
    robotDrive.setTurnVelocity(20, percent);
    oldDrive(17.5);
    oldTurn(right, 52, 0.5);
    oldDrive(15, 0.5);
    oldOuttake(1);
    oldDrive(-10, 0.5);
    oldTurn(right, 215, 0.5);
    oldDrive(-16, 0.5);
    oldDrive(10, 0.5);
    oldDrive(-15, 0.5);
    oldDrive(10, 0.5);
}

// all balls over (skills)
void auton9(void) {
    spinCatapultTo(300);

    while (1) {
        robotCata.spin(fwd, 90, percent);
        wait(50, msec);

        while (1) {
            if (abs((int)cataSensor.angle(degrees)-300) <= 5) {
                robotCata.stop();
                break;
            }
        } wait(150, msec);
    }
}