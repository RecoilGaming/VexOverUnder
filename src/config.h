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
motor robotIntake = motor(PORT5 , blueCartridge, false);
motor robotHang = motor(PORT6 , greenCartridge, false);
motor leftCata = motor(PORT3, redCartridge, false);
motor rightCata = motor(PORT8, redCartridge, true);
motor_group robotCata = motor_group(leftCata, rightCata);
rotation cataSensor = rotation(PORT12);

digital_out leftWing = digital_out(robotBrain.ThreeWirePort.B);
digital_out rightWing = digital_out(robotBrain.ThreeWirePort.A);