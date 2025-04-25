#include <devices.hpp>

vex::brain       brain;
vex::controller  controller;

// Drivetrain
vex::pneumatics  base_pto(brain.ThreeWirePort.C);

// Right side motors
vex::motor       right_motor1(vex::PORT3, vex::ratio6_1, false);
vex::motor       right_motor2(vex::PORT4, vex::ratio6_1, false);
vex::motor       right_motor3(vex::PORT2, vex::ratio6_1, true);

// Left side motors
vex::motor       left_motor1(vex::PORT19, vex::ratio6_1, true);
vex::motor       left_motor2(vex::PORT20, vex::ratio6_1, true);
vex::motor       left_motor3(vex::PORT17, vex::ratio6_1, false);

// Intake motor
vex::motor       intake_motor(vex::PORT11, vex::ratio6_1, true);
vex::rotation    intake_hook_rotation(vex::PORT5, false);
vex::pneumatics  intake_pto(brain.ThreeWirePort.B);

// Ladybrown motor
vex::motor       lb_motor(vex::PORT10, vex::ratio18_1, false);

// Mogo
vex::optical     mogo_color(vex::PORT12);
vex::pneumatics  mogo_piston(brain.ThreeWirePort.D);

vex::pneumatics  hang_piston(brain.ThreeWirePort.B);

// Inertial sensor
vex::inertial    imu(vex::PORT11);

