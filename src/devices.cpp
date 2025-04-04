#include <devices.hpp>

vex::brain       brain;
vex::controller  controller;

// drivetrain

vex::pneumatics  base_pto(brain.ThreeWirePort.A);

// right side motors
vex::motor       right_motor1(vex::PORT3, vex::ratio6_1, false);
vex::motor       right_motor2(vex::PORT2, vex::ratio6_1, false);
vex::motor       right_motor3(vex::PORT1, vex::ratio6_1, true);

// left side motors
vex::motor       left_motor1(vex::PORT13, vex::ratio6_1, true);
vex::motor       left_motor2(vex::PORT12, vex::ratio6_1, true);
vex::motor       left_motor3(vex::PORT11, vex::ratio6_1, false);

// lb motors

vex::motor       lb_motor(vex::PORT17, vex::ratio18_1, false);

// mogo 
vex::optical     mogo_color(vex::PORT15);

vex::inertial    imu(vex::PORT11);

