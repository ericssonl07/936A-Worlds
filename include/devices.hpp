#ifndef DEVICES_HPP
#define DEVICES_HPP

#include <definitions.hpp>

extern vex::brain       brain;
extern vex::controller  controller;
extern vex::pneumatics  base_pto;

extern vex::motor       right_motor1;
extern vex::motor       right_motor2;
extern vex::motor       right_motor3;

extern vex::motor       left_motor1;
extern vex::motor       left_motor2;
extern vex::motor       left_motor3;

extern vex::motor       intake_motor;
extern vex::rotation    intake_hook_rotation;
extern vex::optical     intake_hook_color;
extern vex::optical     intake_stage1_color;
extern vex::pneumatics  intake_pto;

extern vex::motor       lb_motor;

extern vex::optical     mogo_color;
extern vex::pneumatics  mogo_piston;

extern vex::pneumatics  hang_piston;

extern vex::inertial    imu;

#endif // DEVICES_HPP