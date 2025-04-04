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

extern vex::motor       lb_motor;

extern vex::optical     mogo_color;

extern vex::inertial    imu;


#endif // DEVICES_HPP