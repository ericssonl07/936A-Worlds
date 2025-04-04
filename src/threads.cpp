#include <definitions.hpp>
#include <devices.hpp>
#include <threads.hpp>

int chassis_thread() {
    while(1) {
        move_motor(right_motor1, g_right_power);
        move_motor(right_motor2, g_right_power);
        move_motor(right_motor3, g_right_power);
        move_motor(left_motor1, g_left_power);
        move_motor(left_motor2, g_left_power);
        move_motor(left_motor3, g_left_power);

        if (g_toggle_base_pto) {
            base_pto.open();
        } else {
            base_pto.close();
        } 
        vex::this_thread::sleep_for(10);
    }

    return 0;
}

int lb_thread() {
    while (1) {
        move_motor(lb_motor, g_lb_power);
        vex::this_thread::sleep_for(10);
    }
    return 0;
}

int mogo_thread() {
    while (1) {
        // print mogo sensor values
        if (mogo_color.isNearObject()) {
            brain.Screen.printAt(1, 20, "1");
            brain.Screen.printAt(1, 40, "hue: %f", mogo_color.hue());
        } else{
            brain.Screen.clearScreen();
            brain.Screen.printAt(1, 20, "0");
        }

        vex::this_thread::sleep_for(10);

    }
    return 0;
}

int odom_thread() {
    while(1) {

        vex::this_thread::sleep_for(10);
    }
    return 0;
}