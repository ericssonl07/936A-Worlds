#include <definitions.hpp>
#include <devices.hpp>
#include <threads.hpp>

// Returns: 0 for red, 1 for blue, 2 for yellow, -1 if no match
static inline int detectColor(double hue) {
    // Red typically ranges from ~0-30 and ~330-360 in HSV color space
    if (hue <= 30 || hue >= 330) {
        return 0; // Red
    }
    // Blue typically ranges from ~170-270
    else if (hue >= 170 && hue <= 270) {
        return 1; // Blue
    }
    // Yellow typically ranges from ~40-80
    else if (hue >= 40 && hue <= 80) {
        return 2; // Yellow
    }

    return -1; // No color match
}

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

/*
hold intake + no mogo
 - load first ring into halfway up the hooks, pto, first stage intake on

hold intake + mogo (no lb)
 - if red -> score
 - if blue -> throw

hold intake + lb
 - if one ring -> load into lb (don't raise), pto, first stage intake on
 - if two rings -> load into lb (don't raise), don't pto, intake off (with hard override)

anti jam?
 - if first stage optical continues to read ring, when intake is spinning and second stage is not reading
    -> reverse intake for a bit, then forward again


    
*/ 

int intake_thread() {
    while (1) {
    //     if (!(!g_intake_ring_queue.empty() && g_intake_ring_queue.front().hooked_stage == get_intake_stage()) && get_intake_hook_pos() > 700 && intake_hook_color.isNearObject()) {
    //         Ring ring;
    //         ring.color = g_team_color;
    //         g_intake_ring_queue.push_front(ring);
    //     }

    //     if (!g_intake_ring_queue.empty()) {
    //         if (g_intake_ring_queue.front().get_pos() < 30) {
    //             g_intake_ring_queue.pop_front();
    //         }
    //     }
    //     brain.Screen.printAt(1, 20, "intake pos: %f, intake stage: %f, hook stage = %f", get_intake_pos(), get_intake_stage(), get_intake_hook_pos());
    //     move_motor(intake_motor, g_intake_power);
    //     vex::this_thread::sleep_for(10);
    // }
        if (g_toggle_hang_piston) {
            hang_piston.open();
        } else {
            hang_piston.close();
        }
        vex::this_thread::sleep_for(10);
    }
    return 0;
}

int lb_thread() {
    double prev_lb_pos = 0;
    while (1) {
        if (get_lb_pos() < 0) {
            lb_motor.resetPosition();
        }

        if (!g_lb_macro_mode) {
            // manual control
            if (g_lb_power<= 0 && get_lb_pos() < 100) {
                move_motor(lb_motor, -10);
            }
            else {
                move_motor(lb_motor, g_lb_power);
            }
        } else {
            g_lb_pid.target(g_lb_target_arm_height);
            g_lb_power = g_lb_pid.calculate_raw(get_lb_pos());
            move_motor(lb_motor, g_lb_power);
        }

        prev_lb_pos = get_lb_pos();
        vex::this_thread::sleep_for(10);
    }
    return 0;
}

int mogo_thread() {
    bool goal_detected = false;
    bool prev_goal_detected = false;
    while (1) {
        // print mogo sensor values
        if (mogo_color.isNearObject() && mogo_color.hue() >= 60 && mogo_color.hue() <= 80) {
            brain.Screen.printAt(1, 20, "mogo hue: %f", mogo_color.hue());
            goal_detected = true;
        } else {
            goal_detected = false;
        }

        if (!prev_goal_detected && goal_detected) {
            // If the goal was previously detected and is now not detected, close the mogo piston
            g_toggle_mogo = true;
        } 

        if (g_toggle_mogo) {
            mogo_piston.open();
        } else {
            mogo_piston.close();
        }

        prev_goal_detected = goal_detected;

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