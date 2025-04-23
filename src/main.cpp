#include <definitions.hpp>
#include <devices.hpp>
#include <threads.hpp>

vex::thread CHASSIS(chassis_thread);
vex::thread INTAKE(intake_thread);
vex::thread LADYBROWN(lb_thread);
vex::thread MOGO(mogo_thread);
vex::thread ODOM(odom_thread);

int main() {
    // Initializing Robot Configuration. DO NOT REMOVE!
    imu.calibrate();
    mogo_color.setLightPower(100);
    mogo_color.setLight(vex::ledState::on);
    mogo_color.objectDetectThreshold(254);

    while (imu.isCalibrating()) {
        vex::this_thread::sleep_for(10);
    }
    
    // end of calibration

    bool prev_mogo_state = false;    
    bool prev_base_pto_state = false;
    
    // Ladybrown control variables
    bool prev_lb_raise_state = false;
    bool prev_lb_lower_state = false;
    int descore_state = 0;

    bool prev_climb_state = false;
   
    while(1) {
        // drivetrain movement
        g_right_power = DRIVE_AXIS_FORWARD - DRIVE_AXIS_TURN;
        g_left_power = DRIVE_AXIS_FORWARD + DRIVE_AXIS_TURN;

        if (PTO_BIND != prev_base_pto_state && prev_base_pto_state != 1){
            g_toggle_base_pto = !g_toggle_base_pto;
        }

        if (MOGO_BIND != prev_mogo_state && prev_mogo_state != 1){
            g_toggle_mogo = !g_toggle_mogo;
        }

        if (HANG_BIND != prev_climb_state && prev_climb_state != 1){
            g_toggle_hang_piston = !g_toggle_hang_piston;
        }

        if (INTAKE_BIND) {
            g_intake_power = 100;
            // if (ladybrown_pos > FLOAT_HEIGHT - 20) {
                // g_store_mode = !OUTTAKE_BIND;
            // }
        } else if (OUTTAKE_BIND) {
            g_intake_power = -100;
        } else {
            g_intake_power = 0;
        }

        // Basic ladybrown movement control
        if (LB_RAISE_BIND) {
            g_lb_power = 100;
        } else if (LB_LOWER_BIND) {
            g_lb_power = -100;
        } else {
            g_lb_power = 0;
        }
        
        // Advanced ladybrown macro controls
        if (LB_RAISE_BIND != prev_lb_raise_state && !prev_lb_raise_state) {
            if (LB_LOWER_BIND) {
                g_lb_macro_mode = 1;
                g_lb_target_arm_height = 100;
                    printf("pee\n");
            } else {
                g_lb_macro_mode = 0;
            }
        }
        
        if (LB_LOWER_BIND != prev_lb_lower_state && !prev_lb_lower_state) {
            if (LB_RAISE_BIND) {
                g_lb_macro_mode = 1;
                descore_state++;
                if (descore_state == 1) {
                    g_lb_target_arm_height = 470;
                    printf("poop\n");
                } else if (descore_state >= 2) {
                    g_lb_target_arm_height = 40;
                }
            } else {
                descore_state = 0;
                g_lb_macro_mode = 0;
            }
        }
        
        // Update previous button states
        prev_base_pto_state = PTO_BIND;
        prev_mogo_state = MOGO_BIND;
        prev_lb_raise_state = LB_RAISE_BIND;
        prev_lb_lower_state = LB_LOWER_BIND;
        prev_climb_state = HANG_BIND;
        
        // Allow other tasks to run
        vex::this_thread::sleep_for(10);
    }
}
