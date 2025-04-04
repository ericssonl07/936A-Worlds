#include <definitions.hpp>
#include <devices.hpp>
#include <threads.hpp>

vex::thread CHASSIS(chassis_thread);
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
    
    bool prev_base_pto_state = false;
   
    while(1) {
        // drivetrain movement
        g_right_power = DRIVE_AXIS_FORWARD - DRIVE_AXIS_TURN;
        g_left_power = DRIVE_AXIS_FORWARD + DRIVE_AXIS_TURN;


        if (PTO_BIND != prev_base_pto_state && prev_base_pto_state != 1){
            g_toggle_base_pto = !g_toggle_base_pto;
        }
        prev_base_pto_state = PTO_BIND;

        // ladybrown movement
        if (LB_RAISE_BIND) {
            g_lb_power = 100;
        } else if (LB_LOWER_BIND) {
            g_lb_power = -100;
        } else {
            g_lb_power = 0;
        }
        
        // Allow other tasks to run
        vex::this_thread::sleep_for(10);
    }
}
