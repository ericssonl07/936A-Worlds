#ifndef HIGHSTAKES_HPP
#define HIGHSTAKES_HPP

#include <vex.h>
#include <chassis.hpp>
#include <pid.hpp> // Include PID header

class HighStakesChassis: public Chassis {
private:
    // Base Chassis Devices (already passed via base constructor)
    // MotorGroup* left;
    // MotorGroup* right;
    // vex::rotation* forward_track;
    // vex::rotation* side_track;
    // vex::inertial* imu;
    // vex::controller* controller;

    // High Stakes Specific Devices
    vex::motor* intake_motor;
    vex::motor* lb_motor;
    vex::pneumatics* base_pto;
    vex::pneumatics* intake_pto;
    vex::pneumatics* mogo_piston;
    vex::pneumatics* hang_piston;
    vex::pneumatics* ring_doinker; // Added
    vex::pneumatics* goal_doinker; // Added
    vex::optical* intake_hook_color; // Added
    vex::optical* mogo_color; // Added
    vex::optical* first_stage_color; // Added
    vex::rotation* intake_rotation; // Added


    // Control state variables (previously globals)
    bool toggle_base_pto = false;
    bool toggle_mogo = false;
    bool toggle_hang_piston = false;
    bool toggle_ring_doinker = false; // Added state for new pneumatic
    bool toggle_goal_doinker = false; // Added state for new pneumatic
    double intake_power = 0.0;
    double lb_power = 0.0;
    bool lb_macro_mode = false;
    double lb_target_arm_height = 0.0;
    int descore_state = 0; // For LB macro

    // PID controller for Ladybrown
    PID lb_pid;


    // Configuration (already passed via base constructor)
    // double base_width;
    // double forward_offset;
    // double side_offset;
    // double wheel_radius;
    // double tracking_radius;
    // double external_ratio;
    // double max_radial_acceleration;

    // Helper function to get Ladybrown position (assuming lb_motor has encoder)
    double get_lb_pos() {
        return lb_motor->position(vex::rotationUnits::deg); // Adjust units if necessary
    }

    // Helper function to get Intake Rotation position
    double get_intake_rotation_pos() {
        return intake_rotation->position(vex::rotationUnits::deg); // Adjust units if necessary
    }


public:
    HighStakesChassis(MotorGroup* left, MotorGroup* right, vex::rotation* forward_track, vex::rotation* side_track,
                      vex::inertial* imu, vex::controller* controller,
                      // High Stakes specific devices
                      vex::motor* intake_motor, vex::motor* lb_motor,
                      vex::pneumatics* base_pto, vex::pneumatics* intake_pto,
                      vex::pneumatics* mogo_piston, vex::pneumatics* hang_piston,
                      vex::pneumatics* ring_doinker, vex::pneumatics* goal_doinker, // Added
                      vex::optical* intake_hook_color, vex::optical* mogo_color, // Added
                      vex::optical* first_stage_color, vex::rotation* intake_rotation, // Added
                      // Base config
                      double base_width, double forward_offset, double side_offset, double wheel_radius,
                      double tracking_radius, double external_ratio, double max_radial_acceleration):
        Chassis(left, right, forward_track, side_track, imu, controller, base_width, forward_offset,
                side_offset, wheel_radius, tracking_radius, external_ratio, max_radial_acceleration),
        // Initialize High Stakes members
        intake_motor(intake_motor), lb_motor(lb_motor), base_pto(base_pto), intake_pto(intake_pto),
        mogo_piston(mogo_piston), hang_piston(hang_piston),
        ring_doinker(ring_doinker), goal_doinker(goal_doinker), // Added initialization
        intake_hook_color(intake_hook_color), mogo_color(mogo_color), // Added initialization
        first_stage_color(first_stage_color), intake_rotation(intake_rotation), // Added initialization
        // Initialize LB PID controller (adjust gains as needed from original definitions.hpp/pid.cpp if they existed)
        lb_pid(PID(1.0, 0.1, 0.5, 0.0, 5.0, 100.0, 0.0, 1000.0, 1000.0, 0.0, 50.0, 0.99)) // Placeholder gains
         {
        // Constructor implementation (if any more needed)
        lb_motor->resetPosition(); // Reset LB position on init
        intake_rotation->resetPosition(); // Reset Intake rotation on init
    }

    // Friend declarations for thread functions
    friend int ladybrown_thread(void* o);
    friend int intake_thread(void* o);
    friend int pneumatics_thread(void* o); // Added for pneumatics control
    friend int highstakes_control(void* o);
};

int ladybrown_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::motor* lb_motor_ptr = base->lb_motor; // Get pointer for direct use

    while (true) {
        double current_lb_pos = base->get_lb_pos();
        if (current_lb_pos < 0) {
            lb_motor_ptr->resetPosition();
        }

        if (!base->lb_macro_mode) {
            // Manual control
            // Apply slight holding power when near bottom and moving down/stopped
            if (base->lb_power <= 0 && current_lb_pos < 100) {
                 lb_motor_ptr->spin(vex::directionType::fwd, -10, vex::percentUnits::pct); // Use percent or voltage as appropriate
            } else {
                 lb_motor_ptr->spin(vex::directionType::fwd, base->lb_power, vex::percentUnits::pct);
            }
        } else {
            // Macro mode (PID control)
            base->lb_pid.target(base->lb_target_arm_height);
            // Use calculate_raw if the original PID class had it, otherwise use calculate
            double pid_output_power = base->lb_pid.calculate(current_lb_pos, false);
            // Convert PID output (likely voltage) to percent if needed, or adjust PID gains for percent output
            lb_motor_ptr->spin(vex::directionType::fwd, pid_output_power, vex::voltageUnits::volt); // Assuming PID outputs voltage
        }

        vex::this_thread::sleep_for(10); // Use std::chrono if preferred
    }
    return 0; // Should not be reached
}

int intake_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::motor* intake_motor_ptr = base->intake_motor;

    while (true) {
        // Control intake motor based on power variable
        intake_motor_ptr->spin(vex::directionType::fwd, base->intake_power, vex::percentUnits::pct);

        // Add anti-jam logic or other intake controls here if needed

        vex::this_thread::sleep_for(10);
    }
    return 0; // Should not be reached
}

// Separate thread for pneumatics as they were mixed in original threads
int pneumatics_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::pneumatics* base_pto_ptr = base->base_pto;
    vex::pneumatics* mogo_piston_ptr = base->mogo_piston;
    vex::pneumatics* hang_piston_ptr = base->hang_piston;
    vex::pneumatics* ring_doinker_ptr = base->ring_doinker; // Added
    vex::pneumatics* goal_doinker_ptr = base->goal_doinker; // Added
    vex::optical* mogo_color_ptr = base->mogo_color; // Added for potential auto-clamp
    // vex::pneumatics* intake_pto_ptr = base->intake_pto; // Control this if needed

    while (true) {
        if (base->toggle_base_pto) {
            base_pto_ptr->open();
        } else {
            base_pto_ptr->close();
        }

        // Add mogo color sensor logic here if needed to auto-trigger mogo piston
        // Example: if (mogo_color_ptr->isNearObject() && mogo_color_ptr->color() == vex::color::red /* or desired color */) { base->toggle_mogo = true; }
        if (base->toggle_mogo) {
            mogo_piston_ptr->open();
        } else {
            mogo_piston_ptr->close();
        }

        if (base->toggle_hang_piston) {
            hang_piston_ptr->open();
        } else {
            hang_piston_ptr->close();
        }

        // Control for new pneumatics
        if (base->toggle_ring_doinker) {
            ring_doinker_ptr->open();
        } else {
            ring_doinker_ptr->close();
        }

        if (base->toggle_goal_doinker) {
            goal_doinker_ptr->open();
        } else {
            goal_doinker_ptr->close();
        }


        vex::this_thread::sleep_for(10);
    }
    return 0;
}


int highstakes_control(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::controller* controller_ptr = base->controller; // Use getter method

    // Previous button states for toggle logic
    bool prev_mogo_state = false;
    bool prev_base_pto_state = false;
    bool prev_lb_raise_state = false;
    bool prev_lb_lower_state = false;
    bool prev_climb_state = false;
    bool prev_ring_doinker_state = false; // Added
    bool prev_goal_doinker_state = false; // Added

    // Button mapping (adjust defines/variables as needed)
    #define DRIVE_AXIS_FORWARD controller_ptr->Axis3.position(vex::percentUnits::pct)
    #define DRIVE_AXIS_TURN    controller_ptr->Axis1.position(vex::percentUnits::pct)
    #define PTO_BIND           controller_ptr->ButtonB.pressing() // Example mapping
    #define MOGO_BIND          controller_ptr->ButtonDown.pressing() // Example mapping
    #define HANG_BIND          controller_ptr->ButtonA.pressing() // Example mapping
    #define INTAKE_BIND        controller_ptr->ButtonR1.pressing() // Example mapping
    #define OUTTAKE_BIND       controller_ptr->ButtonR2.pressing()  // Example mapping
    #define LB_RAISE_BIND      controller_ptr->ButtonL1.pressing() // Example mapping
    #define LB_LOWER_BIND      controller_ptr->ButtonL2.pressing() // Example mapping
    #define RING_DOINKER_BIND  controller_ptr->ButtonX.pressing() // Example mapping - Added
    #define GOAL_DOINKER_BIND  controller_ptr->ButtonY.pressing() // Example mapping - Added


    while (true) {
        // Drivetrain control (using Chassis::steer)
        double linear_power = DRIVE_AXIS_FORWARD;
        double turn_power = DRIVE_AXIS_TURN;
        // Convert percent to voltage for steer method
        double left_voltage = (linear_power + turn_power) * 0.128;
        double right_voltage = (linear_power - turn_power) * 0.128;
        base->steer(left_voltage, right_voltage); // Use base class steer method

        // Pneumatics Toggles
        bool current_pto_bind = PTO_BIND;
        if (current_pto_bind != prev_base_pto_state && !prev_base_pto_state){ // Rising edge
            base->toggle_base_pto = !base->toggle_base_pto;
        }
        prev_base_pto_state = current_pto_bind;

        bool current_mogo_bind = MOGO_BIND;
        if (current_mogo_bind != prev_mogo_state && !prev_mogo_state){ // Rising edge
            base->toggle_mogo = !base->toggle_mogo;
        }
        prev_mogo_state = current_mogo_bind;

        bool current_climb_bind = HANG_BIND;
        if (current_climb_bind != prev_climb_state && !prev_climb_state){ // Rising edge
            base->toggle_hang_piston = !base->toggle_hang_piston;
        }
        prev_climb_state = current_climb_bind;

        // Added toggles for new pneumatics
        bool current_ring_doinker_bind = RING_DOINKER_BIND;
        if (current_ring_doinker_bind != prev_ring_doinker_state && !prev_ring_doinker_state){ // Rising edge
            base->toggle_ring_doinker = !base->toggle_ring_doinker;
        }
        prev_ring_doinker_state = current_ring_doinker_bind;

        bool current_goal_doinker_bind = GOAL_DOINKER_BIND;
        if (current_goal_doinker_bind != prev_goal_doinker_state && !prev_goal_doinker_state){ // Rising edge
            base->toggle_goal_doinker = !base->toggle_goal_doinker;
        }
        prev_goal_doinker_state = current_goal_doinker_bind;


        // Intake Control (sets base->intake_power for intake_thread)
        if (INTAKE_BIND) {
            base->intake_power = 100;
        } else if (OUTTAKE_BIND) {
            base->intake_power = -100;
        } else {
            base->intake_power = 0;
        }

        // Ladybrown Control (sets base->lb_power for ladybrown_thread)
        // Basic manual movement
        if (LB_RAISE_BIND) {
            base->lb_power = 100;
            base->lb_macro_mode = false; // Exit macro mode on manual input
            base->descore_state = 0;     // Reset descore state
        } else if (LB_LOWER_BIND) {
            base->lb_power = -100;
            base->lb_macro_mode = false; // Exit macro mode on manual input
            base->descore_state = 0;     // Reset descore state
        } else if (!base->lb_macro_mode) { // Only set power to 0 if not in macro mode and no buttons pressed
             base->lb_power = 0;
        }


        // Advanced ladybrown macro controls (Triggered by pressing both buttons)
        bool current_lb_raise = LB_RAISE_BIND;
        bool current_lb_lower = LB_LOWER_BIND;

        // Raise + Lower pressed simultaneously (rising edge detection for raise)
        if (current_lb_raise != prev_lb_raise_state && !prev_lb_raise_state) { // Rising edge of Raise
            if (current_lb_lower) { // Check if Lower is also pressed
                base->lb_macro_mode = true;
                base->lb_target_arm_height = 100; // Target low height (adjust as needed)
                // printf("Macro: Target Low\\n");
            } else {
                 // If only Raise is pressed (and wasn't before), manual mode takes over above
                 // If Raise is released while Lower is held, nothing happens here, Lower manual takes over
            }
        }

        // Lower + Raise pressed simultaneously (rising edge detection for lower)
        if (current_lb_lower != prev_lb_lower_state && !prev_lb_lower_state) { // Rising edge of Lower
            if (current_lb_raise) { // Check if Raise is also pressed
                base->lb_macro_mode = true;
                base->descore_state++;
                if (base->descore_state == 1) {
                    base->lb_target_arm_height = 470; // First descore height
                    // printf("Macro: Descore 1\\n");
                } else { // >= 2
                    base->lb_target_arm_height = 40; // Second descore height (lower)
                    // printf("Macro: Descore 2+\\n");
                    // Reset state after second press if desired
                    // if (base->descore_state > 2) base->descore_state = 0;
                    base->descore_state = 0;
                }
            } else {
                 // If only Lower is pressed (and wasn't before), manual mode takes over above
                 // If Lower is released while Raise is held, nothing happens here, Raise manual takes over
            }
        }

        // If neither macro combo is active, ensure descore state is reset if not already in macro mode
        if (!current_lb_raise && !current_lb_lower) {
             // If buttons released AND we were in a macro state triggered by BOTH, reset descore?
             // Or maybe only reset descore when manual buttons are pressed (handled above)
             // If we want macro mode to persist until manual override:
             // if (!base->lb_macro_mode) base->descore_state = 0;
             // If we want macro mode to stop when buttons released:
             // base->lb_macro_mode = false; // Uncomment this line if macro should stop on button release
             // base->descore_state = 0;
        }


        // Update previous button states for LB macros
        prev_lb_raise_state = current_lb_raise;
        prev_lb_lower_state = current_lb_lower;

        vex::this_thread::sleep_for(10);
    }
    return 0; // Should not be reached
}


#endif // #ifndef HIGHSTAKES_HPP