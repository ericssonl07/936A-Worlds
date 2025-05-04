#ifndef HIGHSTAKES_HPP
#define HIGHSTAKES_HPP

#include <vex.h>
#include <chassis.hpp>
#include <pid.hpp> // Include PID header

#include <deque>

vex::brain brain;
vex::controller controller;

vex::motor leftmotor1(vex::PORT18, vex::gearSetting::ratio6_1, true); // correct
vex::motor leftmotor2(vex::PORT20, vex::gearSetting::ratio6_1, true); // correct
vex::motor leftmotor3(vex::PORT17, vex::gearSetting::ratio6_1, false); // correct
vex::motor rightmotor1(vex::PORT2, vex::gearSetting::ratio6_1, false); // correct
vex::motor rightmotor2(vex::PORT4, vex::gearSetting::ratio6_1, false); // correct
vex::motor rightmotor3(vex::PORT3, vex::gearSetting::ratio6_1, true); // correct

void move_motor(vex::motor m, int pct) {
    m.spin(vex::fwd, 128 * pct, vex::voltageUnits::mV);
}

class HighStakesChassis: public Chassis {
    friend int main();
private:
    


public:
// old public
    struct Ring {
        HighStakesChassis* chassis;
        int  hooked_stage;
        bool color;          // 1 = red, 0 = blue
        bool initial_color;  // 1 = red, 0 = blue

        Ring(HighStakesChassis* new_chassis) {
            chassis      = new_chassis;
            hooked_stage = chassis->get_intake_stage();
            color        = chassis->team_color;

        }

        double get_pos() {
            // printf("Hooked stage: %d\n", hooked_stage);
            // printf("Intake pos: %.5f\n", chassis->get_intake_pos());
            // printf("Intake period: %.5f\n", chassis->intake_period);
            // printf("get_pos return: %.5f\n", chassis->get_intake_pos() - hooked_stage * chassis->intake_period);
            return chassis->get_intake_pos() - hooked_stage * chassis->intake_period;
        }
    };

    // High Stakes Specific Devices
    vex::motor* intake_motor;
    vex::motor* lb_motor1;
    vex::motor* lb_motor2;
    vex::pneumatics* base_pto;
    vex::pneumatics* intake_pto;
    vex::pneumatics* mogo_piston;
    vex::pneumatics* hang_piston;
    vex::pneumatics* ring_doinker; // Added
    vex::pneumatics* goal_doinker; // Added
    vex::pneumatics* intake_lift;
    vex::optical* intake_hook_color; // Added
    vex::optical* mogo_color; // Added
    vex::optical* first_stage_color; // Added
    vex::distance* lb_distance;
    vex::rotation* intake_rotation; // Added


    // Control state variables (previously globals)
    bool toggle_base_pto = false;
    bool toggle_mogo = false;
    bool toggle_hang_piston = false;
    bool toggle_ring_doinker = false; // Added state for new pneumatic
    bool toggle_goal_doinker = false; // Added state for new pneumatic
    bool toggle_intake_pto = false;
    bool toggle_intake_lift = false;
    double intake_power = 0.0;

    double lb_power = 0.0;
    bool lb_macro_mode = false;
    double lb_target_arm_height = 0.0;
    bool lb_has_ring = false;

    const double lb_load_height = 90.0; // Load height for LB
    const double lb_descore1_height = 470.0; // Descore height for LB
    const double lb_descore2_height = 520.0; // Score height for LB
    const double lb_store_height = 180.0; // Store height for LB

    bool lb_store_mode = false;

    int descore_state = 0; // For LB macro
    const double  intake_period = 19.0 * (360.0 / 6.0); // 19 chain links per hook //1140
    const bool team_color = true; // 1 = red, 0 = blue
    bool intake_ring_fire = false;
    std::deque<Ring> intake_ring_queue;

    // PID controller for Ladybrown
    PID lb_pid;

    // Helper function to get Ladybrown position (assuming lb_motor has encoder)
    double get_lb_pos() {
        return lb_motor1->position(vex::rotationUnits::deg); // Adjust units if necessary
    }

    // Helper function to get Intake Rotation position
    inline double get_intake_pos() {
        return intake_rotation->position(vex::rotationUnits::deg); // Adjust units if necessary
    }

    inline int get_intake_stage() {
        return floor(get_intake_pos() / intake_period);
    }

    inline double get_intake_hook_pos() {
        return get_intake_pos() - get_intake_stage() * intake_period;
    }

    bool detectRed(double hue) {
        if (hue <= 170 || hue >= 300) {
            return true;
        }

        return false;
    }

    bool is_lb_load_height() {
        printf("get_lb_pos(): %.5f\n", get_lb_pos());
        printf("lb_load_height: %.5f\n", lb_load_height);
        double lb_pos = get_lb_pos();
        if (lb_pos > lb_load_height - 20.0 && lb_pos < lb_load_height + 20.0) {
            printf("parameters %.5f, %.5f\n", lb_load_height - 20.0, lb_load_height + 20.0);
            return true;
        }
        return false;
    }

// old private
    HighStakesChassis(MotorGroup* left, MotorGroup* right, vex::rotation* forward_track, vex::rotation* side_track,
                      vex::inertial* imu, vex::controller* controller,
                      // High Stakes specific devices
                      vex::motor* intake_motor, vex::motor* lb_motor1, vex::motor* lb_motor2,
                      vex::pneumatics* base_pto, vex::pneumatics* intake_pto,
                      vex::pneumatics* mogo_piston, vex::pneumatics* hang_piston,
                      vex::pneumatics* ring_doinker, vex::pneumatics* goal_doinker, vex::pneumatics* intake_lift,
                      vex::optical* intake_hook_color, vex::optical* mogo_color, // Added
                      vex::optical* first_stage_color, vex::distance* lb_distance, vex::rotation* intake_rotation, // Added
                      // Base config
                      double base_width, double forward_offset, double side_offset, double wheel_radius,
                      double tracking_radius, double external_ratio, double max_radial_acceleration):
        Chassis(left, right, forward_track, side_track, imu, controller, base_width, forward_offset,
                side_offset, wheel_radius, tracking_radius, external_ratio, max_radial_acceleration),
        // Initialize High Stakes members
        intake_motor(intake_motor), lb_motor1(lb_motor1), lb_motor2(lb_motor2), base_pto(base_pto), intake_pto(intake_pto),
        mogo_piston(mogo_piston), hang_piston(hang_piston),
        ring_doinker(ring_doinker), goal_doinker(goal_doinker), intake_lift(intake_lift), // Added initialization
        intake_hook_color(intake_hook_color), mogo_color(mogo_color), // Added initialization
        first_stage_color(first_stage_color), lb_distance(lb_distance), intake_rotation(intake_rotation), // Added initialization
        // Initialize LB PID controller (adjust gains as needed from original definitions.hpp/pid.cpp if they existed)
        lb_pid(PID(1.0, 0.0, 3.5, 0.0, 5.0, 100.0, 0.0, 1000.0, 1000.0, 0.0, 50.0, 0.99)) // Placeholder gains
         {
        // Constructor implementation (if any more needed)
        lb_motor1->resetPosition(); // Reset LB position on init
        intake_rotation->resetPosition(); // Reset Intake rotation on init
    }

    Ring* first_stage_ring = nullptr; 

    // Friend declarations for thread functions
    friend int ladybrown_thread(void* o);
    friend int intake_helper_thread(void* o);
    friend int intake_thread(void* o);
    friend int mogo_thread(void* o);
    friend int pneumatics_thread(void* o); // Added for pneumatics control
    friend int highstakes_control(void* o);
};

int ladybrown_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::motor* lb_motor1_ptr = base->lb_motor1; // Get pointer for direct use
    vex::motor* lb_motor2_ptr = base->lb_motor2; // Get pointer for direct use
    bool prev_lb_has_ring = false;
    double prev_lb_pos = 0.0;

    while (true) {
        double current_lb_pos = base->get_lb_pos();
        if (current_lb_pos < 0) {
            lb_motor1_ptr->resetPosition();
        }

        if (base->get_lb_pos() > base->lb_store_height - 20 && prev_lb_pos <= base->lb_store_height - 20) {

            base->lb_store_mode = true;

        } else if (base->get_lb_pos() <= base->lb_store_height - 20 && prev_lb_pos > base->lb_store_height - 20) {

            base->lb_store_mode = false;
        }

        if (base->lb_has_ring && !prev_lb_has_ring) {
            printf("macro\n");
            base->lb_macro_mode = true;
            base->lb_target_arm_height = base->lb_store_height;
        }

        if (base->lb_has_ring && base->lb_distance->objectDistance(vex::distanceUnits::mm) > 150 || base->get_lb_pos() > base->lb_store_height - 20) {
            base->lb_has_ring = false;
        }

        if (!base->lb_macro_mode) {
            // Manual control
            // Apply slight holding power when near bottom and moving down/stopped
            if (base->lb_power <= 0 && current_lb_pos < 200) {
                move_motor(*lb_motor1_ptr, -10);
                move_motor(*lb_motor2_ptr, -10); 
            } else {
                move_motor(*lb_motor1_ptr, base->lb_power);
                move_motor(*lb_motor2_ptr, base->lb_power); 
            }
        } else {
            // Macro mode (PID control)
            base->lb_pid.target(base->lb_target_arm_height);
            // Use calculate_raw if the original PID class had it, otherwise use calculate
            double pid_output_power = base->lb_pid.calculate_raw(current_lb_pos, false);
            // Convert PID output (likely voltage) to percent if needed, or adjust PID gains for percent output
            move_motor(*lb_motor1_ptr, pid_output_power);
            move_motor(*lb_motor2_ptr, pid_output_power);
        }

        prev_lb_pos = base->get_lb_pos();
        prev_lb_has_ring = base->lb_has_ring;
        vex::this_thread::sleep_for(10); // Use std::chrono if preferred
    }
    return 0; // Should not be reached
}

int intake_helper_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::motor* intake_motor_ptr = base->intake_motor;
    vex::optical* intake_hook_color_ptr = base->intake_hook_color;
    vex::optical* first_stage_color_ptr = base->first_stage_color;

    bool prev_first_stage_color = false;

    while (true) {
        if (first_stage_color_ptr->isNearObject() && base->first_stage_ring == nullptr && prev_first_stage_color == false) {
            base->first_stage_ring = new HighStakesChassis::Ring(base);
            base->first_stage_ring->color = base->detectRed(first_stage_color_ptr->hue());
        }
        prev_first_stage_color = first_stage_color_ptr->isNearObject();

        // push rings into queue
        if (base->first_stage_ring != nullptr &&
            intake_hook_color_ptr->isNearObject() &&
            base->get_intake_hook_pos() > 450 && base->get_intake_hook_pos() < 800) {
            
            // new ring in the hook stage
            HighStakesChassis::Ring ring(base);
            // ring.color = base->first_stage_ring->color; // Pass color from first stage
            ring.color = base->detectRed(intake_hook_color_ptr->hue());
            printf("detected hue = %f\n", intake_hook_color_ptr->hue());
            ring.hooked_stage = base->get_intake_stage();
            base->intake_ring_queue.push_front(ring);

            printf("pushed ring\n");
            printf("color: %d\n", base->first_stage_ring->color);
            printf("hooked stage: %d\n", base->first_stage_ring->hooked_stage);
            printf("hooked pos: %f\n", base->first_stage_ring->get_pos());

            // remove the ring from the first stage intake
            delete base->first_stage_ring;
            base->first_stage_ring = nullptr;
        }

        // outtake 
        if (!base->intake_ring_queue.empty()) {
            // printf("Intake ring queue size: %d\n", base->intake_ring_queue.size());
            if (base->intake_ring_queue.front().get_pos() < 30) {
                base->intake_ring_queue.pop_front();
            }
            
            if (base->intake_ring_queue.back().get_pos() > (base->intake_period - 150)) {
                base->intake_ring_fire = true;
            }
        }

        // Debugging output
        brain.Screen.printAt(1, 20, "intake pos: %.2f, intake stage: %.2f", base->get_intake_pos(), base->get_intake_stage());
        brain.Screen.printAt(1, 40, "intake_hook_pos: %.2f", base->get_intake_hook_pos());
        brain.Screen.printAt(1, 60, "intake ring queue size: %d", base->intake_ring_queue.size());
        brain.Screen.printAt(1, 80, "first stage ring: %s", base->first_stage_ring);
        if (!base->intake_ring_queue.empty()) {
            brain.Screen.printAt(1, 100, "ring color: %d", base->intake_ring_queue.front().color);
            brain.Screen.printAt(1, 120, "back ring pos: %f", base->intake_ring_queue.back().get_pos());
        }

        vex::this_thread::sleep_for(10);
    }
    return 0; // Should not be reached
}

int intake_thread(void *o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::motor* intake_motor_ptr = static_cast<HighStakesChassis*>(o)->intake_motor;
    while (1) {
        if (!base->intake_ring_fire) {
            if (base->lb_store_mode && base->intake_power > 0 && !base->intake_ring_queue.empty() && base->intake_ring_queue.back().get_pos() < base->intake_period - 200) {
                move_motor(*intake_motor_ptr, 1);   
            }
            else if (base->lb_has_ring && base->get_lb_pos() < base->lb_store_height - 20) {
                move_motor(*intake_motor_ptr, 1);
            }
            else {
                move_motor(*intake_motor_ptr, base->intake_power);
            }
        }
        else {
            if (base->is_lb_load_height()) {
                printf("lb\n");
                printf("lb_has_ring: %d\n", base->lb_has_ring);
                while (base->intake_ring_queue.back().get_pos() < (base->intake_period) + 450) {
                    move_motor(*intake_motor_ptr, 100);
                    vex::this_thread::sleep_for(10);
                }

                move_motor(*intake_motor_ptr, 100);
                vex::this_thread::sleep_for(100);
                move_motor(*intake_motor_ptr, -100);
                vex::this_thread::sleep_for(50);
                move_motor(*intake_motor_ptr, 100);
                vex::this_thread::sleep_for(50);

                if (base->lb_distance->objectDistance(vex::distanceUnits::mm) < 150) {
                    printf("true\n");
                    base->lb_has_ring = true;
                    move_motor(*intake_motor_ptr, -10);
                    vex::this_thread::sleep_for(100);
                }
                printf("lb_has_ring: %d after\n", base->lb_has_ring);

                printf("done\n");
            }
            else if (base->intake_ring_queue.back().color != base->team_color) {
                while (base->intake_ring_queue.back().get_pos() < (base->intake_period) + 500) {
                    move_motor(*intake_motor_ptr, 100);
                    vex::this_thread::sleep_for(10);
                }

                move_motor(*intake_motor_ptr, -100);
                vex::this_thread::sleep_for(100);
                printf("throw\n");
            }
            else if (base->intake_ring_queue.back().color == base->team_color) {
                printf("score\n");
                // score sequence
                move_motor(*intake_motor_ptr, 100);
                vex::this_thread::sleep_for(450);
            }

            base->intake_ring_queue.pop_back();
            base->intake_ring_fire = false;
            
        }

        vex::this_thread::sleep_for(10);
    }
}

int mogo_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::pneumatics* mogo_piston_ptr = base->mogo_piston;
    vex::optical* mogo_color_ptr = base->mogo_color; // Added for mogo detection

    bool mogo_goal_detected = false;
    bool prev_mogo_goal_detected = false;
    while (1) {
        // print mogo sensor values
        if (mogo_color_ptr->isNearObject() && mogo_color_ptr->hue() >= 60 && mogo_color_ptr->hue() <= 80) {
            brain.Screen.printAt(1, 20, "mogo hue: %f", mogo_color_ptr->hue());
            mogo_goal_detected = true;
        } else {
            mogo_goal_detected = false;
        }

        if (!prev_mogo_goal_detected && mogo_goal_detected) {
            // If the goal was previously detected and is now not detected, close the mogo piston
            base->toggle_mogo = true;
        } 

        if (base->toggle_mogo) {
            mogo_piston_ptr->open();
        } else {
            mogo_piston_ptr->close();
        }

        prev_mogo_goal_detected = mogo_goal_detected;

        vex::this_thread::sleep_for(10);
    }
    return 0;
}

// Separate thread for pneumatics as they were mixed in original threads
int pneumatics_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    vex::pneumatics* base_pto_ptr = base->base_pto;
    vex::pneumatics* mogo_piston_ptr = base->mogo_piston;
    vex::pneumatics* hang_piston_ptr = base->hang_piston;
    vex::pneumatics* ring_doinker_ptr = base->ring_doinker; // Added
    vex::pneumatics* goal_doinker_ptr = base->goal_doinker; // Added
    vex::pneumatics* intake_lift_ptr = base->intake_lift; // Added
    vex::pneumatics* intake_pto_ptr = base->intake_pto; // Added
    vex::optical* mogo_color_ptr = base->mogo_color; // Added for potential auto-clamp
    // vex::pneumatics* intake_pto_ptr = base->intake_pto; // Control this if needed

    while (true) {
        if (base->toggle_base_pto) {
            base_pto_ptr->close();
        } else {
            base_pto_ptr->open();
        }

        if (base->toggle_intake_pto) {
            intake_pto_ptr->open(); // Uncomment if needed
        } else {
            intake_pto_ptr->close(); // Uncomment if needed
        }

        if (base->toggle_intake_lift) {
            intake_lift_ptr->open(); // Uncomment if needed
        } else {
            intake_lift_ptr->close(); // Uncomment if needed
        }

        // Add mogo color sensor logic here if needed to auto-trigger mogo piston
        // Example: if (mogo_color_ptr->isNearObject() && mogo_color_ptr->color() == vex::color::red /* or desired color */) { base->toggle_mogo = true; }

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
    vex::optical* intake_hook_color_ptr = base->intake_hook_color; // Use getter method
    vex::optical* mogo_color_ptr = base->mogo_color; // Use getter method
    vex::optical* first_stage_color_ptr = base->first_stage_color; // Use getter method
    vex::rotation* intake_rotation_ptr = base->intake_rotation; // Use getter method

    // initialize
    intake_hook_color_ptr->integrationTime(70);
    intake_hook_color_ptr->setLightPower(100);
    mogo_color_ptr->setLightPower(100);
    first_stage_color_ptr->setLightPower(100);
    intake_rotation_ptr->resetPosition();

    // Previous button states for toggle logic
    bool prev_mogo_state = false;
    bool prev_base_pto_state = false;
    bool prev_lb_raise_state = false;
    bool prev_lb_lower_state = false;
    bool prev_climb_state = false;
    bool prev_ring_doinker_state = false; // Added
    bool prev_goal_doinker_state = false; // Added
    bool prev_intake_lift_state = false; // Added

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
    #define RING_DOINKER_BIND  controller_ptr->ButtonY.pressing() // Example mapping - Added
    #define GOAL_DOINKER_BIND  controller_ptr->ButtonRight.pressing() // Example mapping - Added
    #define INTAKE_LIFT_BIND    controller_ptr->ButtonX.pressing() // Example mapping - Added


    while (true) {
        double linear_power = DRIVE_AXIS_FORWARD;
        double turn_power = DRIVE_AXIS_TURN;
        // Convert percent to voltage for steer method
        double left_voltage = (linear_power + turn_power) * 12.8;
        double right_voltage = (linear_power - turn_power) * 12.8;

        move_motor(leftmotor1, left_voltage);
        move_motor(leftmotor2, left_voltage);
        move_motor(leftmotor3, left_voltage);
        move_motor(rightmotor1, right_voltage);
        move_motor(rightmotor2, right_voltage);
        move_motor(rightmotor3, right_voltage);

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

        bool current_intake_lift_bind = INTAKE_LIFT_BIND;
        if (current_intake_lift_bind != prev_intake_lift_state && !prev_intake_lift_state){ // Rising edge
            base->toggle_intake_lift = !base->toggle_intake_lift;
        }
        prev_intake_lift_state = current_intake_lift_bind;

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
        } else if (LB_LOWER_BIND) {
            base->lb_power = -100;
        } else { 
             base->lb_power = 0;
        }


        // Advanced ladybrown macro controls (Triggered by pressing both buttons)
        bool current_lb_raise = LB_RAISE_BIND;
        bool current_lb_lower = LB_LOWER_BIND;

        // Raise + Lower pressed simultaneously (rising edge detection for raise)
        if (current_lb_raise != prev_lb_raise_state && !prev_lb_raise_state) { // Rising edge of Raise
            if (current_lb_lower) { // Check if Lower is also pressed
                base->lb_macro_mode = true;
                base->lb_target_arm_height = base->lb_load_height; // Target low height (adjust as needed)
                // printf("Macro: Target Low\\n");
            } else {
                base->lb_macro_mode = false;
            }
        }

        // Lower + Raise pressed simultaneously (rising edge detection for lower)
        if (current_lb_lower != prev_lb_lower_state && !prev_lb_lower_state) { // Rising edge of Lower
            if (current_lb_raise) { // Check if Raise is also pressed
                base->lb_macro_mode = true;
                base->descore_state++;
                printf("descore state: %d\n", base->descore_state);
                if (base->descore_state == 1) {
                    base->lb_target_arm_height = base->lb_descore1_height; // First descore height
                } else { // >= 2
                    base->lb_target_arm_height = base->lb_descore2_height; // Second descore height (lower)
                }
            } else {
                printf("bruh\n");
                base->descore_state = 0;
                base->lb_macro_mode = false; 
            }
        }

        // Update previous button states for LB macros
        prev_lb_raise_state = current_lb_raise;
        prev_lb_lower_state = current_lb_lower;

        vex::this_thread::sleep_for(10);
    }
    return 0; // Should not be reached
}


#endif // #ifndef HIGHSTAKES_HPP