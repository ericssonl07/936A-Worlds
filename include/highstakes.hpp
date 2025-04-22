#ifndef HIGHSTAKES_HPP
#define HIGHSTAKES_HPP

#include <vex.h>
#include <chassis.hpp>

class HighStakesChassis: public Chassis {
private:
    // Device declarations here
    MotorGroup* left;
    MotorGroup* right;
    vex::rotation* forward_track;
    vex::rotation* side_track;
    vex::inertial* imu;
    vex::controller* controller;
    // AARON YOU: All other High Stakes-specific devices here- lady brown, intake, pneumatics, etc.
    double base_width;
    double forward_offset;
    // ... etc.
public:
    HighStakesChassis(MotorGroup* left, MotorGroup* right, vex::rotation* forward_track, vex::rotation* side_track,
                      vex::inertial* imu, vex::controller* controller,
                      // AARON YOU: All other High Stakes-specific devices here- lady brown, intake, pneumatics, etc.
                      double base_width, double forward_offset, double side_offset, double wheel_radius,
                      double tracking_radius, double external_ratio, double max_radial_acceleration):
        Chassis(left, right, forward_track, side_track, imu, controller, base_width, forward_offset,
                side_offset, wheel_radius, tracking_radius, external_ratio, max_radial_acceleration) {
        // Constructor implementation
    }
    friend int ladybrown_thread(void* o);
    friend int intake_thread(void* o);
    // ... etc.
    friend int highstakes_control(void* o);
};

int ladybrown_thread(void* o) {
    // Always cast the base first
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    // Optionally, get device references (if not, you will have to use base -> left -> method everywhere which might be cluttered)
    MotorGroup left = *base -> left;
    // ... etc.

    // Example usage
    left.spin(12.0, vex::voltageUnits::volt); // If casted
    base -> left -> spin(12.0, vex::voltageUnits::volt); // If not casted

    // AARON YOU: Lady Brown control code here
}

int intake_thread(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    // AARON YOU: Intake control code here
}

int highstakes_control(void* o) {
    HighStakesChassis* base = static_cast<HighStakesChassis*>(o);
    // AARON YOU: Control code here
}

#endif // #ifndef HIGHSTAKES_HPP