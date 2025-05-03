#include <odometry.hpp>
#include <iostream>

/*
class Odometry {
    friend int track(void* o);
    friend int display(void* o);
    vex::thread tracking_thread;
    vex::thread displaying_thread;
    vex::rotation* forward_track;
    vex::rotation* left_track;
    double left_right_offset; // left is positive
    double forward_back_offset; // back is positive
    double tracking_radius;
    double x_position;
    double y_position;
    double rotation_value;
    double get_forward();
    double get_back();
public:
    Odometry(vex::rotation* forward_track, vex::rotation* left_track,
             double base_width, double forward_back_offset, double wheel_radius, double tracking_radius);
    double x();
    double y();
    double rotation();
    void reset();
    void set_pose(double x, double y, double rotation);
}
*/

int track(void* o) {
    Odometry* odometry = static_cast<Odometry*>(o);
    odometry -> reset();
    double last_left = 0.0, last_forward = 0.0, last_rotation = 0.0;
    while (true) {
        double forward = odometry -> get_forward();
        double back = odometry -> get_back();
        double rotation = odometry -> imu -> rotation(vex::rotationUnits::deg) * M_PI / 180.0;
        double d_forward = forward - last_forward;
        double d_back = back - last_left;
        double d_theta = rotation - last_rotation;
        last_forward = forward;
        last_left = back;
        last_rotation = rotation;
        double dx_local, dy_local;
        if (d_theta == 0) {
            dx_local = d_back;
            dy_local = d_forward;
        } else {
            double r_c = d_forward / d_theta - odometry -> left_right_offset;
            double r_cb = d_back / d_theta - odometry -> forward_back_offset;
            // printf("r_c=d_forward/d_theta-lr_offset=%.5f/\t%.5f-\t%.5f=\t%.5f\n", d_forward, d_theta, odometry -> left_right_offset, r_c);
            // printf("r_cb=d_back/d_theta-fb_offset=%.5f/\t%.5f-\t%.5f=\t%.5f\n", d_back, d_theta, odometry -> forward_back_offset, r_cb);
            double half_sin = sin(d_theta * 0.5);
            dx_local = 2.0 * r_cb * half_sin;
            dy_local = 2.0 * r_c * half_sin;
            // printf("(%.5f, %.5f)\n", odometry->x_position, odometry->y_position);

            /*
            Pure turn analysis:
                dx_local = 2 * r_cb * sin(d_theta / 2)
                    = 2 * (d_back / d_theta - fb_offset) * sin(d_theta / 2)
                    = 2 * (d_theta * fb_offset / d_theta - fb_offset) * sin(d_theta / 2)
                    = 2 * (fb_offset - fb_offset) * sin(d_theta / 2)
                    = 0

                dy_local = 2 * r_c * sin(d_theta / 2)
                    = 2 * (d_forward / d_theta - lr_offset) * sin(d_theta / 2)
                    = 2 * (d_theta * lr_offset / d_theta - lr_offset) * sin(d_theta / 2)
                    = 2 * (lr_offset - lr_offset) * sin(d_theta / 2)
                    = 0
            */

            /*
            Small angle analysis (sine <<< 1):

                dx_local = 2 * r_cb * d_theta / 2 = r_cb * d_theta
                    = (d_back / d_theta - fb_offset) * d_theta
                    = d_back - fb_offset * d_theta

                dy_local = 2 * r_c * d_theta / 2 = r_c * d_theta
                    = (d_forward / d_theta - lr_offset) * d_theta
                    = d_forward - lr_offset * d_theta
            
                Case 1: near-stationary
                    However, if d_back and d_forward are small, and d_theta is small, then dx_local and dy_local
                    are also small.

                Case 2: translation
                    If d_theta is small while d_back and d_forward are large, then dx_local and dy_local
                    are approximately equal to d_back and d_forward, respectively.

                Special case: zero offset
                    If the offset is zero, then dx_local and dy_local are equal to d_back and d_forward,
                    respectively, regardless of the value of d_theta.
            */
        }
        double theta_mid = odometry -> rotation_value + d_theta * 0.5;
        double sine = sin(theta_mid);
        double cosine = cos(theta_mid);
        double dx_global = dx_local * sine + dy_local * cosine;
        double dy_global = -dx_local * cosine + dy_local * sine;

        odometry -> x_position += dx_global;
        odometry -> y_position += dy_global;
        odometry -> rotation_value += d_theta;
        vex::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int display(void* o) {
    // Odometry* odometry = static_cast<Odometry*>(o);
    // while (true) {
    //     std::cout << "(" << odometry -> get_forward() << ", " << odometry -> get_back() << ", "
    //               << odometry -> imu -> rotation(vex::rotationUnits::deg) << ")" << std::endl;
    //     printf("(%.5f, %.5f, %.5f)\n", odometry -> x(), odometry -> y(), odometry -> rotation() / M_PI * 180.0);
    // }
    return 0;
}

double Odometry::get_forward() {
    return forward_track -> position(vex::rotationUnits::deg) * M_PI / 180.0 * tracking_radius;
}

double Odometry::get_back() {
    return side_track -> position(vex::rotationUnits::deg) * M_PI / 180.0 * tracking_radius;
}

Odometry::Odometry(vex::rotation* forward_track, vex::rotation* side_track, vex::inertial* imu,
             double left_right_offset, double forward_back_offset, double tracking_radius)
    : forward_track(forward_track), side_track(side_track), imu(imu),
      left_right_offset(left_right_offset), forward_back_offset(forward_back_offset),
      tracking_radius(tracking_radius) {
    reset();
    tracking_thread = vex::thread(track, this);
    displaying_thread = vex::thread(display, this);
}

double Odometry::x() {
    return x_position;
}

double Odometry::y() {
    return y_position;
}

double Odometry::rotation() {
    return rotation_value;
}

void Odometry::reset() {
    x_position = 0.0;
    y_position = 0.0;
    side_track -> resetPosition();
    forward_track -> resetPosition();
    imu -> resetRotation();
    rotation_value = 0.0;
}

void Odometry::set_pose(double x, double y, double rotation) {
    x_position = x;
    y_position = y;
    rotation_value = rotation;
}

// int track(void* o) {
//     // Cast to your odometry structure type.
//     Odometry* odometry = (Odometry*) o;
//     // Reset tracking wheel sensors.
//     odometry->left->reset();
//     odometry->back->resetPosition();

//     // Set a threshold for what we consider a "small" d_theta.
//     // You can adjust this constant based on your numerical precision and motion profile.
//     const double small_angle_threshold = 1e-6;

//     double last_left = 0.0, last_right = 0.0, last_back = 0.0;
//     while (true) {
//         // Read the current sensor values.
//         double left = odometry -> get_left();
//         double right = odometry -> get_right();
//         double back = odometry -> get_back();

//         // Compute differences from the last sensor readings.
//         double d_left  = left  - last_left;
//         double d_right = right - last_right;
//         double d_back  = back  - last_back;

//         // Store current readings as the last readings for the next update.
//         last_left = left;
//         last_right = right;
//         last_back = back;

//         // Calculate the change in orientation.
//         // Note: This is based on your convention: positive d_left and negative d_right for turning.
//         double d_theta = (d_left - d_right) / odometry->base_width;

//         // Declare variables for the local x and y displacement.
//         double dx_local = 0.0, dy_local = 0.0;

//         // Avoid division by zero.
//         if (d_theta == 0) {
//             dx_local = d_back;  // Lateral movement from the back tracking wheel.
//             dy_local = d_right; // Forward movement given by the side wheels.
//         } else {
//             // Use the circular arc model.
//             // Compute the "effective radii" for the side wheel (r_c) and back wheel (r_cb).
//             // Tracking the geometric center of the robot so offsets.
//             double r_c  = d_left / d_theta - odometry -> base_width * 0.5;
//             double r_cb = d_back / d_theta - odometry -> back_offset;
//             // The factor 2*sin(d_theta/2) converts the radius to a chord length.
//             double half_sin = sin(d_theta * 0.5);
//             dx_local = 2.0 * r_cb * half_sin;
//             dy_local = 2.0 * r_c  * half_sin;
//         }

//         // Compute the intermediate heading for the update (using the midpoint integration method).
//         double theta_mid = odometry -> rotation_value + d_theta * 0.5;
//         // Transform the local displacements into global coordinates using the standard rotation matrix.
//         double dx_global = dx_local * cos(theta_mid) - dy_local * sin(theta_mid);
//         double dy_global = dx_local * sin(theta_mid) + dy_local * cos(theta_mid);

//         // Update the global odometry.
//         odometry -> x_position      += dx_global;
//         odometry -> y_position      += dy_global;
//         odometry -> rotation_value  += d_theta;

//         // Sleep to yield processor time (and to set your odometry update rate).
//         vex::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }
// }

// int display(void* o) {
//     // Odometry* odometry = (Odometry*) o;
//     // while (true) {
//     //     printf("Left: %.5f, Right: %.5f\n", odometry -> get_left(), odometry -> get_right());
//     //     printf("(%.5f, %.5f, %.5f)\n", odometry -> x_position, odometry -> y_position, odometry -> rotation_value);
//     //     vex::this_thread::sleep_for(std::chrono::milliseconds(100));
//     // }
//     return 0;
// }

// double Odometry::get_left() {
//     return left -> wheel_position(vex::rotationUnits::deg) * M_PI / 180.0 * main_radius;
// }

// double Odometry::get_right() {
//     return right -> wheel_position(vex::rotationUnits::deg) * M_PI / 180.0 * main_radius;
// }

// double Odometry::get_back() {
//     return back -> position(vex::rotationUnits::deg) * M_PI / 180.0 * drift_radius;
// }

// Odometry::Odometry(MotorGroup* left, MotorGroup* right, vex::rotation* back, double base_width, double back_offset, double main_radius, double drift_radius)
//     : left(left), right(right), back(back), base_width(base_width), back_offset(back_offset), main_radius(main_radius), drift_radius(drift_radius) {
//     reset();
//     tracking_thread = vex::thread(track, this);
// }

// double Odometry::x() {
//     return Odometry::x_position;
// }

// double Odometry::y() {
//     return Odometry::y_position;
// }

// double Odometry::rotation() {
//     return Odometry::rotation_value;
// }

// void Odometry::reset() {
//     x_position = 0.0;
//     y_position = 0.0;
//     rotation_value = 0.0;
// }

// void Odometry::set_pose(double x, double y, double rotation) {
//     Odometry::x_position = x;
//     Odometry::y_position = y;
//     Odometry::rotation_value = rotation;
// }