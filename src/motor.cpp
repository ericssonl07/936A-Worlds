#include <motor.hpp>
#include <iostream>
#include <iomanip>

double Motor::gear_ratio(vex::gearSetting gears) {
    switch (gears) {
        case vex::gearSetting::ratio6_1:
            return 1 / 6.0;
        case vex::gearSetting::ratio18_1:
            return 1 / 18.0;
        case vex::gearSetting::ratio36_1:
            return 1 / 36.0;
    }
}

void Motor::spin_to_impl(void* instance) {
    Motor* motor = (Motor*) instance;
    double error0 = motor -> spin_to_impl_position - motor -> device_position(vex::rotationUnits::deg);
    PID controller(0.06, 0.0066, 0.06,                                      // gains
                   motor -> spin_to_impl_position,                          // target position
                   motor -> spin_to_impl_tolerance,                         // tolerance: allowed error
                   motor -> spin_to_impl_maximum,                           // max_value
                   motor -> spin_to_impl_minimum,                           // min_value
                   fabs(error0 * motor -> spin_to_impl_activation_ratio),   // activation_threshold
                   fabs(error0 * motor -> spin_to_impl_integral_ratio),     // integration_threshold
                   0.05,                                                    // derivative_threshold
                   50.0,                                                    // max_integral
                   0.999);                                                  // gamma
    double pos, output;
    // std::cout << std::scientific << std::setprecision(5);
    // std::cout << "x\t\tP\t\tI\t\tD\t\tOut\n--------------------------------------------------------------------------------\n";
    while (!controller.arrived()) {
        pos = motor -> device_position(vex::rotationUnits::deg);
        output = controller.calculate(pos);
        motor -> spin(output, vex::voltageUnits::volt);
        // printf("(%d, %.5f)\n", iter++, pos);
        // std::cout << pos << "\t" << controller.get_info()["P"] << "\t" << controller.get_info()["I"] << "\t" << controller.get_info()["D"] << "\t" << output << "\n";
        vex::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    motor -> stop();
}

Motor::Motor(vex::motor* device, double external_ratio, double wheel_radius):
        device(device), external_ratio(external_ratio), wheel_radius(wheel_radius) {
    auto cartridge = device -> getMotorCartridge();
    cartridge_ratio = gear_ratio(cartridge);
}

Motor::Motor(int32_t index, vex::gearSetting gears, bool reverse, double external_ratio, double wheel_radius):
        external_ratio(external_ratio), wheel_radius(wheel_radius) {
    device = new vex::motor(index, gears, reverse);
    cartridge_ratio = gear_ratio(gears);
}

Motor::Motor(const Motor& other):
        device(other.device), cartridge_ratio(other.cartridge_ratio),
        external_ratio(other.external_ratio), wheel_radius(other.wheel_radius) {}

Motor::Motor(Motor&& other):
        device(std::move(other.device)), cartridge_ratio(std::move(other.cartridge_ratio)),
        external_ratio(std::move(other.external_ratio)), wheel_radius(std::move(other.wheel_radius)) {}

Motor& Motor::operator=(const Motor& other) {
    device = other.device;
    cartridge_ratio = other.cartridge_ratio;
    external_ratio = other.external_ratio;
    wheel_radius = other.wheel_radius;
    return *this;
}

Motor& Motor::operator=(Motor&& other) {
    device = std::move(other.device);
    cartridge_ratio = std::move(other.cartridge_ratio);
    external_ratio = std::move(other.external_ratio);
    wheel_radius = std::move(other.wheel_radius);
    return *this;
}

Motor::~Motor() {
    delete device;
}

void Motor::spin(double velocity) {
    double rps_wheel = velocity / (wheel_radius * M_PI * 2);
    double rpm_wheel = rps_wheel * 60;
    double rpm_motor = rpm_wheel / (cartridge_ratio * external_ratio);
    double ratio = rpm_motor / max_motor_rpm;
    double voltage = ratio * max_voltage;
    if (voltage > max_voltage) voltage = max_voltage;
    if (voltage < -max_voltage) voltage = -max_voltage;
    device -> spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
}

void Motor::spin(double power, vex::percentUnits units) {
    double ratio = power / 100;
    double voltage = ratio * max_voltage;
    device -> spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
}

void Motor::spin(double voltage, vex::voltageUnits units) {
    device -> spin(vex::directionType::fwd, voltage, units);
}

void Motor::reset_position() {
    device -> resetPosition();
}

void Motor::stop() {
    device -> stop(vex::brakeType::brake);
}

bool Motor::connected() {
    return device -> installed();
}

double Motor::device_position(vex::rotationUnits units) const {
    return device -> position(units);
}

double Motor::wheel_position(vex::rotationUnits units) const {
    return device_position(units) * external_ratio;
}

double Motor::max_wheel_velocity() const {
    return max_motor_rpm * cartridge_ratio * external_ratio * wheel_radius * M_PI * 2;
}

void Motor::spin_to(double position, double tolerance, double maximum, double minimum, double activation_ratio, double integral_ratio, bool block) {
    spin_to_impl_position = position;
    spin_to_impl_tolerance = tolerance;
    spin_to_impl_maximum = maximum;
    spin_to_impl_minimum = minimum;
    spin_to_impl_activation_ratio = activation_ratio;
    spin_to_impl_integral_ratio = integral_ratio;
    if (block) {
        spin_to_impl(this);
    } else {
        vex::thread execute(&spin_to_impl, this);
        execute.detach();
    }
}