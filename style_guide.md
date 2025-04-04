Based on the conversation, I can recommend a comprehensive naming convention for your project that combines both team members' preferences while maintaining clarity and consistency:

# Recommended Naming Convention

## File Structure
- Use definitions.h for global declarations as Aaron suggested
- Group subsystem-related code and values together (e.g., Intake_Power)

## Variables
- *Local variables*: snake_case (lowercase with underscores)
  cpp
  int motor_speed = 100;
  float sensor_value = 0.5;
  

- *Global variables*: g_snake_case (prefix with g_ to distinguish)
  cpp
  int g_max_speed = 200;
  bool g_is_initialized = false;
  

- *Constants*: UPPER_SNAKE_CASE
  cpp
  const int MAX_VOLTAGE = 12;
  #define TIMEOUT_MS 500
  

- *Subsystem-specific globals*: SubsystemName_snake_case
  cpp
  int Intake_power = 80;
  bool Shooter_is_ready = true;
  

## Functions
- Use snake_case for function names
  cpp
  void update_motor_values() { }
  bool is_system_ready() { }
  

## Classes
- Use PascalCase (CamelCase with first letter capitalized)
  cpp
  class MotorController { };
  class RobotDrive { };
  

## Lambda Expressions
- For simple lambdas:
  cpp
  auto compare_values = [](int a, int b) { return a < b; };
  

- For lambdas capturing variables:
  cpp
  auto calculate = [current_value](int input) { return current_value + input; };
  

## General Guidelines
- Avoid abbreviations except for well-known ones
- Use descriptive names that explain purpose
- Be consistent with the style across the project
- Use comments for complex code sections

This convention combines both your preferences while adding clear distinctions for global variables to prevent scope confusion.