#include "main.h"


// ===== CHASSIS CONFIGURATION =====
// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-4, -5, -6}, // Left Chassis Ports (negative port will reverse it!) - SWAPPED
    {1, 2, 3},    // Right Chassis Ports (negative port will reverse it!) - SWAPPED

    8,      // IMU Port
    4.125,  // Wheel diameter
    450     // Cartridge RPM
);

// ===== SUBSYSTEMS =====
// 2 motor intake group spinning same direction
Intake intake({11, 12});

// ===== ODOMETRY TRACKING WHEELS =====
// - `9` and `10` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(9, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels (horizontal)
ez::tracking_wheel vert_tracker(10, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels (vertical)

// ===== INITIALIZATION =====
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Initialize intake motors
  intake.init();

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  chassis.odom_tracker_left_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(false); // Disable curve buttons since we're using custom control
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"DRIVE FORWARD TEST\n\nDrive forward 24 inches", drive_test},
      {"TURN TEST\n\nTurn 90 degrees", turn_test},
      {"INTAKE TEST\n\nRun intake for 3 seconds", intake_test},
      {"ODOM TEST\n\nTest odometry movement", odom_test},
      {"PID TEST - DRIVE 24 INCHES", pid_tuning_test}
  });

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

// ===== COMPETITION FUNCTIONS =====
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  intake.set_state_and_move(Intake::State::NONE);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display debug information
          ez::screen_print(
            "comp: " + std::to_string(pros::competition::is_connected()) + "\n" +
            "X: " + std::to_string(chassis.odom_x_get()) + "\n" +
            "Y: " + std::to_string(chassis.odom_y_get()) + "\n" +
            "Theta: " + std::to_string(chassis.odom_theta_get()) + "\n" +
            "Intake Temp: " + std::to_string(intake.intake_motors.get_temperature_all()[0]) + "C",
            1
          );
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {

    if (master.get_digital_new_press(DIGITAL_DOWN) && master.get_digital(DIGITAL_B))
      autonomous();
    
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

// ===== OPERATOR CONTROL =====
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  intake.set_state_and_move(Intake::State::NONE);

  while (true) {
    ez_template_extras();

    chassis.opcontrol_arcade_standard(ez::SPLIT);
    intake.opcontrol(master);

    // Pneumatic controls
    intake_lift.button_toggle(master.get_digital(DIGITAL_UP));    // Port E - Up arrow
    
    // Indexer: L1 extends, L2 retracts
    if (master.get_digital(DIGITAL_L1)) {
      indexer.set(true);   // Extend
    } else if (master.get_digital(DIGITAL_L2)) {
      indexer.set(false);  // Retract
    }
    
    wing.button_toggle(master.get_digital(DIGITAL_A));            // Port G - Button A
    little_will_mech.button_toggle(master.get_digital(DIGITAL_Y)); // Port H - Button Y
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
