#include "main.h"
#include "autons.hpp"

// ===== AUTONOMOUS CONSTANTS =====
// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

// ===== PID CONSTANTS =====
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(5.0, 0.0, 65.0);         // Fwd/rev constants
  chassis.pid_heading_constants_set(0.0, 0.0, 0.0);        // DISABLED heading correction to prevent spinning
  chassis.pid_turn_constants_set(2.5, 0.0, 25.0, 15);      // Turn in place constants
  chassis.pid_swing_constants_set(5.0, 0.0, 30.0);         // Swing constants
  chassis.pid_odom_angular_constants_set(6.0, 0.0, 50.0);  // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.0, 0.0, 45.0); // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(70_ms, 3_deg, 200_ms, 6_deg, 300_ms, 200_ms);
  chassis.pid_swing_exit_condition_set(70_ms, 3_deg, 200_ms, 6_deg, 300_ms, 200_ms);
  chassis.pid_drive_exit_condition_set(70_ms, 1_in, 200_ms, 3_in, 300_ms, 200_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 300_ms, 200_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 300_ms, 200_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 100);
  chassis.slew_drive_constants_set(1_in, 100);
  chassis.slew_swing_constants_set(3_in, 100);

  // The amount that turns are prioritized over driving in odom motions
  // - with tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.95);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// ===== AUTONOMOUS ROUTINES =====
void pid_tuning_test() {
  // Drive 48 inches (4 feet) forward for PID tuning
  chassis.pid_drive_set(48_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void drive_test() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
}

void turn_test() {
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
}

void intake_test() {
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(3000);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(3000);
  intake.set_state_and_move(Intake::State::NONE);
}

void odom_test() {
  // Reset odometry position
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  // Simple odometry test - drive forward and display position
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  pros::delay(1000);
  
  // Turn 90 degrees and drive again
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  pros::delay(1000);
  
  // Return to start
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

void leftAuton() {
  wing.set(true);
  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_drive_set(30_in, (DRIVE_SPEED));
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-136_deg, (TURN_SPEED));
  chassis.pid_wait();
  
  wing.set(false);
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(300);
  wing.set(true);
  intake.set_state_and_move(Intake::State::MIDDLE);
  pros::delay(1100);
  intake.set_state_and_move(Intake::State::NONE);
  chassis.pid_drive_set(48_in, (DRIVE_SPEED));
  chassis.pid_wait();

  lil_krith.set(true);

  chassis.pid_turn_relative_set(-45_deg, (TURN_SPEED));
  chassis.pid_wait();

  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_drive_set(22_in, (DRIVE_SPEED));
  chassis.pid_wait();
  pros::delay(300);
  chassis.pid_drive_set(5_in, (DRIVE_SPEED));
  pros::delay(100);
 
  chassis.pid_drive_set(-32_in, (DRIVE_SPEED));
  chassis.pid_wait();



  wing.set(false);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(200);
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(2000);
  intake.set_state_and_move(Intake::State::NONE);
  chassis.pid_drive_set(17_in, (DRIVE_SPEED));
  chassis.pid_wait();
  wing.set(true);
  chassis.pid_drive_set(-50_in, (DRIVE_SPEED));
  chassis.pid_wait();


}

void rightAuton() {
  wing.set(true);
  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_drive_set(28_in, (DRIVE_SPEED));
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-65_deg, (TURN_SPEED));
  chassis.pid_wait();

  pros::Task([] {
    pros::delay(300);
    intake.set_state_and_move(Intake::State::OUTTAKING);
  });
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1500);

  intake.set_state_and_move(Intake::State::NONE);
  chassis.pid_drive_set(-42_in, (DRIVE_SPEED));
  chassis.pid_wait();

  lil_krith.set(true);

  chassis.pid_turn_relative_set(-138_deg, (TURN_SPEED));
  chassis.pid_wait();

  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_drive_set(22_in, (DRIVE_SPEED));
  chassis.pid_wait();
  pros::delay(100);
 
  chassis.pid_drive_set(-32_in, (DRIVE_SPEED));
  chassis.pid_wait();



  wing.set(false);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(200);
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(2000);
  intake.set_state_and_move(Intake::State::NONE);
  chassis.pid_drive_set(17_in, (DRIVE_SPEED));
  chassis.pid_wait();
  wing.set(true);
  chassis.pid_drive_set(-50_in, (DRIVE_SPEED));
  chassis.pid_wait();
}

