#include "main.h"
#include "autons.hpp"

// ===== AUTONOMOUS CONSTANTS =====
// These are out of 127
const int DRIVE_SPEED = 110;  // Reduced from 127 to prevent wheel slip
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

// ===== PID CONSTANTS =====
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(5.0, 0.0, 63.0);         // Fwd/rev constants
  chassis.pid_heading_constants_set(3.0, 0.0, 20.0);       // ENABLED heading correction - CRITICAL FIX
  chassis.pid_turn_constants_set(2.5, 0.0, 25.0, 15);      // Turn in place constants
  chassis.pid_swing_constants_set(5.0, 0.0, 30.0);         // Swing constants
  chassis.pid_odom_angular_constants_set(6.0, 0.0, 50.0);  // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.0, 0.0, 45.0); // Angular control for boomerang motions

  // Exit conditions - slightly more lenient for consistency
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(100_ms, 3_deg, 300_ms, 7_deg, 600_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(100_ms, 1_in, 300_ms, 3_in, 600_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);  // Increased from 50
  chassis.slew_swing_constants_set(3_in, 60);

  // The amount that turns are prioritized over driving in odom motions
  // - with tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.85);  // Slightly reduced for smoother motions

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(14_in);  // Reduced from 16_in for tighter control
  chassis.odom_boomerang_dlead_set(0.55);      // Reduced from 0.625 for less aggressive end

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// ===== AUTONOMOUS ROUTINES =====

void leftOdomAuton() {
  // Set starting position: -1500mm X, 0mm Y, 45° heading
  chassis.odom_xyt_set(-1500_mm, 0_mm, 45_deg);
  
  // 1. Drive to (-600, 600) while intaking
  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_odom_set({{{-600_mm, 600_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  pros::delay(100);  // Small delay for mechanism stability
  
  // 2. Turn 90° clockwise (from 45° to 135°)
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  
  // 3. Move forward to (-200, 200)
  chassis.pid_odom_set({{{-200_mm, 200_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 4. Reverse intake for 500ms
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(500);
  
  // 5. Go reverse to (-1200, 1200)
  chassis.pid_odom_set({{{-1200_mm, 1200_mm}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  pros::delay(100);
  
  // 6. Turn 135° clockwise (from 135° to 270°)
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  
  // 7. Go to (-1700, 1200)
  chassis.pid_odom_set({{{-1700_mm, 1200_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 8. Activate lil_krith and intake for 400ms
  lil_krith.set(true);
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(400);
  intake.set_state_and_move(Intake::State::NONE);
  pros::delay(100);
  
  // 9. Go reverse to (-600, 1200)
  chassis.pid_odom_set({{{-600_mm, 1200_mm}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 10. Activate intake_lift and outtake for 1500ms
  intake_lift.set(true);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(1500);
  
  // 11. Go forward 300mm to (-900, 1200)
  chassis.pid_odom_set({{{-900_mm, 1200_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  pros::delay(100);
  
  // 12. Turn 150° clockwise (from 270° to 60°)
  chassis.pid_turn_set(60_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  
  // 13. Move forward 300mm at 60° angle
  // From (-900, 1200), moving 300mm at 60° = (-900 + 300*cos(60°), 1200 + 300*sin(60°))
  // = (-900 + 150, 1200 + 260) = (-750, 1460)
  chassis.pid_odom_set({{{-750_mm, 1460_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 14. Activate wing and move forward 450mm
  // From (-750, 1460), moving 450mm at 60° = (-750 + 450*cos(60°), 1460 + 450*sin(60°))
  // = (-750 + 225, 1460 + 390) = (-525, 1850)
  wing.set(true);
  pros::delay(100);  // Let wing extend
  chassis.pid_odom_set({{{-525_mm, 1850_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  intake.set_state_and_move(Intake::State::NONE);
}

void rightOdomAuton() {
  // RIGHT SIDE - Mirror over Y-axis (flip X coordinates, keep Y the same)
  // Set starting position: 1500mm X (flipped from -1500), 0mm Y, -45° heading (flipped from 45°)
  chassis.odom_xyt_set(1500_mm, 0_mm, -45_deg);
  
  // 1. Drive to (600, 600) while intaking - mirrored X from (-600, 600)
  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_odom_set({{{600_mm, 600_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  pros::delay(100);
  
  // 2. Turn 90° counter-clockwise (from -45° to -135°)
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  
  // 3. Move forward to (200, 200) - mirrored X from (-200, 200)
  chassis.pid_odom_set({{{200_mm, 200_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 4. Instead of reverse intake, set intake_lift to false and outtake
  intake_lift.set(false);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(500);
  
  // 5. Go reverse to (1200, 1200) - mirrored X from (-1200, 1200)
  chassis.pid_odom_set({{{1200_mm, 1200_mm}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  pros::delay(100);
  
  // 6. Turn 135° counter-clockwise (from -135° to -270° = 90°)
  chassis.pid_turn_set(-270_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  
  // 7. Go to (1700, 1200) - mirrored X from (-1700, 1200)
  chassis.pid_odom_set({{{1700_mm, 1200_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 8. Activate lil_krith and intake for 400ms
  lil_krith.set(true);
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(400);
  intake.set_state_and_move(Intake::State::NONE);
  pros::delay(100);
  
  // 9. Go reverse to (600, 1200) - mirrored X from (-600, 1200)
  chassis.pid_odom_set({{{600_mm, 1200_mm}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 10. Activate intake_lift and outtake for 1500ms
  intake_lift.set(true);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(1500);
  
  // 11. Go forward 300mm to (900, 1200) - mirrored X from (-900, 1200)
  chassis.pid_odom_set({{{900_mm, 1200_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  pros::delay(100);
  
  // 12. Turn to -60° (mirrored from 60°)
  chassis.pid_turn_set(-60_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  
  // 13. Move forward 300mm at -60° angle - mirrored from 60°
  // From (900, 1200), moving 300mm at -60° = (900 + 300*cos(-60°), 1200 + 300*sin(-60°))
  // = (900 + 150, 1200 - 260) = (1050, 940) - WAIT this doesn't match the pattern
  // Actually mirroring: (-750, 1460) -> (750, 1460)
  chassis.pid_odom_set({{{750_mm, 1460_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  // 14. Activate wing and move forward 450mm at -60° angle
  // Mirrored from (-525, 1850) -> (525, 1850)
  wing.set(true);
  pros::delay(100);
  chassis.pid_odom_set({{{525_mm, 1850_mm}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  
  intake.set_state_and_move(Intake::State::NONE);
}

void pid_tuning_test() {
  // Drive 24 inches (2 feet) forward for PID tuning
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
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
  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_drive_set(28_in, (DRIVE_SPEED-64));
  chassis.pid_wait();
  pros::delay(200);

  chassis.pid_turn_relative_set(84_deg, TURN_SPEED);
  chassis.pid_wait();
  intake.set_state_and_move(Intake::State::NONE);

  chassis.pid_drive_set(11_in, (DRIVE_SPEED));
  chassis.pid_wait();
  indexer.set(true);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(100);
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(2000);

  intake.set_state_and_move(Intake::State::NONE);
  indexer.set(false);
  chassis.pid_drive_set(-44_in, (DRIVE_SPEED));
  chassis.pid_wait();

  intake_lift.set(true);
  lil_krith.set(true);
  pros::delay(100);
  chassis.pid_turn_relative_set(136_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.set_state_and_move(Intake::State::INTAKING);
  chassis.pid_drive_set(17_in, (DRIVE_SPEED));
  chassis.pid_wait();
  pros::delay(1000);
  
  chassis.pid_drive_set(-17_in, (DRIVE_SPEED));
  chassis.pid_wait();
  intake.set_state_and_move(Intake::State::NONE);
  lil_krith.set(false);
  pros::delay(200);

  chassis.pid_turn_relative_set(175_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, (DRIVE_SPEED));
  chassis.pid_wait();
  indexer.set(true);
  intake.set_state_and_move(Intake::State::OUTTAKING);
  pros::delay(200);
  intake.set_state_and_move(Intake::State::INTAKING);
  pros::delay(3000);
  intake.set_state_and_move(Intake::State::NONE);
  

  chassis.pid_drive_set(-10_in, (DRIVE_SPEED));
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-100_deg, TURN_SPEED);
  chassis.pid_wait();
  wing.set(true);
  chassis.pid_drive_set(17_in, (DRIVE_SPEED));
  chassis.pid_wait();
  chassis.pid_turn_relative_set(110_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, (DRIVE_SPEED));
  chassis.pid_wait();
  wing.set(false);
  chassis.pid_drive_set(14_in, (DRIVE_SPEED));
  chassis.pid_wait();


}