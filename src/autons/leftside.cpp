#include "leftside.hpp"

void left_side_auton() {
  // 1. go forward 1 tile
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(0.865_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // 2. turn towards left side
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  // 3. go forward 0.65 tile
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_drive_set(0.6_tile, DRIVE_SPEED*0.35);
  chassis.pid_wait();

  scoring.set_state_and_move(Scoring::State::INTAKING);
  tongue_piston.set(true);
  chassis.pid_drive_set(0.2_tile, DRIVE_SPEED*0.35);
  chassis.pid_wait();
  tongue_piston.set(false);

  // 4. turn 45 degrees counterclockwise
  chassis.pid_turn_set(-124_deg, TURN_SPEED);
  chassis.pid_wait();

  // 5. move backwards 0.75 tiles
  chassis.pid_drive_set(-0.65_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // 6. set state outtake for 1.25 seconds then turn off
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(1250);
  scoring.set_state_and_move(Scoring::State::NONE);

  // 7. go forward 1.625 tiles to matchloader
  chassis.pid_wait();
  chassis.pid_odom_set({{-1.75_tile, -0.25_tile, -136.5_deg}, FORWARD, (int)(0.75*DRIVE_SPEED)});
  chassis.pid_wait();

  // 8. turn 45 degrees counterclockwise
  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  chassis.pid_wait();

  // 9. set tongue_piston true
  tongue_piston.set(true);

  // 10. set state intaking
  scoring.set_state_and_move(Scoring::State::INTAKING);

  // 11. go forward 14 inches
  chassis.pid_drive_set(16.5_in, 0.6*DRIVE_SPEED);
  chassis.pid_wait();

  // 12. back 4 inches
  chassis.pid_drive_set(-4_in, 0.6*DRIVE_SPEED);
  chassis.pid_wait();

  // 13. forward 5 inches
  chassis.pid_drive_set(5_in, 0.6*DRIVE_SPEED);
  chassis.pid_wait();

  // 14. back 1.75 tiles
  chassis.pid_drive_set(-1.9_tile, 0.8*DRIVE_SPEED);
  chassis.pid_wait();

  // 15. set state high goal for 2 seconds
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // 16. move forward 0.75 tile
  chassis.pid_drive_set(0.75_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // 17. turn towards 135 degrees then backwards 1 tile
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.65_tile, 0.75*DRIVE_SPEED);
  chassis.pid_wait();

  // 18. activate wing piston
  wing_piston.set(true);

  // go staight
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // 19. move backward one tile
  chassis.pid_drive_set(-0.8_tile, 0.75*DRIVE_SPEED);

  // 18. let wing piston fal
  wing_piston.set(false);

  // 20. move forward for 100ms
  chassis.pid_drive_set(1_tile, DRIVE_SPEED);
  chassis.pid_wait();
}

