#include "skills.hpp"

// 7972

// comment because this function felt lonely without one
void skills_auton() {
  chassis.odom_xyt_set(0_in, 0_in, 90_deg); // STARTING POSITION: ROTATION: FACE RIGHT, X: EXACTLY MIDDLE OF FIELD, Y: RIGHT SIDE TOUCHING PARKING ZONE

  // RIGHT SIDE

  // go to between high goal and loader (activate tonge)
  chassis.pid_odom_set({{2_tile - 3_in, 0_tile}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // 2. turn towards matchloader
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  tongue_piston.set(true);
  pros::delay(500);

  // 4. thrust forward (without odom)
  chassis.pid_drive_set(13.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake
  scoring.set_state_and_move(Scoring::State::INTAKING);
  for (int i = 0; i < 5; i++) {
    chassis.pid_drive_set(-2_in, DRIVE_SPEED);
    chassis.pid_wait();

    if (i == 2) scoring.set_state_and_move(Scoring::State::OUTTAKING);

    chassis.pid_drive_set(3_in, DRIVE_SPEED);
    chassis.pid_wait();

    scoring.set_state_and_move(Scoring::State::INTAKING);
  }

  // 6. move to long goal (back some offset)
  chassis.pid_odom_set(-1_tile - 8_in, (int) (DRIVE_SPEED * 0.5));
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  chassis.odom_xy_set(2_tile, 1_tile - 5_in); // checkp
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to between high goal and loader
  chassis.pid_odom_set({{2_tile, 0.5_tile}, FORWARD, (int) (DRIVE_SPEED * 0.5)});
  chassis.pid_wait();

  // Go to left side
  chassis.pid_odom_set({{-2_tile + 2_in, 0.5_tile}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // LEFT SIDE

  // 2. turn towards matchloader
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  tongue_piston.set(true);
  pros::delay(500);

  // 4. thrust forward (without odom)
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake
  scoring.set_state_and_move(Scoring::State::INTAKING);
  for (int i = 0; i < 5; i++) {
    chassis.pid_drive_set(-2_in, DRIVE_SPEED);
    chassis.pid_wait();

    if (i == 2) scoring.set_state_and_move(Scoring::State::OUTTAKING);

    chassis.pid_drive_set(3_in, DRIVE_SPEED);
    chassis.pid_wait();
    
    scoring.set_state_and_move(Scoring::State::INTAKING);
  }

  // 6. move to long goal (back some offset)
  chassis.pid_odom_set(-1_tile - 8_in, (int) (DRIVE_SPEED * 0.5));
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  chassis.odom_xy_set(-2_tile, 1_tile - 5_in); // checkp2
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to posisition before parking zone
  chassis.pid_odom_set({{0.5_tile, -0.5_tile, 90_deg}, FORWARD, (int) (DRIVE_SPEED * 0.5)});
  chassis.pid_wait();

  // PARK

  // go back to 'charge up park'
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // go forward fast to park (no odom)
  chassis.pid_drive_set(40_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
}
