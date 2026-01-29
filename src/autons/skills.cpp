#include "skills.hpp"

// comment because this function felt lonely without one
void skills_auton() {
  chassis.odom_xyt_set(0_tile, 0_tile, 90_deg); // STARTING POSITION: ROTATION: FACE RIGHT, X: EXACTLY MIDDLE OF FIELD, Y: RIGHT SIDE TOUCHING PARKING ZONE

  // RIGHT SIDE

  // go to between high goal and loader (activate tonge)
  chassis.pid_odom_set({{2_tile, 0_tile}, FORWARD, DRIVE_SPEED});
  tongue_piston.set(true);
  chassis.pid_wait();

  // 2. turn towards matchloader
  chassis.pid_turn_set({2_tile, -1_tile}, FORWARD, TURN_SPEED);
  chassis.pid_wait_quick();

  // 4. thrust forward (without odom)
  chassis.pid_drive_set(14.9_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake
  scoring.set_state_and_move(Scoring::State::INTAKING);
  pros::delay(450);

  // 6. move to long goal (back some offset)
  chassis.pid_odom_set({{2_tile, 1_tile - 5_in}, REVERSE, (int) (DRIVE_SPEED * 0.5)});
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to between high goal and loader
  chassis.pid_odom_set({{2_tile, 0_tile}, FORWARD, (int) (DRIVE_SPEED * 0.5)});
  chassis.pid_wait();

  // Go to left side
  chassis.pid_odom_set({{-2_tile, 0_tile}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // LEFT SIDE

  // 2. turn towards matchloader
  chassis.pid_turn_set({-2_tile, -1_tile}, FORWARD, TURN_SPEED);
  chassis.pid_wait_quick();

  // 4. thrust forward (without odom)
  chassis.pid_drive_set(14.9_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake
  scoring.set_state_and_move(Scoring::State::INTAKING);
  pros::delay(450);

  // 6. move to long goal (back some offset)
  chassis.pid_odom_set({{-2_tile, 1_tile - 5_in}, REVERSE, (int) (DRIVE_SPEED * 0.5)});
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to between high goal and loader
  chassis.pid_odom_set({{-2_tile, 0_tile}, FORWARD, (int) (DRIVE_SPEED * 0.5)});
  chassis.pid_wait();

  // PARK

  // go to starting position to park
  chassis.pid_odom_set({{0_in, 0_in}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // point to the enemy side
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // go forward to 'charge up park'
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // go back fast to park (no odom)
  chassis.pid_drive_set(-36_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
}
