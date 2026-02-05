#include "skills.hpp"

// Autonomous Constants
// These are out of 127
const int DRIVE_SPEED = 105;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

// comment because this function felt lonely without one
void skills_auton() {
  chassis.odom_xyt_set(0_in, 0_in, 90_deg); // STARTING POSITION: ROTATION: FACE RIGHT, X: EXACTLY MIDDLE OF FIELD, Y: RIGHT SIDE TOUCHING PARKING ZONE

  // RIGHT SIDE

  // go to between high goal and loader (activate tonge)
  chassis.pid_odom_set({{2_tile - 6.5_in, 0_tile}, FORWARD, (int) (DRIVE_SPEED * 0.8)});
  chassis.pid_wait();

  // 2. turn towards matchloader
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  tongue_piston.set(true);
  pros::delay(300);

  // 4. thrust forward (without odom)
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_drive_set(15_in, DRIVE_SPEED*0.8);
  chassis.pid_wait_quick();

  // 5. intake
  matchload_wiggle(6);

  // 6. move to long goal (back some offset)
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_odom_set({{2_tile - 7_in, 1_tile - 6_in}, REVERSE, (int) (DRIVE_SPEED * 0.65)});
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  highgoal_antijam(7);

  chassis.odom_xy_set(2_tile - 1.5_in, 1_tile - 5_in); // reset pos
  scoring.set_state_and_move(Scoring::State::NONE);
  tongue_piston.set(false);

  // go to between high goal and loader
  chassis.pid_odom_set({{2_tile, 0.5_tile}, FORWARD, (int) (DRIVE_SPEED * 0.65)});
  chassis.pid_wait();

  // Go to left side
  chassis.pid_odom_set({{-2_tile, 0.5_tile}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // LEFT SIDE

  // 2. turn towards matchloader
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  tongue_piston.set(true);
  pros::delay(500);

  // 4. thrust forward (without odom)
  chassis.pid_drive_set(19_in, DRIVE_SPEED*0.9);
  chassis.pid_wait_quick();

  // 5. intake
  matchload_wiggle(6);

  // 6. move to long goal (back some offset)
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_odom_set({{-2_tile + 2_in, 1_tile - 2_in}, REVERSE, (int) (DRIVE_SPEED * 0.65)});
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  highgoal_antijam(7);

  chassis.odom_xy_set(-2_tile - 1.5_in, 1_tile - 5_in); // reset pos
  scoring.set_state_and_move(Scoring::State::NONE);
  tongue_piston.set(false);

  // go to between high goal and loader
  chassis.pid_odom_set({{-2_tile, 0.5_tile}, FORWARD, (int) (DRIVE_SPEED * 0.5)});
  chassis.pid_wait();

  // PARK

  // go to posisition before parking zone and shoot out balls if collected
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  chassis.pid_odom_set({{-0.5_tile, -0.5_tile, 90_deg}, REVERSE, (int) (DRIVE_SPEED * 0.5)});
  chassis.pid_wait();

  // trhust
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-100_in, 127);
  chassis.pid_wait();
}
