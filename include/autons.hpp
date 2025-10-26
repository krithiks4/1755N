/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

void default_constants();
void pid_tuning_test();

void longGoalRight(); // preload + matchload -> long goal on right side
void longGoalLeft(); // preload + matchload -> long goal on left side

void rightGoals(); // preload+matchload -> long goal on right side -> pick up middle 3 -> low goal
void leftGoals(); // preload+matchload -> long goal on left side -> pick up middle 3 -> middle goal

void rightLongGoalFullMiddle(); // preload+matchload -> long goal on right side -> pick up middle 3 -> low goal -> middle goal
void leftLongGoalFullMiddle(); // preload+matchload -> long goal on left side -> pick up middle 3 -> middle goal -> low goal

void allGoalsRight(); // preload+matchload -> long goal on right side -> pick up middle 3 -> low goal -> pick up left side middle 3 -> left side long goal
void allGoalsLeft(); // preload+matchload -> long goal on left side -> pick up middle 3 -> middle goal -> pick up right side middle 3 -> right side long goal
