/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************

Simple controller for salamander.wbt simulation in Webots.

From the work of A. Ijspeert, A. Crespi (real Salamandra Robotica robot)
http://biorob.epfl.ch/

The authors of any publication arising from research using this software are
kindly requested to add the following reference:

  A. Ijspeert, A. Crespi, D. Ryczko, and J.M. Cabelguen.
  From swimming to walking with a salamander robot driven by a spinal cord model.
  Science, 315(5817):1416-1420, 2007.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

******************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdlib.h>
/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs */
#define NUM_MOTORS 10

/* must be the same as in salamander_physics.c */
#define WATER_LEVEL 0.0

/* virtual time between two calls to the run() function */
#define CONTROL_STEP 32

#include <webots/receiver.h>
/*
 * The following macros are useful for retrieving the x,y coordinates and
 * angles value for each robot specified by a team id: 'y' or 'b' and a
 * player id: '1', '2', or '3'
 */
#define ROBOTS 6
#define robot_get_id(t, p) (3 * ((p) - '1' + (((t) == 'y') ? ROBOTS / 2 : 0)))
#define robot_get_x(t, p) packet[robot_get_id(t, p)]
#define robot_get_y(t, p) packet[robot_get_id(t, p) + 1]
#define robot_get_orientation(t, p) packet[robot_get_id(t, p) + 2]
#define ball_get_x() packet[ROBOTS * 3]
#define ball_get_y() packet[ROBOTS * 3 + 1]


/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double phase = 0.0; /* current locomotion phase */

/* control types */
enum { AUTO, KEYBOARD, STOP };
static int control = AUTO;

/* locomotion types */
enum {
  WALK,
  SWIM,
};
static int locomotion = WALK;

/* motors position range */
static double min_motor_position[NUM_MOTORS];
static double max_motor_position[NUM_MOTORS];

double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  } else if (min == 0 && max == 0) {
    return value;
  }

  return value < min ? min : value > max ? max : value;
}

void read_keyboard_command() {
  static int prev_key = 0;
  int new_key = wb_keyboard_get_key();
  if (new_key != prev_key) {
    switch (new_key) {
      case WB_KEYBOARD_LEFT:
        control = KEYBOARD;
        if (spine_offset > -0.4)
          spine_offset -= 0.1;
        printf("Spine offset: %f\n", spine_offset);
        break;
      case WB_KEYBOARD_RIGHT:
        control = KEYBOARD;
        if (spine_offset < 0.4)
          spine_offset += 0.1;
        printf("Spine offset: %f\n", spine_offset);
        break;
      case WB_KEYBOARD_UP:
        if (ampl < 1.5)
          ampl += 0.2;
        printf("Motion amplitude: %f\n", ampl);
        break;
      case WB_KEYBOARD_DOWN:
        if (ampl > -1.5)
          ampl -= 0.2;
        printf("Motion amplitude: %f\n", ampl);
        break;
      case 'A':
        control = AUTO;
        printf("Auto control ...\n");
        break;
      case ' ':
        control = STOP;
        printf("Stopped.\n");
        break;
    }
    prev_key = new_key;
  }
}

int main() {

  WbDeviceTag receiver; /* to receive coordinate information */
  WbDeviceTag left_motor, right_motor;
  char team;   /* can be either 'y' for yellow or 'b' for blue */
  char player; /* can be either '1', '2' or '3' */
  const char *name;
  int counter = 0, max = 50;
  double y, d;

  wb_robot_init();
  name = wb_robot_get_name();
  team = name[0];
  player = name[1];
  receiver = wb_robot_get_device("receiver");

  
  const double FREQUENCY = 1.4; /* locomotion frequency [Hz] */
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 1.0; /* radians */

  /* body and leg motors */
  WbDeviceTag motor[NUM_MOTORS];
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /* distance sensors and gps devices */
  WbDeviceTag ds_left, ds_right, gps;

  /* Initialize Webots lib */
  wb_robot_init();

  /* for loops */
  int i;

  /* get the motors device tags */
  const char *MOTOR_NAMES[NUM_MOTORS] = {"motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                                         "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4"};
  for (i = 0; i < NUM_MOTORS; i++) {
    motor[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    min_motor_position[i] = wb_motor_get_min_position(motor[i]);
    max_motor_position[i] = wb_motor_get_max_position(motor[i]);
  }

  /* get and enable left and right distance sensors */
  ds_left = wb_robot_get_device("ds_left");
  wb_distance_sensor_enable(ds_left, CONTROL_STEP);
  ds_right = wb_robot_get_device("ds_right");
  wb_distance_sensor_enable(ds_right, CONTROL_STEP);

  /* get and enable gps device */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, CONTROL_STEP);

  /* enable keyboard */
  wb_keyboard_enable(CONTROL_STEP);

  /* print user instructions */
  printf("----- Salamandra Robotica -----\n");
  printf("You can steer this robot!\n");
  printf("Select the 3D window and press:\n");
  printf(" 'Left/Right' --> TURN\n");
  printf(" 'Up/Down' --> INCREASE/DECREASE motion amplitude\n");
  printf(" 'Spacebar' --> STOP the robot motors\n");
  printf(" 'A' --> return to AUTO steering mode\n");

  /* control loop: sense-compute-act */
  while (wb_robot_step(CONTROL_STEP) != -1) {

    int left_speed = 0;
    int right_speed = 0;

    while (wb_receiver_get_queue_length(receiver) > 0) {
      const double *packet = wb_receiver_get_data(receiver);
      switch (player) {
        /* case '1': /\* first player *\/ */
        case '1': /* first player */        
          /* falls through */
          y = robot_get_y(team, player); /* my own y coordinate */
          printf("teamA1 = ");
          /* printf(y); */
        case '2': /* second player */
          /*
           * This is a very stupid random behavior, up to you to improve it!
           * Advice: compare the ball coordinates to your own coordinates to
           * decide how to move the robot.
           */
          counter++;
          if (counter < max) {
            left_speed = 10;
            right_speed = 10;
          } else {
            left_speed = -10;
            right_speed = 10;
          }
          if (counter == 100) {
            counter = 0; /* reset the counter */

            /* Random number between 30 and 99 */
            max = 30 + rand() / (RAND_MAX / 70);
          }
          break;
        case '3': /* goal keeper */
          /*
           * Here, we make the goal keeper follow the y coordinate of the ball
           * while remaining inside the goal width with y inside [-0.16;+0.16]
           * This is very simple and doesn't guarantee the goal keeper will
           * remain in the goal for ever...
           */
          y = robot_get_y(team, player); /* my own y coordinate */
          d = ball_get_y() - y;

          if (d > 0.01 && y < 0.16) {
            right_speed = 10;
            left_speed = 10;
          } else if (d < -0.01 && y > -0.16) {
            right_speed = -10;
            left_speed = -10;
          }
          break;
      }

      wb_receiver_next_packet(receiver);
    }
    
    read_keyboard_command();

    if (control == AUTO) {
      /* perform sensor measurment */
      double left_val = wb_distance_sensor_get_value(ds_left);
      double right_val = wb_distance_sensor_get_value(ds_right);

      /* change direction according to sensor reading */
      spine_offset = (right_val - left_val);
    }

    if (control == AUTO || control == KEYBOARD) {
      /* increase phase according to elapsed time */
      phase -= (double)CONTROL_STEP / 1000.0 * FREQUENCY * 2.0 * M_PI;

      /* get current elevation from gps */
      double elevation = wb_gps_get_values(gps)[Y];

      if (locomotion == SWIM && elevation > WATER_LEVEL - 0.003) {
        locomotion = WALK;
        phase = target_position[6];
      } else if (locomotion == WALK && elevation < WATER_LEVEL - 0.015) {
        locomotion = SWIM;

        /* put legs backwards for swimming */
        double backwards_position = phase - fmod(phase, 2.0 * M_PI) - M_PI / 2;
        for (i = 6; i < NUM_MOTORS; i++)
          target_position[i] = backwards_position;
      }

      /* switch locomotion control according to current robot elevation and water level */
      if (locomotion == WALK) {
        /* above water level: walk (s-shape of robot body) */
        const double A[6] = {-0.7, 1, 1, 0, -1, -1};
        for (i = 0; i < 6; i++)
          target_position[i] = WALK_AMPL * ampl * A[i] * sin(phase) + spine_offset;

        /* rotate legs */
        target_position[6] = phase;
        target_position[7] = phase + M_PI;
        target_position[8] = phase + M_PI;
        target_position[9] = phase;
      } else { /* SWIM */
        /* below water level: swim (travelling wave of robot body) */
        for (i = 0; i < 6; i++)
          target_position[i] = SWIM_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
      }
    }

    /* motors actuation */
    for (i = 0; i < NUM_MOTORS; i++) {
      target_position[i] = clamp(target_position[i], min_motor_position[i], max_motor_position[i]);
      wb_motor_set_position(motor[i], target_position[i]);
    }
  }

  wb_robot_cleanup();

  return 0; /* this statement is never reached */
}
