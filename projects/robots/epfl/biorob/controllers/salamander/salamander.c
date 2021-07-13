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
#include <stdlib.h>
#include <time.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
/* #include <Point.h> */
/* #include <ColorFinder.h> */
/* #include <Image.h> */
/* #include <ImgProcess.h> */

/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs */
#define NUM_MOTORS 10

/* must be the same as in salamander_physics.c */
#define WATER_LEVEL 0.0

/* virtual time between two calls to the run() function */
/* #define CONTROL_STEP 32 */
#define CONTROL_STEP 64

/* camera */
#define THRESHOLD 200
#define MAX_SPEED 600
#define BACKWARD_SPEED 200
#define K_TURN 4

/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double phase = 0.0; /* current locomotion phase */
static int searching_flag =1;
static int initializing_flag =1;
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
  const double FREQUENCY = 1.4; /* locomotion frequency [Hz] */
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 1.0; /* radians */
  srand((unsigned int)time(NULL));
  double initial_spine_offset = pow(-1,rand()%2)*0.1;
  /* sleep(0.1); */
  printf("rand =%d",rand());
  printf("initial_spine_offset=%f",initial_spine_offset);
  /* body and leg motors */
  WbDeviceTag motor[NUM_MOTORS];
  /* double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; */
  /* 0428 2 */
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI, M_PI, M_PI, M_PI};  

  /* distance sensors and gps devices */
  WbDeviceTag ds_left, ds_right, gps;

 /* camera devices */
  WbDeviceTag cam;
  unsigned short width, height;
  /* Initialtialize Webots lib */
  wb_robot_init();

  /* for loops */
  int i;

  /* get the motors device tags */
  const char *MOTOR_NAMES[NUM_MOTORS] = {"motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                                         "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4"}; /* 0428 1 */
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

  /* get camera */
  cam = wb_robot_get_device("camera");
  /* wb_camera_enable(cam, TIME_STEP_CAM); */
  wb_camera_enable(cam, CONTROL_STEP);  
  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);
  float *intensity = (float *)malloc(sizeof(float) * width);

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

  int counter =0;
  /* control loop: sense-compute-act */
  while (wb_robot_step(CONTROL_STEP) != -1) {
    if (40<counter){
      initializing_flag=0;
    }else{
      counter++;
    }
    read_keyboard_command();
    
    if (control == AUTO) {
      /* perform sensor measurment */
      double left_val = wb_distance_sensor_get_value(ds_left);
      double right_val = wb_distance_sensor_get_value(ds_right);

      /* change direction according to sensor reading */
      /* spine_offset = (right_val - left_val); */

    /* add 0502 camera */
    /* use  camera  */
    const unsigned char *image;
    int i, j, delta, max, index_max = 0, speed[2];
    image = wb_camera_get_image(cam);

  /*   Point2D pos; */
  /* mBuffer->m_BGRAFrame->m_ImageData = (unsigned char *)image; */
  /* // Convert the image from BGRA format to HSV format */
  /* ImgProcess::BGRAtoHSV(mBuffer); */
  /* // Extract position of the ball from HSV verson of the image */
  /* pos = mFinder->GetPosition(mBuffer->m_HSVFrame);     */
    
    int sum_blue=0;
    int sum_red=0;
    int sum_green=0;
    // 2. Handle the sensor values
    for (i = 0; i < width; i++) {
      int count_blue = 0;
      int count_green = 0;
      int count_red = 0;
      for (j = 0; j < height-24; j++){
        count_blue += wb_camera_image_get_blue(image, width, i, j); /* why wb_camera_image_get_blue detect red */
        count_green += wb_camera_image_get_green(image, width, i, j);
        count_red += wb_camera_image_get_red(image, width, i, j);
      }
      intensity[i] = count_red;
      sum_blue+= count_blue;
      sum_green+= count_green;
      sum_red+= count_red;
    }
    int ave_blue = sum_blue / width;
    int ave_red = sum_red / width;
    
    int ave_green = sum_green / width;
    /* double area_red = sum_red / ((height-24)*width*(height-24)); */
    /* printf("area_red =%f\n", area_red); */
    /* printf("ave_red =%d\n", ave_red); */
    /* printf("ave_blue =%d\n", ave_blue); */
    /* printf("ave_green =%d\n", ave_green); */
    /* printf("sum_red =%d\n", sum_red); */
    /* printf("sum_blue =%d\n", sum_blue); */
    /* printf("sum_green =%d\n", sum_green); */
    int thre=9800;
    /* if (ave < thre){ */
    /*   if (spine_offset > -0.4){ */
    /*     spine_offset -= 0.2; */
    /*   }else{ */
    /*     spine_offset=0; */
    /*   } */
    /* } */
    delta = 0;
    max = 0;
    for (i = 0; i < width; i++) {
      if (max < intensity[i]) {
        max = intensity[i];
        index_max = i;
        delta = i - (width / 2);
      }
    }
    
    /* printf("max=%d\n", max); */
    /* if (max <2000){ */
    /*   printf("max=%d\n", max); */
    /*   delta=100; */
    /* } */
    
    /* int iter = 0; */
    /* if (index_max >= 0 && index_max < height) { */
    /*   for (j = 0; j < height; j++) { */
    /*     if (THRESHOLD < wb_camera_image_get_red(image, width, index_max, j)) */
    /*       iter++; */
    /*   } */
    /* } else */
    /*   iter = (MAX_SPEED * height) / (MAX_SPEED + BACKWARD_SPEED); */
    /* printf("iter=%d\n", iter); */
    /* if(delta == 100){ */
    /*   spine_offset = 0.4; */
    /* }else  */
    
    /* if(delta > 0){ */
    /*   if (spine_offset < 0.4) */
    /*     spine_offset += 0.1; */
    /*   printf("Spine offset: %f\n", spine_offset); */
    /* }else{ */
    /*   if (spine_offset > -0.4){ */
    /*     spine_offset -= 0.1; */
    /*   } */
    /*   printf("Spine offset: %f\n", spine_offset); */
    /* } */
    /* comment out */
    if(delta > 0){
      if (spine_offset > -0.4){
        spine_offset -= 0.1;
      }
    }else{
      if (spine_offset < 0.4){
          spine_offset += 0.1;
      }
    }
/* main */
    /* if (ave_blue> ave_red || ave_green> ave_red){ */
    /* /\* if (0.4> area_red){ *\/ */
    /*   searching_flag +=1; */
    /*   printf("searching_flag = %d\n",searching_flag); */
    /* }else{ */
    /*   searching_flag=0; */
    /*   printf("searching_flag reset\n"); */
    /* } */

    /* if(ave_blue<ave_red && ave_green<ave_red){ */
    /*   searching_flag=-1; */
    /* }else  */if (ave_blue<ave_red || ave_green<ave_red){
      searching_flag=0;
      printf("searching_flag reset\n");
    }else{
      searching_flag +=1;
      printf("searching_flag = %d\n",searching_flag);
    }
    /* printf("ave_red =%d\n",ave_red); */
    
    /* if (sum_blue> sum_red){ */
    /*   searching_flag +=1; */
    /* }else{ */
    /*   searching_flag=0; */
    /*   /\* spine_offset=0; *\/ */
    /*   printf("searching_flag reset\n"); */
    /* } */
    if (initializing_flag){
        printf("in initializing");
        if(searching_flag<20){
          spine_offset=initial_spine_offset*-1;
        }else{
          spine_offset=initial_spine_offset ;
        }
      }else{
      if(searching_flag==0){
        printf("find\n");
      }else if (0< searching_flag && searching_flag<10){
        spine_offset=0.3;
      }else if(30<searching_flag){
        spine_offset=0.4;
      }
    }
    
/* main           */
    /* if(60<searching_flag){ */
    /*   spine_offset=0.3; */
    /* }else if(30<searching_flag && initializing_flag ==1){ */
    /*   spine_offset=initial_spine_offset*-1; */
    /*   printf("in initializing"); */
    /* }else if (0<searching_flag && initializing_flag==1){ */
    /*   spine_offset=initial_spine_offset ; */
    /*   printf("in initializing"); */
    /* } */
    
    
      /* printf("sum_blue> sum_red %d,%d\n",sum_blue,sum_red); */
      /* if (((searching_flag%4)==0) || (((searching_flag%4)==1))){ */
      /*   spine_offset=0.1; */
      /* }else{ */
      /*   spine_offset= -0.1; */
      /* } */
    /* } */
    printf("spine_offset=%f\n",spine_offset);
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
        /* 0428 2 */
        /* target_position[6] = M_PI; */
        /* target_position[7] =  M_PI; */
        /* target_position[8] =  M_PI; */
        /* target_position[9] = M_PI; */       
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
