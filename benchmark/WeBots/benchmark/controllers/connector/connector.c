/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  An example of controller using a Connector device.
 */

#include <math.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SPEED 2.0
#define TIME_STEP 64

int main() {
  wb_robot_init();

  WbDeviceTag left_connector = wb_robot_get_device("left connector");
  WbDeviceTag right_connector = wb_robot_get_device("right connector");

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  wb_connector_enable_presence(left_connector, TIME_STEP);
  wb_connector_enable_presence(right_connector, TIME_STEP);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  int robot_number = wb_robot_get_name()[6] - '0';

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed = 0.0;
    double right_speed = 0.0;

    if (wb_connector_get_presence(left_connector)) {
      wb_connector_lock(left_connector);
    }

    if (wb_connector_get_presence(right_connector)) {
      wb_connector_lock(right_connector);
    }

    // Only MyBot 1 drives sideways/forward into the other robot.
    // if (robot_number == 1) 
    {
      left_speed = SPEED;
      right_speed = SPEED;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}