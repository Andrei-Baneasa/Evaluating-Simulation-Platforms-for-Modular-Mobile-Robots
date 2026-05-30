#include <stdio.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SPEED 2.0
#define TIME_STEP 64

#define DOCKED_STEPS 120
#define UNDOCKED_STEPS 80

typedef enum {
  APPROACHING,
  DOCKED,
  UNDOCKING,
  SEPARATING,
  FINISHED
} State;

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

  State state = APPROACHING;
  int state_counter = 0;
  int docked_connector = 0;  // -1 = left, +1 = right, 0 = none

  printf("%s started\n", wb_robot_get_name());

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed = 0.0;
    double right_speed = 0.0;

    int left_presence = wb_connector_get_presence(left_connector);
    int right_presence = wb_connector_get_presence(right_connector);
    
    printf("%s presence L=%d R=%d locked L=%d R=%d\n",
       wb_robot_get_name(),
       wb_connector_get_presence(left_connector),
       wb_connector_get_presence(right_connector),
       wb_connector_is_locked(left_connector),
       wb_connector_is_locked(right_connector));

    switch (state) {
      case APPROACHING:
        /*
         * Only robot 1 actively approaches.
         * Robot 2 remains passive, acting as the docking target.
         */
        if (robot_number == 1) {
          left_speed = SPEED;
          right_speed = SPEED;
        }
        else if (robot_number == 2) {
          left_speed = SPEED/4;
          right_speed = SPEED/4;
        }
        

        if (right_presence) {
          wb_connector_lock(right_connector);
          docked_connector = +1;
          state = DOCKED;
          state_counter = 0;
          printf("%s: DOCKED using right connector\n", wb_robot_get_name());
        } else if (left_presence) {
          wb_connector_lock(left_connector);
          docked_connector = -1;
          state = DOCKED;
          state_counter = 0;
          printf("%s: DOCKED using left connector\n", wb_robot_get_name());
        }
        break;

      case DOCKED:
        /*
         * Move briefly while connected.
         * This demonstrates that the topology has changed and the connected
         * structure is simulated after docking. */ 
        {
          left_speed = SPEED;
          right_speed = SPEED;
        }

        state_counter++;

        if (state_counter >= DOCKED_STEPS) {
          state = UNDOCKING;
          state_counter = 0;
        }
        break;

      case UNDOCKING:
        /*
         * Runtime reconfiguration event: unlock connector during simulation.
         */
        if (docked_connector == +1) {
          wb_connector_unlock(right_connector);
          printf("%s: UNDOCKED right connector\n", wb_robot_get_name());
        } else if (docked_connector == -1) {
          wb_connector_unlock(left_connector);
          printf("%s: UNDOCKED left connector\n", wb_robot_get_name());
        }

        docked_connector = 0;
        state = SEPARATING;
        state_counter = 0;
        break;

      case SEPARATING:
        /*
         * Move away after undocking so the separation is visible.
         */
        if (robot_number == 1) {
          left_speed = -SPEED;
          right_speed = -SPEED;
        }

        state_counter++;

        if (state_counter >= UNDOCKED_STEPS) {
          state = FINISHED;
          printf("%s: dynamic reconfiguration scenario finished\n", wb_robot_get_name());
        }
        break;

      case FINISHED:
        left_speed = 0.0;
        right_speed = 0.0;
        break;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}