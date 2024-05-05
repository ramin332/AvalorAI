#include <stdio.h>
#define TIME_STEP 32
// how many sensors are on the robot
#define MAX_SENSOR_NUMBER 16
// maximal value returned by the sensors
#define MAX_SENSOR_VALUE 1024
// minimal weight for the robot to turn
#define WHEEL_WEIGHT_THRESHOLD 50
// minimal distance, in meters, for an obstacle to be considered
#define MIN_DISTANCE 1.0
// maximal speed allowed
#define MAX_SPEED 6.4
// enum to represent the state of the robot
typedef enum
{
  FORWARD,
  LEFT,
  RIGHT
} State;

// initialise state and speed
State state = FORWARD;
double speed[4] = {0.0, 0.0, 0.0, 0.0};

// State machine function that avoids obstacles based on distance sensor data
double *obstacle_avoidance(double *wheel_weight_total)
{
  // Set up memory for speed array
  memset(speed, 0, sizeof(double) * 4);

  // State machine to handle the direction of the robot
  switch (state)
  {
  // when the robot is going forward, it will start turning in either direction when an obstacle is close enough
  case FORWARD:
    if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD)
    {
      printf("1 \n");
      speed[0] = 0.7 * MAX_SPEED;
      speed[1] = -0.7 * MAX_SPEED;
      speed[2] = 0.7 * MAX_SPEED;
      speed[3] = -0.7 * MAX_SPEED;
      state = LEFT;
    }
    else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD)
    {
      printf("2 \n");
      speed[0] = -0.7 * MAX_SPEED;
      speed[1] = 0.7 * MAX_SPEED;
      speed[2] = -0.7 * MAX_SPEED;
      speed[3] = 0.7 * MAX_SPEED;
      state = RIGHT;
    }
    else
    {
      printf("3 \n");
      speed[0] = MAX_SPEED;
      speed[1] = MAX_SPEED;
      speed[2] = MAX_SPEED;
      speed[3] = MAX_SPEED;
    }
    break;
  // when the robot has started turning, it will go on in the same direction until no more obstacle are in sight
  // this will prevent the robot from being caught in a loop going left, then right, then left, and so on.
  case LEFT:
    if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD)
    {
      printf("4 \n");
      speed[0] = 0.7 * MAX_SPEED;
      speed[1] = -0.7 * MAX_SPEED;
      speed[2] = 0.7 * MAX_SPEED;
      speed[3] = -0.7 * MAX_SPEED;
    }
    else
    {
      printf("5 \n");
      speed[0] = MAX_SPEED;
      speed[1] = MAX_SPEED;
      speed[2] = MAX_SPEED;
      speed[3] = MAX_SPEED;
      state = FORWARD;
    }
    break;
  case RIGHT:
    if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD)
    {
      printf("6 \n");
      speed[0] = -0.7 * MAX_SPEED;
      speed[1] = 0.7 * MAX_SPEED;
      speed[2] = -0.7 * MAX_SPEED;
      speed[3] = 0.7 * MAX_SPEED;
    }
    else
    {
      printf("7 \n");
      speed[0] = MAX_SPEED;
      speed[1] = MAX_SPEED;
      speed[2] = MAX_SPEED;
      speed[3] = MAX_SPEED;
      state = FORWARD;
    }
    break;
  default:
    state = FORWARD;
  }

  return speed;
}
