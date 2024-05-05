#include <stdio.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#define TIME_STEP 64

void recognition(short arr[])
{
  // Get devicetags from robot
  WbDeviceTag camera = wb_robot_get_device("camera");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");

  // Enable all units
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);

  // Determine yaw
  const double *ground_truth_attitude = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  double yaw = ground_truth_attitude[2]; // is in radian

  // Get current number of object recognized
  int i;
  int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
  const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

  // Get position of found objects
  for (i = 0; i < number_of_objects; ++i)
  {
    const double *position = wb_gps_get_values(gps);

    // Global position of the robot
    double x_global_robot = position[0];
    double y_global_robot = position[1];

    // Relative position of target to robot
    double sideways_distance_target = objects[i].position[0];
    double depth_distance_target = objects[i].position[1];
    double angle_robot_target = atan2(depth_distance_target, sideways_distance_target);
    double target_vector = sqrt(depth_distance_target * depth_distance_target + sideways_distance_target * sideways_distance_target);

    // Get target position and get global x and y vectors from them
    double global_robot_angle = yaw + angle_robot_target - 3.14;
    arr[0] = round(x_global_robot - cos(global_robot_angle) * target_vector);
    arr[1] = round(y_global_robot - sin(global_robot_angle) * target_vector);
    // printf("Model %s, Id %d\n\n",  objects[i].model ,objects[i].id);
    // printf("gps_robot: %lfm %lfm \n", x_global_robot, y_global_robot);
    // printf("global_target x, y in m new: %lfm %lfm \n", round(arr[0]), round(arr[1]));
  }
}