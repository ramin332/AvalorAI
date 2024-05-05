#include <webots/supervisor.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <webots/robot.h>
#define FIELD_SIZE 1000.0
#define MAX_ROBOTS 10
#define TIME_STEP 64

int main(int argc, char **argv) {
  WbNodeRef robot_node[MAX_ROBOTS];
  WbFieldRef trans_field[MAX_ROBOTS];
  WbFieldRef rotation_field[MAX_ROBOTS];
  WbFieldRef battery_field[MAX_ROBOTS];
  srand(time(NULL));  // initialize random seed
  wb_robot_init();

  // Place the robots randomly within the field
  for (int i = 0; i < MAX_ROBOTS; i++) {
    char node_name[15];
    sprintf(node_name, "Scout_%d", i + 1);
    robot_node[i] = wb_supervisor_node_get_from_def(node_name);
    trans_field[i] = wb_supervisor_node_get_field(robot_node[i], "translation");
    double x = (int) rand() % 100 - 50;
    double y = (int) rand() % 100 - 50;
    double rot = rand() % 200*M_PI - 100*M_PI;
    wb_supervisor_field_set_sf_vec3f(trans_field[i], (const double[]){x, y, 0.01});
    battery_field[i] = wb_supervisor_node_get_field(robot_node[i], "battery");
    wb_supervisor_field_set_mf_float(wb_supervisor_node_get_field(robot_node[i], "battery"),0,933120);
    rotation_field[i] = wb_supervisor_node_get_field(robot_node[i], "rotation");
    double rotation[4] = {0, 0, 1, rot/100}; // random axis-angle rotation
    wb_supervisor_field_set_sf_rotation(rotation_field[i], rotation);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    // Check if all battery values are zero
    double battery_values[MAX_ROBOTS];
    bool all_batt_empty = true;
    for (int i = 0; i < MAX_ROBOTS; i++) {
      battery_values[i] = wb_supervisor_field_get_mf_float(battery_field[i], 0);
      if (battery_values[i] != 0.0) {
        all_batt_empty = false;
      }
    } 
    
    // Perform actions if all battery values are zero
    if ((all_batt_empty || (wb_robot_get_time() > (2*3600+ 15*60) && wb_robot_get_time() < (2*3600 + 15.2*60)))) 
    {
      char filename[40];
      time_t current_time = time(NULL);
      struct tm *local_time = localtime(&current_time);   // Convert to local time
      strftime(filename, sizeof(filename), "screenshot_%Y%m%d_%H%M%S.png", local_time); // Format the timestamp and append to filename                                                
      wb_supervisor_export_image(filename, 50);  // save screenshot of current view
      wb_supervisor_world_save(NULL);
      wb_supervisor_world_reload();
      for (int i = 0; i < MAX_ROBOTS; i++) {
        wb_supervisor_field_set_mf_float(wb_supervisor_node_get_field(robot_node[i], "battery"), 0, 933120);
      }
    }
  }
  wb_robot_cleanup();
  return 0;
}
