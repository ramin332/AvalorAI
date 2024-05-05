#include <time.h>
#include <stdlib.h>
#define MAX_SPEED 6.4
#define MAX_REPEL_LIST 10
double k_b = 5000;                // Repulsion gain coefficient
double k_a = 1;                // Attraction gain coefficient
double x_robot_to_obstacle = 0; // position robot_x - position target_x 
double y_robot_to_obstacle = 0; // position robot_y - postition target_y 
double x_robot_to_goal = 0;
double y_robot_to_goal = 0;
double Frep_x = 0;
double Frep_y = 0;
double Fatt_x = 0;
double Fatt_y = 0;
double Ftot_x = 0;
double Ftot_y = 0;
double pos_goal_x = 0;
double pos_goal_y = 0;
double magnitude = 0;
double repulsion_angle_tot = 0;
double repulsion_angle_rep = 0;
double attraction_angle_atr = 0;
short cluster_to_repel = -1;
/* Make sure the yaw value is always between [0,360], is used for yaw determination in main 
 * and the determination of the repulsion_angle in Force_vector in current file.
 * rad2deg!
 */
double normalize_angle(double angle_to_normalize)
{
  if (angle_to_normalize > 2*M_PI)
  {
    angle_to_normalize = (angle_to_normalize -2*M_PI) * 180 / M_PI;
  }
  else if (angle_to_normalize < 0)
  {
    angle_to_normalize = (angle_to_normalize +  2*M_PI) * 180 / M_PI;
  }
  else
  {
    angle_to_normalize = (angle_to_normalize)*180 / M_PI;
  }

  return (double)angle_to_normalize;
}

/* Calculates the force vector for the artifical potential field by getting the robot position
*  receiving points to be reppeled from (or clusters or obstacles), and generating random goal positions
*  to be attracted to.
*  repel_list[][2] contains a list of [repel_list][2] coordinates. So repel_list[0][2] is t
*/  


int previous_index;
// MAY WANT TO REMOVE DISTANCE_REPEL_LIST!
void Force_vector(int MIN_POTENTIAL_DISTANCE, double x_robot_position, double y_robot_position, double pos_goal_x, double pos_goal_y)
{    
   x_robot_to_goal = x_robot_position-pos_goal_x;
   y_robot_to_goal = y_robot_position-pos_goal_y;
   Fatt_x = - k_a * (x_robot_to_goal);
   Fatt_y = - k_a * (y_robot_to_goal);
   double total_repulsion_x = x_robot_position;
   double total_repulsion_y = y_robot_position;
   for (int i = 0; i < MAX_REPEL_LIST; i++) {
        x_robot_to_obstacle = 0;
        y_robot_to_obstacle = 0;
        double distance_obstacle_robot = sqrt(x_robot_to_obstacle * x_robot_to_obstacle + y_robot_to_obstacle * y_robot_to_obstacle);
        if (distance_obstacle_robot < MIN_POTENTIAL_DISTANCE) {
            Frep_x = k_b * (((1 / distance_obstacle_robot) - (1 / 10)) *
                (1 / distance_obstacle_robot) * (1 / distance_obstacle_robot) *
                (x_robot_to_obstacle / distance_obstacle_robot));

            Frep_y = k_b * (((1 / distance_obstacle_robot) - (1 / 10)) *
                (1 / distance_obstacle_robot) * (1 / distance_obstacle_robot) *
                (y_robot_to_obstacle / distance_obstacle_robot));

            total_repulsion_x += Frep_x;
            total_repulsion_y += Frep_y;
        }
        else
        {
            Frep_x=Frep_y=0;
        }
    }
    Ftot_x = Fatt_x + total_repulsion_x;
    Ftot_y = Fatt_y + total_repulsion_y;
    if (pos_goal_x == 0 && pos_goal_y == 0)
    {
        attraction_angle_atr = attraction_angle_atr;
    }
    else
    {
        attraction_angle_atr = normalize_angle(atan2(Fatt_y,Fatt_x))*M_PI / 180;
    }
    
    repulsion_angle_rep = normalize_angle(atan2(Frep_y,Frep_x))*M_PI / 180;
    repulsion_angle_tot = attraction_angle_atr+repulsion_angle_rep;
    //printf("%d, [%f]\n", cluster_to_repel, attraction_angle_atr);
    //printf("repulsion and attraction angle before was  [%f][%f]\n",repulsion_angle_rep,attraction_angle_atr);
    //repulsion_angle = normalize_angle(atan2(Ftot_y, Ftot_x)) * M_PI / 180;
    //printf("Frep is [%f][%f], Fatt [%f][%f], repel_list[%d][%d], pos_goal is [%f][%f], angle is [%f] \n",Frep_x,Frep_y,Fatt_x,Fatt_y,repel_list[closest_index][0],repel_list[closest_index][1],pos_goal_x,pos_goal_y, repulsion_angle);
    //printf("Frep is [%f][%f][%f], Fatt [%f][%f][%f], repel_list[%d][%d] AND ROBOT FUCKING POSITION [%f][%f]\n",Frep_x,Frep_y,repulsion_angle_rep, Fatt_x,Fatt_y,attraction_angle_atr,repel_list[closest_index][0],repel_list[closest_index][1],x_robot_position, y_robot_position);
    //printf("pos_goal is [%f][%f], repel_list[closest_index][] is [%d][%d] \n",pos_goal_x,pos_goal_y, repel_list[closest_index][0],repel_list[closest_index][1]);

    if (cluster_to_repel == -1)
    {
       x_robot_to_obstacle = 0;
       y_robot_to_obstacle = 0;
       x_robot_to_goal = 0;
       y_robot_to_goal = 0;
       //pos_goal_x = 0; // Declare pos_goal_x as a static variable
       //pos_goal_y = 0; // Declare pos_goal_y as a static variable
    }
}

/* Using the repulsion angle calculated by taking the cartesian components of the Force Vector,
*  a yaw angle is decided upon. This angle will influence the wheel speeds, reducing one side
*  to minimize the difference of the angle.
*/ 
void calculateWheelSpeeds(double repulsion_angle, double yaw, double *speed)
{
    double angle_diff = repulsion_angle - yaw * M_PI / 180;
    // Normalize the angle difference to be within the range [-pi, pi] radians
    if (angle_diff >= M_PI)
    {
       angle_diff -= 2 * M_PI;
    }
    else if (angle_diff < -M_PI)
    {
       angle_diff += 2 * M_PI;
    }
    //printf(" angle_diff is %f\n", angle_diff);
    // Calculate the wheel speeds based on the angle difference
    speed[0] = MAX_SPEED; // Left front
    speed[1] = MAX_SPEED; // Right front
    speed[2] = MAX_SPEED; // Left back
    speed[3] = MAX_SPEED; // Right back
    
    // Reduce the speed of the wheels on one side based on the angle difference
    double reduction_factor = 1.0 - fabs(angle_diff) / (0.5 * M_PI); // Modify the reduction factor
    if (reduction_factor < 0.0)
    {
       reduction_factor = 0.0;
    }
    if (angle_diff >= 0)
    {
       speed[0] *= reduction_factor; // Decrease left front speed
       speed[2] *= reduction_factor; // Decrease right back speed
    }
    else if (angle_diff < 0)
    {
       speed[1] *= reduction_factor; // Decrease right front speed
       speed[3] *= reduction_factor; // Decrease left back speed
    }
}