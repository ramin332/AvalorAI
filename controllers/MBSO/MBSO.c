/* Main Controller
 * Information that is send between robot are shorts
 * Local target information that is used for navigation and movement are doubles
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
/* Including function files */
#include "E:\AvalorAI\controllers\MBSO\recognition.c"
#include "E:\AvalorAI\controllers\MBSO\levy_distribution.c"
#include "E:\AvalorAI\controllers\MBSO\\normal_distribution.c"
#include "E:\AvalorAI\controllers\MBSO\fuzzy_treshold.c"
#include "E:\AvalorAI\controllers\MBSO\save.c"
#include "E:\AvalorAI\controllers\MBSO\APF.c"
/* Definitions for Sensor and Robot*/
#define WHEEL_WEIGHT_THRESHOLD 50
#define MAX_SENSOR_NUMBER 16
#define MAX_SENSOR_VALUE 1024
#define MIN_DISTANCE_SENSOR 3
#define MAX_SPEED 6.4
#define TARGET_RECOGNITION_TRESHOLD 300
#define LOCAL_SEARCH_TIME_TRESHOLD 30 * 60
#define RANGE_OF_CLUSTER 100
#define GAMMA_MAX 33
#define EMITTER_RANGE 40
#define BS_TRESHOLD 0
#define N_Chargers 4
#define MIN_POTENTIAL_DISTANCE 75
/* Enables broadcasting to all channels */
#define WB_CHANNEL_BROADCAST -1
/* Time */
#define TIME_STEP 64
/* How many targets can the robot find? */
#define AMOUNT_OF_TARGETS 9
#define MAX_ROBOTS 10
/* States */
typedef enum
{
    RESET,   //0
    LEVY,    //1
    GOTO,    //2
    HOMING,  //3
    AVOID,   //4
    LEFT,    //5
    RIGHT,   //6
} State;



/////////////////////////////////Main script/////////////////////////////////
/////////////////////////////////Main script/////////////////////////////////
/////////////////////////////////Main script/////////////////////////////////
int main(int argc, const char *argv[])
{
    wb_robot_init();
    srand(time(NULL)); // Seed the random number generator with the current time
    int CUSTOM_SEED = *argv[1];
    WbDeviceTag communication;
    communication = wb_robot_get_device("emitter");
    wb_emitter_set_range(communication, EMITTER_RANGE);
    WbDeviceTag communication_receiver;
    communication_receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(communication_receiver, TIME_STEP);
    wb_robot_battery_sensor_enable(TIME_STEP);
    /////////////////////////////////Wheels init/////////////////////////////////
    /* Stores device IDs */
    WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
    WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
    WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
    WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");

    /* Init motors */
    wb_motor_set_position(front_left_wheel, INFINITY);
    wb_motor_set_position(front_right_wheel, INFINITY);
    wb_motor_set_position(back_left_wheel, INFINITY);
    wb_motor_set_position(back_right_wheel, INFINITY);

    /* Init speed value */
    double front_left_speed = 0.0;
    double front_right_speed = 0.0;
    double back_left_speed = 0.0;
    double back_right_speed = 0.0;

    /* Set init velocity */
    wb_motor_set_velocity(front_left_wheel, front_left_speed);
    wb_motor_set_velocity(front_right_wheel, front_right_speed);
    wb_motor_set_velocity(back_left_wheel, back_left_speed);
    wb_motor_set_velocity(back_right_wheel, back_right_speed);

    /////////////////////////////////ONLOOKER specific init/////////////////////////////////
    WbDeviceTag gps = wb_robot_get_device("gps");
    WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
    wb_gps_enable(gps, TIME_STEP);
    wb_inertial_unit_enable(inertial_unit, TIME_STEP);

    /////////////////////////////////Sensory init/////////////////////////////////
    typedef struct
    {
        WbDeviceTag device_tag;
        double wheel_weight[4];
    } SensorData;

    /* Home position array*/
    static short home_array[N_Chargers][2] = {{300, 300}, {300, -300}, {-300, 300}, {-300, -300}};

    /* Effect Sensor index[0-15] on wheel index[0-3] */
    static SensorData sensors[MAX_SENSOR_NUMBER] = 
    {
    {.wheel_weight = {150, 0, 150, 0}}, {.wheel_weight = {200, 0, 200, 0}}, 
    {.wheel_weight = {300, 0, 300, 0}}, {.wheel_weight = {600, 0, 600, 0}}, 
    {.wheel_weight = {0, 600, 0, 600}}, {.wheel_weight = {0, 300, 0, 300}},
    {.wheel_weight = {0, 200, 0, 200}}, {.wheel_weight = {0, 150, 0, 150}}, 
    {.wheel_weight = {0, 0, 0, 0}}, {.wheel_weight = {0, 0, 0, 0}}, 
    {.wheel_weight = {0, 0, 0, 0}}, {.wheel_weight = {0, 0, 0, 0}}, 
    {.wheel_weight = {0, 0, 0, 0}}, {.wheel_weight = {0, 0, 0, 0}}, 
    {.wheel_weight = {0, 0, 0, 0}}, {.wheel_weight = {0, 0, 0, 0}}
    };

    /* Enabel distance sensors */
    char sensor_name[5] = "";
    int i, j;
    for (i = 0; i < MAX_SENSOR_NUMBER; ++i)
    {
        sprintf(sensor_name, "so%d", i);
        sensors[i].device_tag = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(sensors[i].device_tag, TIME_STEP);
    }

    /* Init sensor influence on wheel speed variables*/
    double wheel_weight_total[4] = {0.0, 0.0, 0.0, 0.0};
    double distance, speed_modifier, sensor_value;
    double speed[4] = {0.0, 0.0, 0.0, 0.0};

    /* State control, resetting and updating of targets*/
    int state = LEVY;
    short xy_array_target[2] = {0};
    
    /* Onlooker or Scout?*/
    bool onlooker = false;
    bool home_reached = false;
    bool onlooker_treshold = true;
    
     /* Print final results only once*/
    bool has_printed = false;
  
    /* Global position of the target and home*/
    int x_t = 0;
    int y_t = 0;
    int x_h = 0;
    int y_h = 0;
    
    /* Initalize Normal Generator*/
    double mu;
    double sigma = M_PI;
    
    /* Initialize distance vector to chosen home or target double because distance is calculated on 0.1 m accuracy*/ 
    double distance_target = 0.0;

     /* Initialize target timer from last target found for cluster forming*/
    double last_target_time = 0.0; 
    
    /* Information states about the robot*/
    short BS;     // Battery state --> Battery level of robot
    short OS = 0; // Operatation state --> Targets found by current robot
    short IS = 0; // Interadtion state --> Robots in range

    /////////////////////////////////Helper functions/////////////////////////////////
    /* Initialize normal generator 
     * Check normal_distribution.c file for generator
     */
    double normal = normalize_angle(randn(mu, sigma, CUSTOM_SEED));
    double reset_normal()
    {
        return randn(mu, sigma,  CUSTOM_SEED);
    } 
    
    /* Initialize gamma to favour exploration 
    * set_gamma sets gamma to favour exploitation when function is run
    */ 
    double gamma_levy = GAMMA_MAX;
    double set_gamma()
    {
        gamma_levy = 10;
        return gamma_levy;
    }
    
    /* Initialize Levy generator 
    * Check levy_distribution.c.c file for generator
    */
    short levy = levy_distribution(CUSTOM_SEED, gamma_levy);
    short reset_levy()
    {
        return levy_distribution(CUSTOM_SEED, gamma_levy);
    }
    
    /* Find first zero in targets_array of current robot */
    short targets_array[AMOUNT_OF_TARGETS][3] = {0};
    short findfirstzero(short ROWS, short targets_array[AMOUNT_OF_TARGETS][3])
    {
        for (int i = 0; i < ROWS; i++)
        {
            if (targets_array[i][0] == 0 && targets_array[i][1] == 0)
            {
                return i;
            }
        }
        return -1;
    }
    
    /* Euclidean distance between two points (x1, y1) and (x2, y2) */
    double euclidean_distance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }

    /* Init local buffer for receiving targets by other robots 
     * Init array to send to other robots 
     * last_updated is a counter that helps enhance performance
     */
    short localbuffer[AMOUNT_OF_TARGETS * 2] = {0};
    short targets_array_received[AMOUNT_OF_TARGETS][2] = {0};
    short send[AMOUNT_OF_TARGETS * 2] = {0};
    double last_updated[MAX_ROBOTS] = {0.0};

    /////////////////////////////////Receiver function/////////////////////////////////
    /* Fill targets_array[2] with newly aquired targets
     * Calculate interaction state IS, based on the distance of other emitters
     * This is complicated as robots only receive the different values of every emitter
     * Emitter 1 can send [50,51,51,52]
     * Emitter 2 can send [66,66,67,68]
     * The receiver will receive [50,66,51,66,51,67,52,68]
     * It needs to differentiate between the varying numbers, therefore a more complicated scheme is used
     */
    short signal_strength;
    double robot_list[MAX_ROBOTS] = {0.0};
    void get_receiver_data()
    {
        /* Is there at least one packet in the receiver's queue? */
        if (wb_receiver_get_queue_length(communication_receiver) > 0)
        {
          const unsigned char *buffer = wb_receiver_get_data(communication_receiver);
            signal_strength = sqrt(1 / wb_receiver_get_signal_strength(communication_receiver));
            for (i = 0; i < MAX_ROBOTS; i++) {
                /* Check if the signal strength is 0 or 99 (indicating no signal or signal lost) */ 
                if (signal_strength == 0 || signal_strength == 99) {
                    robot_list[i] = 0;  // No signal or signal lost, set the value to 0
                    last_updated[i] = wb_robot_get_time();  // Update the last_updated timestamp
                    break;  
                } 
                else {
                    /* Assign the signal strength to the first empty slot */
                    if (robot_list[i] == 0) {
                        robot_list[i] = signal_strength;  
                        last_updated[i] = wb_robot_get_time();
                        break;  
                    /* Update the existing robot's signal strength instead of creating a new entry */
                    } 
                    else if (abs(robot_list[i] - signal_strength) <= 1) {
                        robot_list[i] = (robot_list[i] + signal_strength) / 2;
                        last_updated[i] = wb_robot_get_time();  
                        break;  
                    }
                }
            }
            /* Remove duplicates in robot_list */ 
            for (i = 0; i < MAX_ROBOTS - 1; i++) 
            {
                if (robot_list[i] == 0) {
                    continue;  // Skip empty slots
                }
                for (j = i + 1; j < MAX_ROBOTS; j++) 
                {
                    if (robot_list[j] == 0) {
                        continue;  // Skip empty slots
                    }
                    if (robot_list[i] == robot_list[j]) {
                         robot_list[j] = 0;  // Duplicate found, set the value to 0
                    }
                }
            }
            /* Check if any value hasn't been updated in the last 10 seconds and set it to 0 */
            double current_time = wb_robot_get_time();
            for (i = 0; i < MAX_ROBOTS; i++) {
                if (robot_list[i] != 0 && current_time - last_updated[i] > 10.0) {
                    robot_list[i] = 0;  
                }
            }
            /* Copy buffer data into locally defined variable with amount of bytes to be copied, now n*2*2 (with short) */
            memcpy(localbuffer, buffer, AMOUNT_OF_TARGETS*2*2);
            /* Translate the singular array of localbuffer [targetsx + targetsy] --> [AMOUNT_OF_TARGETS][2] 
             * This is how the emitter sends the targets, not as an array but as a one-dimensional list
             */
            for (i = 0; i < AMOUNT_OF_TARGETS; ++i)
            {
                targets_array_received[i][0] = localbuffer[i * 2];
                targets_array_received[i][1] = localbuffer[i * 2 + 1];   
            }
            /* Find the first zero of the targets_array */
            short first_zero_index = findfirstzero(AMOUNT_OF_TARGETS, targets_array);
            /* The robots target_array is updated with that of the received target array
             * First we check if the received targets are not already in our own list
             */
            for (short i = 0; i < AMOUNT_OF_TARGETS; i++)
            {
                short received_target_found = 0;
                for (short j = 0; j < AMOUNT_OF_TARGETS; j++)
                {
                    if (targets_array_received[i][0] == targets_array[j][0] && targets_array_received[i][1] == targets_array[j][1])
                    {
                        received_target_found = 1;
                        break;
                    }
                }
                if (!received_target_found && (targets_array_received[i][0] != 0) && (targets_array_received[i][1] != 0))
                {
                    targets_array[first_zero_index][0] = targets_array_received[i][0];
                    targets_array[first_zero_index][1] = targets_array_received[i][1];
                    first_zero_index = -1;
                    break;
                }
            }
            /* Receive next package */
            wb_receiver_next_packet(communication_receiver);
            
            /* Calculation of interction state, a value of 0 or emitter_range-1 means no signal again */
            int num_zeros = 0, num_non_zeros = 0;
            for (i = 0; i < MAX_ROBOTS - 1; i++)
            {
                if (robot_list[i] == 0 || robot_list[i] == EMITTER_RANGE-1)
                {
                    num_zeros++;
                }
                else
                {
                    num_non_zeros++;
                }
            }
            IS = num_non_zeros;
            //printf("robot_list[%d]\n",IS );
            // for (i = 0; i < MAX_ROBOTS; i++)
            // {
            // printf("index [%d] is robot_list[%f]\n",i, robot_list[i]);
            
            // }
        }
        
    }
    /////////////////////////////////Emitter function/////////////////////////////////
    /* Send our own target_array list filled with targets to other robots
     * Update our operation state, which is how many targets have we found in the last 30 minutes
     */
    void send_emitter_data()
    {
        /* Fill targets_array[2] with newly aquired targets*/
        wb_emitter_set_channel(communication, WB_CHANNEL_BROADCAST);
        bool target_found_exists = false;
        short target_index = -1;
        for (i = 0; i < AMOUNT_OF_TARGETS; i++)
        {
            if (xy_array_target[0] == targets_array[i][0] && xy_array_target[1] == targets_array[i][1])
            {
                target_found_exists = true;
                break;
            }
            else if (targets_array[i][0] == 0 && targets_array[i][1] == 0 && target_index == -1)
            {
                target_index = i;
            }
        }
        if (!target_found_exists && target_index != -1)
        {
            targets_array[target_index][0] = xy_array_target[0];
            targets_array[target_index][1] = xy_array_target[1];
            OS = OS + 1;
            // printf("OS is equal to %d\n",OS);
            for (i = 0; i < AMOUNT_OF_TARGETS; ++i)
            {
                send[i * 2] = targets_array[i][0];
                send[i * 2 + 1] = targets_array[i][1];
            }
        }
        wb_emitter_send(communication, send, AMOUNT_OF_TARGETS * 2 * sizeof(short)); //Changed from float to short
    }
    
   

    /////////////////////////////////Main loop/////////////////////////////////
    /////////////////////////////////   :)    /////////////////////////////////
    /////////////////////////////////   :)    /////////////////////////////////
    while (wb_robot_step(TIME_STEP) != -1)
    {
        //printf("levy is %d\n",levy); 
        // Get GPS position */
        const double *position = wb_gps_get_values(gps);
        double x_r = position[0];
        double y_r = position[1];
        /* Run helper functions
         * recognition updates xy_array_target which in turn updates targets_array if a target is found
         * get and receive your targets_array
         */
        recognition(xy_array_target);
        get_receiver_data();
        send_emitter_data();
        
        /* Used to print performance of robot during simulation */
        for (i = 0; i < AMOUNT_OF_TARGETS; i++)
        {
            int x = targets_array[i][0];
            int y = targets_array[i][1];
            int z = targets_array[i][2];
            static int prev_x[AMOUNT_OF_TARGETS] = {0};
            static int prev_y[AMOUNT_OF_TARGETS] = {0};
            static int prev_z[AMOUNT_OF_TARGETS] = {0};
            if (x != prev_x[i] || y != prev_y[i] || z != prev_z[i])
            {
                //printf("Onlooker is [%d], Gamma is [%f], OS is [%d], IS is [%d], target [%d] is at = [%d,%d,%d] \n", onlooker_treshold, gamma_levy, OS, IS, i, x, y, z);
                printf("target [%d] found at = [%d,%d] and visited = [%d] with IS = [%d] \n", i,  x, y, z, IS);

                prev_x[i] = x;
                prev_y[i] = y;
                prev_z[i] = z;
            }
        }
        /* Get the battery percentage of the current robot */
        BS = (wb_robot_battery_sensor_get_value() / 933120) * 100;
     
        // Get time of synchronized simulation */
        double time_robot = wb_robot_get_time();
        double time_since_target = time_robot - last_target_time;
        /* Slowly decline the operation state to represent the performance over half an hour */
        if ((time_since_target > 30*60) && (OS!= 0) && !(time_since_target > 30.05*60))
        {
            OS-=1;
        }
        /* Get the yaw value of  the current robot in radians*/
        const double *ground_truth_attitude = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
        double yaw = ground_truth_attitude[2]; // Is in rad
        /* Make sure the yaw value is always between [0,360], normalize function is found in file APF */
        yaw = normalize_angle(yaw);
         //printf("yaw is %f\n",yaw);
         
        /* Calculate sensor influences on wheels
         * Initialize memory at the beginning of the loop
         */
        memset(wheel_weight_total, 0, sizeof(double) * 4);
        for (i = 0; i < MAX_SENSOR_NUMBER; ++i)
        {
            sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);
            /* If the sensor doesn't see anything, we don't use it for this round */
            if (sensor_value == 0.0)
            {
                speed_modifier = 0.0;
            }
            else
            {
                /* Computes the actual distance to the obstacle, given the value returned by the sensor */
                distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE)); // lookup table inverse.
                /* Here we compute how much this sensor will influence the direction of the robot */
                if (distance < MIN_DISTANCE_SENSOR)
                    speed_modifier = 1 - (distance / MIN_DISTANCE_SENSOR);
                else
                    speed_modifier = 0.0;
            }
            /* Add the modifier for all wheels */
            for (j = 0; j < 4; ++j)
            {
                wheel_weight_total[j] += sensors[i].wheel_weight[j] * speed_modifier;
            }
        }
 
        /* If a robot decides to go to a target, it calculates the closest one
         * It then sets the most suitable target to x_t and y_t
         * TARGET
         */
        double closest_d_target = 1000000; // initialize closest distance to a very large value
        int closest_i_target = -1;         // initialize index of closest target to an invalid value
        for (int i = 0; i < AMOUNT_OF_TARGETS; i++)
        {
            int x_temp_target = targets_array[i][0];
            int y_temp_target = targets_array[i][1];
            int d_temp_target = euclidean_distance(x_r, y_r, x_temp_target, y_temp_target);
            if (d_temp_target < TARGET_RECOGNITION_TRESHOLD && (x_temp_target != 0 && y_temp_target != 0) && d_temp_target < closest_d_target)
            { 
                closest_d_target = d_temp_target;
                closest_i_target = i;
            }
        }
        if (closest_i_target != -1 && targets_array[closest_i_target][2] == 0)
        { // if a suitable target was found
            x_t = targets_array[closest_i_target][0];
            y_t = targets_array[closest_i_target][1];
        }
             
        if (x_t == 0 && y_t == 0)
        {
            distance_target = 0;
        }
        else
        {
            distance_target = euclidean_distance(x_r, y_r, x_t, y_t);
        }
        /* If a robot decides to go to a charging station, it calculates the closest one
         * It then sets the most suitable target to x_t and y_t
         * CHARGE
         */
        double closest_d_home = 1000000;   // initialize closest distance to a very large value
        int closest_j_home = -1;           // initialize index of closest target to an invalid value
        for (int j = 0; j < N_Chargers; j++)
        {
            int x_temp_home = home_array[j][0];
            int y_temp_home = home_array[j][1];
            int d_temp_home = euclidean_distance(x_r, y_r, x_temp_home, y_temp_home);
            if ((x_temp_home != 0 && y_temp_home != 0) && d_temp_home <= closest_d_home)
            { 
                closest_d_home = d_temp_home;
                closest_j_home = j;
            }
        }
        if (closest_j_home != -1)
        { // if a suitable target was found
            x_h = home_array[closest_j_home][0];
            y_h = home_array[closest_j_home][1];
        }
   
        /* Decide if a robot should become an onlooker or just keep scouting
         * First the onlooker_treshold returns a true, to become an onlooker or false to stay scout
         * When a robot is a onlooker and becomes a scout, gamma_levy=GAMMA_MAX to favour global search
         */
        onlooker_treshold = fuzzy_treshold(AMOUNT_OF_TARGETS, OS, IS);
        if (distance_target < TARGET_RECOGNITION_TRESHOLD && distance_target > 5 && distance_target != 0 && targets_array[closest_i_target][2] != 1 && onlooker_treshold && (BS >= BS_TRESHOLD))
        { 
            onlooker = 1;
        }
        else if(!onlooker_treshold || targets_array[closest_i_target][2] == 1 )
        {
            onlooker = 0;
        }
        /* In experienced forager mode (After finding a target),
         * gamma_levy is slowly increased from exploiting to exploring
         * After LSTT is reached the value goes to GAMMA_MAX rather quickly
         * Additionally, if LSTT is reached, onlooker is set to 0 (to become a scout
         * THis often happens in clusteredf areas, thus the robot is reppelled from the cluster
         */
        if (gamma_levy <= GAMMA_MAX-0.00035 && time_since_target < LOCAL_SEARCH_TIME_TRESHOLD && time_since_target != time_robot)
        {
            gamma_levy += 0.0009; 
            if(!onlooker_treshold)
            {
                gamma_levy += 0.009;
            }
            
        }
        else if (gamma_levy < GAMMA_MAX && time_since_target >= LOCAL_SEARCH_TIME_TRESHOLD)
        {
            if(time_since_target < LOCAL_SEARCH_TIME_TRESHOLD+1)
            {
                onlooker = 0;
                gamma_levy = GAMMA_MAX;
 
            }
        }
        // If the distance between the robot and the wall is larger then TRT, wall = 0
    
        /* Calculate the force vector when an obstacle or cluster is recognized*/

        /* STATE MACHINE
         *
         * RESET --> Resets levy and normal distribution numbers and add current time to simulation
         * LEVY --> If no obstacles are found then: Go straight with step from levy distribution,
         * then turn left or right depending on normal distributed number for time
         * that is also dependent on normal distribution
         * GOTO --> Goes to the first target that has been found by the robots
         * HOMING --> If BS < BS_treshold, go home to charg
         * FORWARD --> This state decides if the robot should turn left or right depending on sensory information
         * LEFT --> Turn left
         * RIGHT --> Turn right
         */
        /////////////////////////////////State Machine////////////////////////////////
        /* RESET
         *  Resets time
         *  Returns to LEVY
         *  Sends and receives target arrays
         */
        //printf("state is %d\n",state);
        //printf("levy is [%d], time_robot is [%f] normal is [%f], state is [%d]\n", levy, time_robot, normal, state);     
        switch (state)
        {
        case RESET:
            levy = reset_levy() + time_robot;
            // Number is even
            mu = (double)rand()/ RAND_MAX * (2.0 * M_PI);   // Scale to the desired range
            normal = normalize_angle(reset_normal());
            if (normal>360)
            {
                normal = normal-360;
            }
            state = LEVY; 
            break;
            //printf("RESET\n");
        break;

        /* Levy walker
         * Onlooker = 0
         */
        case LEVY:
            // OBSTACLE AVOIDANCE
            if((BS >= BS_TRESHOLD) && 
            (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD)) 
            {
                    state = AVOID;
                    break;
            }  
            // NO OBSTACLES
            else if (wheel_weight_total[0] == 0 || wheel_weight_total[1] == 0)    
            {
                // TARGET RECOGNITION
                if (onlooker && (x_t != 0 && y_t != 0) && (BS >= BS_TRESHOLD))
                {
                    state = GOTO;
                    break;
                }
                //RESETTING STEP GENERATOR
                if (!onlooker && (BS >= BS_TRESHOLD) && 
                (time_robot >= levy + abs(normal)))
                {
                    state = RESET;
                    break;
                }
                // BATTERY MONITERING
                else if (BS < BS_TRESHOLD)
                {
                    state = HOMING;
                    break;
                }  
                // FORWARD STEP
                else if (!onlooker && (BS >= BS_TRESHOLD) && (time_robot < levy))
                {     
                    get_receiver_data();
                    send_emitter_data();  
                    speed[0] = MAX_SPEED;
                    speed[1] = MAX_SPEED;
                    speed[2] = MAX_SPEED;
                    speed[3] = MAX_SPEED;
                }
                // NORMAL ROTATION
                else if ((!onlooker && (BS >= BS_TRESHOLD) && (time_robot > levy)) || (!onlooker && (BS >= BS_TRESHOLD) && (time_robot < levy + 2)))
                {
                //printf("normal is %f, yaw is %f, normal-yaw is %d\n", normal, yaw, abs(normal- yaw));
                    if (normal > normalize_angle(mu) && abs(normal- yaw) > 20)
                    {
                        speed[0] = 0.7 * MAX_SPEED;
                        speed[1] = -0.7 * MAX_SPEED;
                        speed[2] = 0.7 * MAX_SPEED;
                        speed[3] = -0.7 * MAX_SPEED;
                    }
                    else if (normal <= normalize_angle((mu)) && abs(normal- yaw) > 20)
                    {
                        speed[0] = -0.7 * MAX_SPEED;
                        speed[1] = 0.7 * MAX_SPEED;
                        speed[2] = -0.7 * MAX_SPEED;
                        speed[3] = 0.7 * MAX_SPEED;
                    }
                    else if(abs(normal-yaw) < 20 || (time_robot > levy + 5))
                    {
                        state = RESET;
                    break;
                    }
                }   
            }
        break;
        /* GOTO
        * We have a goal to go towards, how do we deal with obstacles?
        */
        case GOTO:
            if (!(onlooker))
            {
                onlooker = 0;
                levy = time_robot;
                last_target_time = time_robot;
                set_gamma();
                state = RESET;
                break;
            }
            else if (onlooker && targets_array[closest_i_target][2] != 1)
            {   
                // BATTERY MONITORING
                if (BS < BS_TRESHOLD) 
                {
                    state = HOMING;
                    break;
                }    
                // OBSTACLE AVOIDANCE
                if (((wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD))) {
                    state = AVOID;
                    break;
                }
                // Nothing to repel, so just go towards the target
                if ((wheel_weight_total[0] < WHEEL_WEIGHT_THRESHOLD) && (wheel_weight_total[1] < WHEEL_WEIGHT_THRESHOLD) && distance_target > 0)
                {
                     get_receiver_data();
                     send_emitter_data();
                     Force_vector(MIN_POTENTIAL_DISTANCE, x_r, y_r, x_t, y_t);
                     calculateWheelSpeeds(attraction_angle_atr, yaw, speed);
                     if (distance_target < 5)
                     {
                         set_gamma();
                         last_target_time = time_robot;
                         targets_array[closest_i_target][2] = 1;
                         onlooker = 0;
                         levy = time_robot;
                         state = RESET;
                         break;
                     }  
                }  
           }
        break;
        /* HOMING
        * Onlooker = 1
        */      
        case HOMING:
            if (!home_reached)
            {
                Force_vector(MIN_POTENTIAL_DISTANCE, x_r, y_r, x_h, y_h);
                calculateWheelSpeeds(attraction_angle_atr, yaw, speed);
            }
            else if (!(BS = 100))
            {
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = 0;
                speed[3] = 0;
            }
            else 
            {
                levy = time_robot;
                last_target_time = time_robot;
                state = RESET;
            }
        break;
        /* AVOID
         * Decide to turn LEFT or RIGHT
         */
        case AVOID:            
   
            /* Left sensors are over treshold --> turn left */
            if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD)
            {
                state = LEFT;          
                break;         
            }
            /* Right sensors are over treshold --> turn right */
            else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD)
            {
                state = RIGHT;
                break;
            }
            /* Go back to LEVY walking */
            else
            {
                state = LEVY;
                break;
            }
        break;
        /* LEFT
         * When the robot has started turning, it will go on in the same direction until no more obstacle.
         * This will prevent the robot from being caught in a loop going left, then right, and so on.
         */
        case LEFT:
            if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD)
            {
                speed[0] = 0.7 * MAX_SPEED;
                speed[1] = -0.7 * MAX_SPEED;
                speed[2] = 0.7 * MAX_SPEED;
                speed[3] = -0.7 * MAX_SPEED;
            }
            else if (wheel_weight_total[0] == 0)
            {
                state = LEVY;
                break;
            }
        break;
            /* RIGHT
             * When the robot has started turning, it will go on in the same direction until no more obstacle.
             * This will prevent the robot from being caught in a loop going left, then right, and so on.
             */
        case RIGHT:
            if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD)
            {   
                speed[0] = -0.7 * MAX_SPEED;
                speed[1] = 0.7 * MAX_SPEED;
                speed[2] = -0.7 * MAX_SPEED;
                speed[3] = 0.7 * MAX_SPEED;
            }
            else if (wheel_weight_total[1] == 0)
            {
                state = LEVY;
                break;
            }
        break;
        /* Default state of state machine */
        default:
            state = LEVY;
        }
        /* Init motors velocity from obstacles seen */
        wb_motor_set_velocity(front_left_wheel, speed[0]);
        wb_motor_set_velocity(front_right_wheel, speed[1]);
        wb_motor_set_velocity(back_left_wheel, speed[2]);
        wb_motor_set_velocity(back_right_wheel, speed[3]);

        if ((BS < 3 && (!has_printed)) || ((wb_robot_get_time() > 2*3600 + 10*60) && (!has_printed)))
        //if (((wb_robot_get_time() > 2*3600 + 5*60) && (!has_printed)))
        //if( (((BS < 3) && (wb_robot_get_time() > (2*3600+ 10*60))) && (!has_printed) )|| ((wb_robot_get_time() > (2*3600+ 10*60))&& (!has_printed))  )
        {
            for (i = 0; i < AMOUNT_OF_TARGETS; i++)
            {
                printf("target [%d] is at = %d,%d,%d \n", i, targets_array[i][0], targets_array[i][1], targets_array[i][2]);
            }
            char filename[40];
            time_t current_time = time(NULL);
            struct tm *local_time = localtime(&current_time);                                   // Convert to local time
            strftime(filename, sizeof(filename), "targets_data_%Y%m%d_%H%M%S.txt", local_time); // Format the timestamp and append to filename
            // sprintf(filename, "targets_data_%d.txt", CUSTOM_SEED);
            save_targets_data(targets_array, filename);                                         // Save the data to the new filename
            has_printed = true;
        }
    }
    wb_robot_cleanup();
    return 0;
}
