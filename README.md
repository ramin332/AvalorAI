The code in this repo is written and is compatible with Webots from Cyberrobotics.

The simulation mainly shows the following features and functions: 
- Decentralized control with limited communication in an unknown environment.
- Multi-target search and inter-robot decision making regarding these targets using fuzzy systems. (depreciated)
- Local coordinate system manipulations for movement and target recognition.
- Distance sensors simulations for obstacle avoidance using Braitenberg obstacle avoidance.
- Movement towards targets using Artificial Potential Fields while avoiding obstacles.

The controller folder has two controllers
- MBSO > This is the folder that contains the code that is being run on every single robot seperatly.
  - APF.c (Artifical Potential Function > Attraction towards targets works, repulsion is depreciated).
  - fuzzy_threshold.c > IS is set on 3 > Operational State (OS), Battery State (BS) and Security State (SS) are depreciated.
  - levy_distribution.c > Used to randomize the step size of the robots > Most optimal random search in an unkown environments (similar to what animals do).
  - normal_distribution.c > Normal distribution to randomize rotations.
  - obstacle_avoidance.c > Braitenberg obstacle avoidance for local and simple avoidance.
  - recognition.c > Uses camera, gps and inertial unit to identify and map the location of targest.
  - save.c > Used to save the target list of every robot after they run out of battery or the simulation is paused.
- Supervisor > This is necessary to randomize the robots and set the battery level for every new run.


Other interesting files can be found in:
- Protos> Pioneer3at.proto > The robot is defined here. Add-ons are:
  -  Pen
  -  Receiver
  -  Emitter
  -  Inertial unit
  -  Camera
  -  GPS
  -  Distacnce sensors
  -  Wheels and motors
