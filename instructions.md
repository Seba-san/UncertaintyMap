# Instructions 
### Build docker container
`docker build -t oenuf_active_slam -f Dockerfile . `
### Run docker container

`cd kf_slam `
`./run.sh oenuf_active_slam `

### Launch a complete simulation
This code load data from "simulaciones_planificadas.yaml", overwrite "parameters.yaml" and launch all ROS nodes.
` roscd pioneer2dx/scripts/kf_slam/ & python master_simulations.py`

### To enter  in a docker container in other terminal
`cd kf_slam &./entrar.sh `

### To launch the system by parts do
`roscd pioneer2dx/scripts/kf_slam/`
load data from parameters.yaml and overwrite documents in the project
`python prepare_simulation.py `
 # Launch the environment, controllers and robot 
`roslaunch test.launch `
it is the way point manager
`python way_point_manager.py  `
implements the modified RRT algorithm
`python active_slam_core_rrt.py  `


### Configuration parameters of "parameters.yaml" file
```
planner: simple  # double or simple
distance_branch: [0, 4.0, -4.0] #m

sigma_max: 1.0 #m 0.6
fov: 5.0 #m
cell_size: 0.1 #m
max_distance_landmark: 1.0 #m max distance to assign points to landmarks. A low parameter, the slam system create a lot of landmarks. A high parameter, the slam system has bad asociations
initial_pose:
  x: -12.0  #m -12 galpon , 3.5 corridor
  y: -8.0     #m +-7.5 galpon, 1.0 corridor
Q_noise: 0.01 #m^2
R_noise: 0.01 #m^2
P_0: 0.01 #m^2
map_size: [60.0, 60.0] #m x,y
UF: false # implements uncertainty frontiers
uf_treshold: 0.3 #m uncertainty threshold
min_obstacle_distance_sdf: 0.8 #m minimun distance to obstacles to avoid.
min_distance_obstacle_avoidance: 0.85 #m 0.7 minimun distance to obstacles to avoid until obstacle avoidance algorithm starts to work. If it are working in corridors, set this parameter at 0.1
#bbox: [[[0,0],[60,60],0.1,0.1],[[20,20],[40,40],0.6,0.5]] # bounding box for explored area in metters. [bbox1, bbox2,...], con bboxi=[sup izq, inf der, sigma,occupado], sup_izq=[x,y] 
bbox: []
previous_map_flag: false # load a previous situation stored in  previous_map.mat
max_iter_rrt: 2000 # maximun iterations for the rrt* algorithm.
map: galpon_1   #name map, it will be used as direction folder in future implementations.
```
### Planified simulations from "simulaciones_planificadas.yaml" file
[initial pose], [planner], type of planner, UF implementation, sigma_max,  map name
` - {x: -12.0, y: -8.0,   planner: simple, mpc: False, UF: False, sigma_max: 1.0, map: galpon_1}  `





