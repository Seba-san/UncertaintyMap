# Instructions 
This document provides step-by-step instructions for building and running the Docker container, launching a complete simulation, and executing specific parts of the system. It also details how to configure simulation parameters.

### Build the Docker Container
To build the Docker container, run the following command in your terminal:

```bash
docker build -t oenuf_active_slam -f Dockerfile . 
```
### Run the Docker Container
To run the Docker container, navigate to the **kf_slam** directory and execute the provided script:
```bash
cd kf_slam 
./run.sh oenuf_active_slam 
```
### Launch a Complete Simulation
The following code loads data from [simulaciones_planificadas.yaml](./kf_slam/simulaciones_planificadas.yaml), overwrites [parameters.yaml](./kf_slam/parameters.yaml), and launches all ROS nodes necessary for the simulation:

```bash
roscd pioneer2dx/scripts/kf_slam/
python master_simulations.py
```

### Enter the Docker Container in Another Terminal
If you need to enter the Docker container through another terminal window, use the following commands:

```bash
cd kf_slam 
./entrar.sh
```

### Launch the System by Parts
To launch specific components of the system, follow these steps:
- **Prepare the Simulation**: Load data from *parameters.yaml* and overwrite documents in the project:
```bash
roscd pioneer2dx/scripts/kf_slam/
python prepare_simulation.py
```
- **Launch Environment, Controllers, and Robot**: 
Start the ROS launch file to initialize the environment and the robot:
```bash 
roslaunch test.launch 
```
- **Waypoint Manager**: Execute the waypoint manager script to manage waypoints:
```bash 
python way_point_manager.py
```
-**Active SLAM Core with RRT Algorithm**:
 Run the script implementing the modified RRT algorithm for active SLAM:
 ```bash 
 python active_slam_core_rrt.py 
 ```
### Close Docker container:
```bash 
cd kf_slam 
./cerrar_docker.sh 
```

### Configuration Parameters
The *parameters.yaml* file allows you to customize various simulation parameters. Here is an overview of some key parameters you can adjust:

  -  **sigma_max, fov, cell_size**: Set maximum uncertainty, field of view, and cell size respectively.
  -  **initial_pose**: Define the starting position of the robot.
  -  **Q_noise, R_noise, P_0**: Configure noise and initial uncertainty parameters of Kalman Filter.
  -  **map_size**: Set the dimensions of the map.
  -  **uf_threshold**: Set the uncertainty threshold.
  -  Additional parameters for obstacle avoidance, previous map loading, RRT* algorithm iterations, and map names.

### Planned Simulations
To configure planned simulations, edit the *simulaciones_planificadas.yaml* file. This file contains simulation setups including initial poses, planner types, UF implementation, and other parameters. For example:

```yaml
- {x: -12.0, y: -8.0, planner: simple, mpc: False, UF: False, sigma_max: 1.0, map: galpon_1}
- {x: -12.0, y: +8.0, planner: simple, mpc: False, UF: False, sigma_max: 1.0, map: galpon_1}
- {x: -12.0, y: +8.0, planner: simple, mpc: False, UF: False, sigma_max: 0.6, map: galpon_0}
```



