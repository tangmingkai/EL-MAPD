# Acknowledgments
This repo is based on [MAPF-LRR2023](https://github.com/DiligentPanda/MAPF-LRR2023). Thanks for the great project.

# Dependency
The dependency includes:

* Boost
* spdlog
* OpenMP

You can install them by

`
sudo apt install libboost-all-dev libomp-dev libspdlog-dev
`

# Compile
Run the bash

```
./compile.sh
```

# Execute
The executable is generated in `./build/main`

```
./build/elmapd -i <input_file> -o <output_file>  --simulationTime <simulation_time> --planningTimeLimit <planning_time_limit> --rechargeThresholdFactor <recharge_threshold_factor>  --energyWeight <energy_weight> --lnsMode <lns_mode> ",
```
The parameters:
+ **input_file**: The path of the input file.
+ **output_file**: The path of the output file.
+ **simulation_time**: The number of time steps for simulation.
+ **planning_time_limit**: The planning time limit for each planning episode.
+ **energy_weight**: The weight in the cost function.
+ **lns_mode**: The mode of the ALNS. 
  +  -1: without use lns. 
  +  1: with lns

For example:
```
./build/elmapd -i ../dataset/kiva_100_0.json -o output.json --simulationTime 5000 --planningTimeLimit 0.5 --rechargeThresholdFactor 0.05  --energyWeight 0.0 --lnsMode 1
```

## Input File
The input file is a json file, named "xxx_yyy.json". "xxx" is the graph name where "kiva" is "G1", "warehouse" is "G2", and "sortation" is "G3" in the paper. "yyy" is the number of agents.

In the input file, there are some parameters:
  + **mapFile**: The path of the map, describing the grid-like graphs
  + **agentFile**:  The path of the agent file, describing the agents' initiliza states.
  + **teamSize**: The number of agents
  + **taskFile**: The path of the task file, describing the task information, where the first number is the pickup location and the second number is the delivery location. Let loc denote the number, cols denote the number of columns. The coordiation is (loc % cols, loc / cols)
  + **idleComsumption**: The energy consumption per second for the action that do not alternate the vertex and the orientation.
  + **activeUnloadedComsumption**: The energy consumption per second for the action that alternate the vertex or the orientation with an unloaded agent.
  + **activeLoadedComsumption**: The energy consumption per second for the action that alternate the vertex or the orientation with an loaded agent.
  + **activeLoadedComsumption**: The energy inreasement per second for the recharge action per second.
  + **fullEnergy**: The energy upper bound for the agent.
  + **pIdle**: The probality that the task assigner doesn't assign a task to an agent in the time step that is after 

**Note**: In the input json file, the `idleComsumption`, `activeUnloadedComsumption`, `activeLoadedComsumption`,
`rechargeEnergy` are the energy consumption or recharge energy quality **per second**. In the driver.cpp, after we read the value, we direct multiple their value with `planning_time_limit` to transform the value to those **per time step**.

## Task File
The first line is the number of task.

In the second line to the last line, there are 2 numbers per line. 
+ The first number is the pickup location 
+ The second number is the delivery location. 

Let `loc` denote the number, `cols` denote the number of columns. The coordiation is `(loc % cols, loc / cols)`.

## Agent File
The first line is the number of agents.

In the second line to the last line, there are 3 numbers per line. 
+ The first number is the agent start location.
+ The second number is the agent start orientation.
+ The third number is the agent initial energy.

Let `loc` denote the number, `cols` denote the number of columns. The coordiation is `(loc % cols, loc / cols)`.
The oritations:
+  0: East
+  1: South
+  2: West
+  3: North

```
      N(3)
       |
 W(2)-----E(0)
       |
      S(1)

 .------------>x
 |
 |
 |
 |
 v
 y
```

## Map File
From the fifth line to the last line:
+ '@': Obstacle
+ 'C': Recharge point
+ 'E': Endpoints where agents interact with human workers.
+ 'S': Endpoints where agents interact with adjacent shelves. 
+ '.': Other vertex.
   
Both 'E' and 'S' can be selected as a pickup vertex and a delivery for a task.

## Output File
+ **start**: The initial state of each agent, which is denoted by `[x, y, orientation, initial energy]`
+ **path**： The action sequence of each agent.
  + C: Rotate Clockwise
  + R: Rorate Counterclockwise
  + F: Move Forward
  + W: Wait
  + P: Pickup
  + D: Delivery
  + E: Recharge
+ **energy**: The number in a line shows the energy quantity for each time step of an agent.
+ **PlannerTimes**: The planning time for each planning episode
+ **schedule**: The number in a line shows the task assign time of an agent, which is denoted by `[start_timestep: task_id]`.
+ **agentGoalInfo**: Each item represents the event that the remainning goal of an agent is changed, e.g. when agent perform a pickup or delivery action or has a new path plan to a new task, which is denoted by `[time_step, agent_id, [goal_list]]`.
  + When `[goal_list].size()=6`, the goal list is `[pickup_x, pickup_y, delivery_x, delivery_y, recharge_x, recharge_y]`
  + When `[goal_list].size()=4`, the goal list is `[delivery_x, delivery_y, recharge_x, recharge_y]`
  + When `[goal_list].size()=2`, the goal list is `[recharge_x, recharge_y]`
