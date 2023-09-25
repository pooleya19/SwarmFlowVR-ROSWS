# SwarmFlowVR - ROS Workspace

To configure SwarmFlowVR for a given swarm of ROSbots:
1. Update [rosbot_swarm_control.launch](/src/adam_thesis/launch/rosbot_swarm_control.launch) with the correct IP address for the mocap server and with the names of the ROSbots in your swarm.
2. Open [ROSBotSwarm.py](/src/adam_thesis/scripts/ROSBotSwarm.py) and update the list of names of ROSbots on line 38.

To run SwarmFlowVR:
1. Open a terminal and in this repository's root workspace and type:
```
source devel/setup.bash

roslaunch /src/adam_thesis/launch/rosbot_swarm_control.launch
```
2. For each ROSbot in the swarm, open a terminal in this repository's root workspace and type:
```
./startRobot.sh [ROSbot name]
```

## High-Level Description
- Each ROSbot is managed by a [waypoint handler](/src/adam_thesis/scripts/ROSBotWaypointHandler.py). The waypoint handler's job is to give the ROSbot velocity commands to move the ROSbot from its current position/orientation to the target waypoint position.
- The waypoint handler receives target waypoint positions from the [VR counter-part](https://github.com/pooleya19/SwarmFlowVR-Unity) of SwarmFlowVR.
- The waypoint handler receives position/orientation from the mocap server.
- The [ROSBotSwarm.py](/src/adam_thesis/scripts/ROSBotSwarm.py) script: 
    - Runs all ROSbot waypoint handlers,
    - Stops all ROSbots if any ROSbots get too close to each other,
    - Displays the published battery voltage of each ROSbot.
