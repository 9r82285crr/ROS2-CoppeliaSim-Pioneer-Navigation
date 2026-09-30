# ROS 2 + CoppeliaSim Pioneer Grid Navigation

A lightweight mobile robot navigation project developed with **ROS 2
Humble** and **CoppeliaSim**. The project uses a simulated Pioneer P3DX
mobile robot to test odometry-based waypoint navigation, laser obstacle
detection, four-direction grid motion, and BFS-based obstacle
replanning.

> **Project status:** Work in progress. The current focus is reliable
> navigation and obstacle avoidance. Stable map generation, SLAM, and
> Nav2 integration are not implemented.

## Repository Structure

``` text
ROS2-CoppeliaSim-Pioneer-Navigation/
├── README.md
├── .gitignore
├──way_p.py
├──scan_judge.py
├──<test_scene>.ttt
```

### `src/way_p.py`

The main ROS 2 navigation program.

The current implementation integrates the principal navigation functions
in a single Python node, including:

-   Pioneer mobile robot velocity control through `/cmd_vel`
-   Robot pose acquisition from odometry
-   LaserScan processing
-   Forward obstacle detection
-   Predefined waypoint navigation
-   Four-direction grid-based movement
-   Approximately 90-degree turns between grid directions
-   Experimental grid representation
-   Blocked-cell recording
-   BFS-based path planning and replanning

The controller is designed around a simple state sequence:

``` text
PLAN_NEXT
    ↓
ALIGN_TO_STEP
    ↓
MOVE_TO_STEP
    ↓
PLAN_NEXT
```

When the robot detects an obstacle while moving toward the next grid
cell, it stops, treats the attempted cell as blocked, and runs the
planner again to find an alternative route.

### `src/scan_judge.py`

An **experimental laser-scan test program** used during development to
inspect and evaluate the simulated laser measurements.

This file is not considered a stable navigation component. It is
retained because it is useful for testing the forward sensing region,
scan geometry, distance-selection methods, and false obstacle
detections.

### `simulation/<test_scene>.ttt`

The CoppeliaSim test scene containing the Pioneer robot and the current
experimental environment.

The simulation-side robot configuration and scripts are currently
contained in the `.ttt` scene rather than maintained as separate `.lua`
files.

## Software Environment

The project has been developed with:

-   Ubuntu 22.04
-   ROS 2 Humble
-   CoppeliaSim 4.10.0
-   Python 3
-   RViz2
-   Pioneer P3DX differential-drive robot

## ROS 2 Interface

The current system primarily uses:

  Topic        Purpose
  ------------ --------------------------------------------
  `/cmd_vel`   Robot linear and angular velocity commands
  `/odom`      Robot odometry
  `/scan`      Simulated laser range measurements
  `/tf`        Coordinate transforms

The current TF structure is based on:

``` text
odom
 └── base_link
      └── laser_frame
```

Because SLAM is not currently used, `odom` is the appropriate reference
frame for the present navigation tests.

## Navigation Method

### Four-Direction Grid Motion

The robot is constrained to four principal movement directions:

``` text
          +Y
           ↑
           │
     -X ← Robot → +X
           │
           ↓
          -Y
```

Diagonal grid motion is not used. Direction changes are therefore
performed as approximately ±90° rotations.

The current grid resolution is:

``` text
0.20 m/cell
```

This simplified motion model was selected to make obstacle avoidance and
route planning more predictable in the test environment.

### BFS Path Planning

The current navigation program uses **Breadth-First Search (BFS)** over
four-connected grid cells.

The four candidate neighbors are:

``` text
(x + 1, y)
(x - 1, y)
(x, y + 1)
(x, y - 1)
```

Earlier local/greedy neighbor selection could cause the robot to
alternate between adjacent cells near an obstacle. BFS was introduced so
that the robot can search for a connected route toward the target
instead of selecting only the locally closest neighboring cell.

### Obstacle Handling

The current obstacle-avoidance sequence is:

``` text
Move toward next grid cell
          ↓
Check forward laser sector
          ↓
     Obstacle detected?
       /           \
     No             Yes
     │               ↓
 Continue           Stop
                     ↓
              Mark attempted
              cell as blocked
                     ↓
                 Run BFS
                     ↓
              Select new cell
                     ↓
                  Continue
```

The current experiments mainly use a narrow forward laser sector because
this region has provided the most useful obstacle-distance information
in the simulated sensor setup.

Typical experimental parameters are:

``` text
Forward sector: approximately ±15°
Stop threshold: 0.38 m
Resume threshold: 0.39 m
```

The exact front-distance selection method is still being tested because
occasional false obstacle detections remain possible.

## Current Project Status

### Implemented

-   Pioneer robot simulation and ROS 2 control
-   `/cmd_vel` motion commands
-   Odometry-based robot pose
-   `/scan` laser data acquisition
-   TF connection between robot and laser frames
-   RViz verification of scan data
-   Waypoint navigation
-   Four-direction grid motion
-   Approximately 90-degree turning
-   Forward obstacle stopping
-   Grid-based blocked-cell representation
-   BFS-based obstacle replanning

### Experimental / Under Development

-   Robust front-distance estimation
-   Reduction of false laser obstacle detections
-   Grid occupancy representation
-   Persistent map generation
-   Map visualization

### Not Implemented

-   Stable autonomous map generation
-   SLAM
-   Global localization
-   ROS 2 Navigation2 integration
-   A\* path planning
-   Standalone `.lua` script package

## Running the Current Project

First source ROS 2 Humble:

``` bash
source /opt/ros/humble/setup.bash
```

Open the `.ttt` test scene in CoppeliaSim and start the simulation.

Check that the required ROS 2 topics are available:

``` bash
ros2 topic list
```

Useful checks include:

``` bash
ros2 topic hz /scan
ros2 topic echo /odom
```

For visualization, start RViz2:

``` bash
rviz2
```

For the current odometry-based setup, use:

``` text
Fixed Frame: odom
```

Then run the main navigation program from the repository root:

``` bash
python3 src/way_p.py
```

The experimental scan test can be run separately when required:

``` bash
python3 src/scan_judge.py
```

> The exact startup sequence may depend on the scripts embedded in the
> supplied CoppeliaSim scene.

## Known Limitations

The current project is an experimental navigation implementation rather
than a complete autonomous navigation stack.

The simulated laser configuration is still being tuned. In particular,
the scan may occasionally produce measurements that cause false
forward-obstacle detections.

Localization currently depends on odometry, so position and heading
errors can accumulate during longer runs.

The internal grid is currently used primarily to support navigation and
BFS replanning. A stable persistent occupancy-map generation pipeline
has not yet been completed.

The project intentionally uses a simple four-direction motion model. It
does not currently perform arbitrary-angle trajectory optimization or
continuous path smoothing.

## Planned Development

The next engineering priorities are:

1.  Improve the reliability of forward obstacle detection.
2.  Validate the current BFS navigation over more obstacle
    configurations.
3.  Complete occupancy-grid/map generation and visualization.
4.  Improve simulated laser processing if required.
5.  Evaluate more advanced planning or localization only after the
    current navigation pipeline is stable.

## Notes

This repository reflects the **current working state of the project**.
Experimental or unfinished functionality is identified explicitly rather
than presented as a completed feature.
