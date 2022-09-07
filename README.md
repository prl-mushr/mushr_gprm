# Global Planner
[![build-test](https://github.com/prl-mushr/mushr_gprm/actions/workflows/ci.yml/badge.svg?branch=release)](https://github.com/prl-mushr/mushr_gprm/actions)

This ROS package hosts a lightweight global planner for the MuSHR system. It is intended to be runnable on any MuSHR car, along with the rest of the MuSHR stack. It uses a lazy A* planner implementation. Search based methods have two parts - first, the planning problem and environment must be represented by a graph, and then the graph must be searched from a starting point to a goal. For this implementation, we use a Roadmap (see roadmap.py) to represent an environment on which we can find valid [Dubins paths](https://en.wikipedia.org/wiki/Dubins_path) (see dubins.py). We use a sampler (samplers.py) and a problem definition (problems.py) to help construct and reason about this roadmap. Within search.py, we use the Networkx A* solver to find a shortest path through our roadmap. Additionally, we optimize our path with shortcutting the final path we have if possible.

## Tutorial
The following [tutorial](https://mushr.io/tutorials/navigation/) goes through installing/running the car.

## Installing on the car
Make sure you install our fork of networkx onto your system (our fork has a tiny modification which determines if there is no solution more efficiently, it is a pending pull request). If you already have networkx installed, you should uninstall it first and reinstall the forked version. Note if you do not have it installed, skip the uninstall step:
```
pip uninstall networkx
pip install git+https://github.com/brianhou/networkx.git
```

Clone [this](https://github.com/prl-mushr/mushr_gprm) repo into `~/catkin_ws/src`

If you're on our docker image, you should be good to go! If not, make sure you have these common ROS dependencies: `tf2_ros`, `std_msg`, `nav_msg`, `visualization_msg`, and `geometry_msg`. Also, make sure you have the Python libraries `matplotlib` and `numpy` as well.

## `mushr_gprm` ROS API

#### Publishers
Topic | Type | Description
------|------|------------
`/path_topic`|[geometry_msgs/Path](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html)|The trajectory computed by the planner.

#### Subscribers
Topic | Type | Description
------|------|------------
`/map`|[nav_msgs/OccupancyGrid](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)|Uses the provided occupancy grid as the graph for planning.
`/goal_topic`|[geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)|Goal to compute path to.
`/start_topic`|[geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)|Starting location of the path being computed.