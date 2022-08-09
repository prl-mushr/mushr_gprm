from __future__ import print_function

import networkx as nx
import numpy as np
import rospy
import tf2_ros
import time

from geometry_msgs.msg import PoseStamped, PoseArray, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header

import mushr_gprm.utils as utils
import mushr_gprm.search as search
from mushr_gprm.problems import SE2Problem
from mushr_gprm.roadmap import Roadmap
import mushr_gprm.samplers as samplers
from mushr_gprm.samplers import sampler_selection


class PlannerROS:
    def __init__(
        self,
        num_vertices,
        connection_radius,
        curvature,
        do_shortcut=True,
        sampler_type="random",
        tf_prefix="",
        tf_listener=None,
    ):
        """Motion planning ROS wrapper.

        Args:
            num_vertices: number of vertices in the graph
            connection_radius: radius for connecting vertices
            curvature: curvature for Dubins paths
            do_shortcut: whether to shortcut the planned path
            sampler_type: name of the sampler to construct
        """
        self.tf_prefix = tf_prefix

        if tf_listener:
            self.tl = tf_listener
        else:
            self.tl = tf2_ros.TransformListener(tf2_ros.Buffer())

        self.num_vertices = num_vertices
        self.connection_radius = connection_radius
        self.do_shortcut = do_shortcut

        self.map_sub = rospy.Subscriber(
            rospy.get_param("~map"), OccupancyGrid, self.get_map
        )
        
        self.permissible_region = None
        self.map_info = None
        self.rm = None
        self.car_pose = None
        self.curvature = curvature
        self.sampler_type = sampler_type

        self.goal_sub = rospy.Subscriber(
            rospy.get_param("~goal_topic"), PoseStamped, self.get_goal
        )

        self.start_sub = rospy.Subscriber(
            rospy.get_param("~start_topic"), PoseStamped, self.get_start
        )

        self.path_pub = rospy.Publisher(
            rospy.get_param("~path_topic"), Path, queue_size=1
        )
        rospy.loginfo("Complete planner initialization.")

    def get_map(self, msg):
        self.permissible_region, self.map_info = utils.get_map(msg)
        self.problem = SE2Problem(
            self.permissible_region,
            map_info=self.map_info,
            check_resolution=0.1,
            curvature=self.curvature,
        )
        # Construct a roadmap
        sampler = sampler_selection[self.sampler_type](self.problem.extents)
        rospy.loginfo("Constructing roadmap...")
        saveto = None
        start_stamp = time.time()
        self.rm = Roadmap(
            self.problem,
            sampler,
            self.num_vertices,
            self.connection_radius,
            lazy=True,
            saveto=saveto,
        )
        load_time = time.time() - start_stamp
        rospy.loginfo("Roadmap constructed in {:2.2f}s".format(load_time))
    
    def plan_to_goal(self, start, goal):
        """Return a planned path from start to goal."""
        # Add current pose and goal to the planning env
        rospy.loginfo("Adding start and goal node")
        try:
            start_id = self.rm.add_node(start, is_start=True)
            goal_id = self.rm.add_node(goal, is_start=False)
        except ValueError:
            rospy.loginfo("Either start or goal was in collision")
            return None

        # Call the Lazy A* planner
        try:
            rospy.loginfo("Planning...")
            start_edges_evaluated = self.rm.edges_evaluated
            start_time = time.time()
            path, _ = search.astar(self.rm, start_id, goal_id)
            end_time = time.time()
            edges_evaluated = self.rm.edges_evaluated - start_edges_evaluated
            rospy.loginfo("Path length: {}".format(self.rm.compute_path_length(path)))
            rospy.loginfo("Planning time: {}".format(end_time - start_time))
            rospy.loginfo("Edges evaluated: {}".format(edges_evaluated))
        except nx.NetworkXNoPath:
        # except Exception:
            rospy.loginfo("Failed to find a plan")
            return None

        # Shortcut if necessary
        if self.do_shortcut:
            rospy.loginfo("Shortcutting path...")
            start_time = time.time()
            path = search.shortcut(self.rm, path)
            end_time = time.time()
            rospy.loginfo(
                "Shortcut length: {}".format(self.rm.compute_path_length(path))
            )
            rospy.loginfo("Shortcut time: {}".format(end_time - start_time))

        # Return interpolated path
        return self.rm.compute_qpath(path)

    def get_start(self, msg):
        """Car pose callback function."""
        self.car_pose = np.array(utils.pose_to_particle(msg.pose))

    def get_goal(self, msg):
        """Goal callback function."""
        if self.rm is None or self.car_pose is None:
            return False

        self.goal = np.array(utils.pose_to_particle(msg.pose))

        path_states = self.plan_to_goal(self.car_pose, self.goal)
        if path_states is None:
            return False
        return self.send_path(path_states)

    def send_path(self, path_states):
        """Send a planned path to the controller."""
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "map"
        desired_speed = 1.0

        path = Path()
        path.header = h
        path.poses = [
            PoseStamped(h, utils.particle_to_pose(state)) for state in path_states
        ]
        self.path_pub.publish(path)
