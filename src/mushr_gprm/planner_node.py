"""
Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
License: BSD 3-Clause. See LICENSE.md file in root directory.
"""

import errno
import networkx as nx
import numpy as np
import os
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
        cache_roadmap=False,
        tf_prefix="",
        tf_listener=None,
        run_visualizations=False,
    ):
        """Motion planning ROS wrapper.

        Args:
            num_vertices: number of vertices in the graph
            connection_radius: radius for connecting vertices
            curvature: curvature for Dubins paths
            do_shortcut: whether to shortcut the planned path
            sampler_type: name of the sampler to construct
            cache_roadmap: whether to cache the constructed roadmap
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
        self.cache_roadmap = cache_roadmap
        self.run_visualizations = run_visualizations

        self.goal_sub = rospy.Subscriber(
            rospy.get_param("~goal_topic"), PoseStamped, self.get_goal
        )

        self.start_sub = rospy.Subscriber(
            rospy.get_param("~start_topic"), PoseStamped, self.get_start
        )

        self.path_pub = rospy.Publisher(
            rospy.get_param("~path_topic"), Path, queue_size=1
        )

        if self.run_visualizations:
            self.nodes_viz = rospy.Publisher(
                "~vertices", PoseArray, queue_size=1, latch=True
            )
            self.edges_viz = rospy.Publisher("~edges", Marker, queue_size=1, latch=True)

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
        if self.cache_roadmap:
            saveto = graph_location(
                "se2",
                self.sampler_type,
                self.num_vertices,
                self.connection_radius,
                self.curvature
            )
            rospy.loginfo("Cached at: {}".format(saveto))
        start_stamp = time.time()
        self.rm = Roadmap(
            self.problem,
            sampler,
            self.num_vertices,
            self.connection_radius,
            saveto=saveto,
        )
        load_time = time.time() - start_stamp
        rospy.loginfo("Roadmap constructed in {:2.2f}s".format(load_time))
        if self.run_visualizations:
            rospy.Timer(rospy.Duration(1), lambda x: self.visualize(), oneshot=True)
            self.rm.visualize(saveto="graph.png")
    
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
        if self.run_visualizations:
            self.rm.visualize(saveto="graph.png")

        # Call the Lazy A* planner
        try:
            rospy.loginfo("Planning...")
            start_edges_evaluated = self.rm.edges_evaluated
            start_time = time.time()
            path = search.astar(self.rm, start_id, goal_id)
            end_time = time.time()
            edges_evaluated = self.rm.edges_evaluated - start_edges_evaluated
            rospy.loginfo("Path length: {}".format(self.rm.compute_path_length(path)))
            rospy.loginfo("Planning time: {}".format(end_time - start_time))
            rospy.loginfo("Edges evaluated: {}".format(edges_evaluated))
            if self.run_visualizations:
                self.rm.visualize(vpath=path, saveto="planned_path.png")
        except nx.NetworkXNoPath:
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

    def visualize(self):
        """Visualize the nodes and edges of the roadmap."""
        vertices = self.rm.vertices.copy()
        poses = list(map(utils.particle_to_pose, vertices))
        msg = PoseArray(header=Header(frame_id="map"), poses=poses)
        self.nodes_viz.publish(msg)

        # There are lots of edges, so we'll shuffle and incrementally visualize
        # them. This is more work overall, but gives more immediate feedback
        # about the graph.
        all_edges = np.empty((0, 2), dtype=int)
        edges = np.array(self.rm.graph.edges(), dtype=int)
        np.random.shuffle(edges)
        split_indices = np.array(
            [500, 1000, 2000, 5000]
            + [10000, 20000, 50000]
            + list(range(100000, edges.shape[0], 100000)),
            dtype=int,
        )
        for batch in np.split(edges, split_indices, axis=0):
            batch_edges = []
            for u, v in batch:
                q1 = self.rm.vertices[u, :]
                q2 = self.rm.vertices[v, :]
                # Check edge validity on the problem rather than roadmap to
                # circumvent edge collision-checking count
                if not self.rm.problem.check_edge_validity(q1, q2):
                    continue
                edge, _ = self.problem.steer(
                    q1, q2, resolution=0.25, interpolate_line=False
                )
                with_repeats = np.repeat(edge[:, :2], 2, 0).reshape(-1, 2)[1:-1]
                batch_edges.append(with_repeats)
            if not batch_edges:
                continue
            batch_edges = np.vstack(batch_edges)
            all_edges = np.vstack((all_edges, batch_edges))
            points = list(map(lambda x: Point(x=x[0], y=x[1], z=-1), all_edges))
            msg = Marker(
                header=Header(frame_id="map"), type=Marker.LINE_LIST, points=points
            )
            msg.scale.x = 0.01
            msg.pose.orientation.w = 1.0
            msg.color.a = 0.1
            self.edges_viz.publish(msg)


def mkdir_p(path):
    """Equivalent to mkdir -p path.

    The exist_ok flag for os.makedirs was introduced in Python 3.2.
    This function provides Python 2 support.
    """
    try:
        os.makedirs(path)
    except OSError as exc:  # Python ≥ 2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        # possibly handle other errno cases here, otherwise finally:
        else:
            raise


def graph_location(
    problem_name,
    sampler_name,
    num_vertices,
    connection_radius,
    curvature=None,
    map_name=None,
):
    """Return the name of this graph in the cache."""
    cache_dir = os.path.expanduser("~/.ros/graph_cache/")
    mkdir_p(cache_dir)
    if map_name is None:
        map_name, _ = os.path.splitext(os.path.basename(rospy.get_param("/map_file")))
    params = [map_name, problem_name, sampler_name, num_vertices, connection_radius]
    if problem_name == "se2":
        params.append(curvature)
    name = "_".join(str(p) for p in params)
    return os.path.join(cache_dir, name + ".pkl")
