"""Wrapper for networx astar."""

import numpy as np
import networkx as nx


def astar(rm, start, goal):
    """Compute the shortest path from start to goal on a roadmap.
    Args:
        rm: Roadmap instance to plan on
        start, goal: integer labels for the start and goal states
    Returns:
        vpath: a sequence of node labels, including the start and goal
    """
    return nx.astar_path(rm.graph, start, goal, rm.heuristic)


def shortcut(rm, vpath, num_trials=100):
    """Shortcut the path between the start and goal.

    Args:
        rm: Roadmap instance to plan on
        vpath: a sequence of node labels from the start to the goal
        num_trials: number of random trials to attempt

    Returns:
        vpath: a subset of the original vpath that connects vertices directly
    """
    for _ in range(num_trials):
        if len(vpath) == 2:
            break

        # Randomly select two indices to try and connect. Verify that an edge
        # connecting the two vertices would be collision-free, and that
        # connecting the two indices would actually be shorter.
        #
        # You may find these Roadmap methods useful: check_edge_validity,
        # heuristic, and compute_path_length.
        indices = np.random.choice(len(vpath), size=2, replace=False)
        i, j = np.sort(indices)
        # BEGIN SOLUTION "QUESTION 2.3" ALT="raise NotImplementedError"
        if i + 1 == j:
            continue

        valid = rm.check_edge_validity(vpath[i], vpath[j])
        if not valid:
            continue

        shortcut_length = rm.heuristic(vpath[i], vpath[j])
        current_length = rm.compute_path_length(vpath[i : j + 1])
        if shortcut_length < current_length:
            vpath = vpath[: i + 1] + vpath[j:]
        # END SOLUTION
    return vpath


# BEGIN SOLUTION NO PROMPT
def shortcut_dense(rm, path, num_trials=100):
    dense_path = []
    for i in range(1, len(path)):
        u, v = path[i - 1 : i + 1]
        q1 = rm.vertices[u, :]
        q2 = rm.vertices[v, :]
        edge, _ = rm.problem.steer(q1, q2)
        dense_path.append(edge)
    path = np.vstack(dense_path)

    for _ in range(num_trials):
        if len(path) == 2:
            break

        indices = np.random.choice(len(path), size=2, replace=False)
        i, j = indices.min(), indices.max()
        if j - 1 == 1:
            continue

        q1, q2 = path[i, :], path[j, :]
        valid = rm.problem.check_edge_validity(q1, q2)
        if not valid:
            continue

        edge, shortcut_length = rm.problem.steer(q1, q2)
        current_length = qseq_length(rm, path[i : j + 1, :])
        if shortcut_length < current_length:
            path = np.vstack((path[:i], edge, path[j + 1 :]))
    return path


def qseq_length(rm, path):
    path_length = 0
    for i in range(1, len(path)):
        q1 = path[i - 1, :]
        q2 = path[i, :]
        path_length += rm.problem.compute_heuristic(q1, q2)
    return path_length


# END SOLUTION
