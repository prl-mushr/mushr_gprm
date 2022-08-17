"""Wrapper for networkx astar."""

import networkx as nx


def astar(rm, start, goal):
    """Compute the shortest path from start to goal on a roadmap.
    Args:
        rm: Roadmap instance to plan on
        start, goal: integer labels for the start and goal states
    Returns:
        vpath: a sequence of node labels, including the start and goal
    """
    return nx.astar_path(rm.graph, start, goal, rm.heuristic, rm.lazy_weight)


def shortcut(rm, vpath):
    """Shortcut the path between the start and goal.

    Args:
        rm: Roadmap instance to plan on
        vpath: a sequence of node labels from the start to the goal
        num_trials: number of random trials to attempt

    Returns:
        vpath: a subset of the original vpath that connects vertices directly
    """
    # This shortcutting method employs a greedy O(N^2) algorithm,
    # which checks all pairs of points along a path and applies the
    # best shortcut possible.
    i = 0
    while i < len(vpath):
        best_shortcut = 0.0
        best_index = -1
        for j in range(i + 2, len(vpath)):
            if rm.check_edge_validity(vpath[i], vpath[j]):
                shortcut_length = rm.heuristic(vpath[i], vpath[j])
                current_length = rm.compute_path_length(vpath[i : j + 1])
                improvement = current_length - shortcut_length
                if improvement > best_shortcut:
                    best_shortcut = improvement
                    best_index = j
        if best_index != -1:
            vpath = vpath[: i + 1] + vpath[best_index:]
        i += 1
    return vpath
