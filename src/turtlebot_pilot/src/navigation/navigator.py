#!/usr/bin/env python3

import math
import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass
import scipy

from scipy.interpolate import splrep
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
import numpy as np
import typing as T

from enum import Enum
from dataclasses import dataclass
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from scipy.interpolate import splev
from std_msgs.msg import Bool

from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.control import BaseController
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
from asl_tb3_lib.math_utils import wrap_angle, distance_linear, distance_angular


class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########
        if x==self.x_init or x==self.x_goal:
            return True
        for dim in range(len(x)):
            if x[dim] < self.statespace_lo[dim]:
                return False
            if x[dim] > self.statespace_hi[dim]:
                return False
        if not self.occupancy.is_free(np.asarray(x)):
            return False
        return True
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        return np.linalg.norm(np.array(x1)-np.array(x2))
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########
        for dx1 in [-self.resolution, 0, self.resolution]:
            for dx2 in [-self.resolution, 0, self.resolution]:
                if dx1==0 and dx2==0:
                    # don't include itself
                    continue
                new_x = (x[0]+dx1,x[1]+dx2)
                if self.is_free(new_x):
                    neighbors.append(self.snap_to_grid(new_x))
        ########## Code ends here ##########
        return neighbors

    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))


    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found

        HINTS:  We're representing the open and closed sets using python's built-in
                set() class. This allows easily adding and removing items using
                .add(item) and .remove(item) respectively, as well as checking for
                set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        while len(self.open_set)>0:
            current = self.find_best_est_cost_through()
            if current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(current)
            self.closed_set.add(current)
            for n in self.get_neighbors(current):
                if n in self.closed_set:
                    continue
                tentative_cost_to_arrive = self.cost_to_arrive[current] + self.distance(current,n)
                if n not in self.open_set:
                    self.open_set.add(n)
                elif tentative_cost_to_arrive >= self.cost_to_arrive[n]:
                    continue
                self.came_from[n] = current
                self.cost_to_arrive[n] = tentative_cost_to_arrive
                self.est_cost_through[n] = self.cost_to_arrive[n] + self.distance(n,self.x_goal)

        return False
        ########## Code ends here ##########



class NavigatorNode(BaseNavigator):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("navigator_node")

        self.kp = 2.0

        self.kpx = 2.0
        self.kpy = 2.0
        self.kdx = 2.0
        self.kdy = 2.0

        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

        self.V_PREV_THRESH = 0.0001


    def compute_heading_control(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        """ Compute only orientation target (used for NavMode.ALIGN and NavMode.Park)

        Returns:
            TurtleBotControl: control target
        """
        error = wrap_angle(goal.theta - state.theta)
                
        omega = self.kp * error
        
        control_message = TurtleBotControl()
        control_message.omega = omega
        
        return control_message
        # raise NotImplementedError("You need to implement this!")

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float,
    ) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """
        dt = t - self.t_prev  # Time step difference
        x, y, th = state.x, state.y, state.theta

        if abs(self.V_prev) < self.V_PREV_THRESH:
            self.V_prev = self.V_PREV_THRESH
        
        x_d = splev(t, plan.path_x_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        
        y_d = splev(t, plan.path_y_spline, der=0)
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)
        u1 = xdd_d + self.kpx * (x_d - x) + self.kdx * (xd_d - self.V_prev * np.cos(th))
        u2 = ydd_d + self.kpy * (y_d - y) + self.kdy * (yd_d - self.V_prev * np.sin(th))
        
        J = np.array([
        [np.cos(th), -self.V_prev * np.sin(th)],
        [np.sin(th), self.V_prev * np.cos(th)]])
        
        V_dot, om = np.linalg.inv(J) @ np.array([u1, u2])
        V = self.V_prev + V_dot * dt
        
        if abs(V) < self.V_PREV_THRESH:
            V = self.V_PREV_THRESH
        
        # V = np.clip(V, -self.V_max, self.V_max)
        # om = np.clip(om, -self.om_max, self.om_max)
        
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om
        control_message = TurtleBotControl()
        control_message.v = V
        control_message.omega = om
        return control_message


        
        # raise NotImplementedError("You need to implement this!")

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float,
    ) -> T.Optional[TrajectoryPlan]:
        """ Compute a trajectory plan using A* and cubic spline fitting

        Args:
            state (TurtleBotState): state
            goal (TurtleBotState): goal
            occupancy (StochOccupancyGrid2D): occupancy
            resolution (float): resolution
            horizon (float): horizon

        Returns:
            T.Optional[TrajectoryPlan]:
        """
        astar = AStar(
        statespace_lo=(state.x - horizon, state.y - horizon),
        statespace_hi=(state.x + horizon, state.y + horizon),
        x_init=(state.x, state.y),
        x_goal=(goal.x, goal.y),
        occupancy=occupancy,
        resolution=resolution)

        if not astar.solve() or len(astar.path) < 4:
            return None
        
        self.V_prev = 0.0
        self.t_prev = 0.0

        path = np.array(astar.path)
        v_desired = 0.15  
        
        ts = np.zeros(len(path))
        for i in range(1, len(path)):
            ts[i] = ts[i - 1] + np.linalg.norm(path[i] - path[i - 1]) / v_desired

        spline_alpha = 0.05  
        path_x_spline = splrep(ts, path[:, 0], k=3, s=spline_alpha)
        path_y_spline = splrep(ts, path[:, 1], k=3, s=spline_alpha)
        
        return TrajectoryPlan(
        path=path,        
        path_x_spline=path_x_spline,
        path_y_spline=path_y_spline, 
        duration=ts[-1] )          


if __name__ == "__main__":
    rclpy.init()       
    node = NavigatorNode() 
    rclpy.spin(node)   
    rclpy.shutdown()     