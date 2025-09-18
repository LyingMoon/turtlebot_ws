#!/usr/bin/env python3

from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw

from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

import rclpy
from rclpy.node import Node
import numpy as np
# from util_explore import explore

from scipy.signal import convolve2d
def explore(occupancy: StochOccupancyGrid2D):

    window_size = 13    # defines the window side-length for neighborhood of cells to consider for heuristics
    # window_size = occupancy.window_size
    ########################### Code starts here ###########################
    kernel_ones = np.ones((window_size, window_size))
    kernel_binary = np.ones((window_size, window_size), dtype=int) 
    threshold = 1e-7
    num_surrounding = window_size**2

    binary_unknow_matrix = (np.abs(occupancy.probs + 1) < threshold).astype(int)
    result_unknow = convolve2d(binary_unknow_matrix, kernel_binary, mode='same', boundary='fill', fillvalue=1)
    result_unknow_condit = (result_unknow >= 0.2 * num_surrounding).astype(int)

    binary_known_occup_matrix = ((occupancy.probs - 0.5) > -1*threshold).astype(int)
    result_knwon_occup = convolve2d(binary_known_occup_matrix, kernel_binary, 
                                    mode='same', boundary='fill', fillvalue=0)
    result_knwon_occup_condit = ((result_knwon_occup) < threshold).astype(int)

    binary_known_unoccup_matrix = (np.abs(occupancy.probs) < 0.5).astype(int)
    result_knwon_unoccup = convolve2d(binary_known_unoccup_matrix, kernel_binary, 
                                      mode='same', boundary='fill', fillvalue=0)
    result_know_unoccup_condit = (result_knwon_unoccup >= 0.3 * num_surrounding).astype(int)

    final_ok_cells = result_unknow_condit * result_knwon_occup_condit * result_know_unoccup_condit

    final_ok_cells = final_ok_cells.T
    frontier_states = occupancy.grid2state(np.column_stack(np.where(final_ok_cells == 1)))
    ########################### Code ends here ###########################
    return frontier_states


# TODO: Need to test the stop sign algo

class Explorer(Node):
    def __init__(self):
        super().__init__('explore')

        # Parameters
        self.declare_parameter('explore_timeout', 5.0)
        self.explore_timeout = self.get_parameter('explore_timeout').get_parameter_value().double_value

        # Flags
        self.is_exploring = True
        self.occupancy: StochOccupancyGrid2D = None
        self.current_state: TurtleBotState = None

        self.is_update_curr_state = False
        self.is_update_map_logged = False
        self.is_first_time = True

        self.stop_detected = False  # Flag for stop sign detection
        self.stop_duration = 5.0  # Stop duration in seconds
        self.ignore_stop_sign_duration = 10.0  # Ignore stop sign duration after resuming
        self.stop_resume_time = 0.0  # Time to resume after stopping
        self.ignore_until_time = None  # Ignore stop sign until this time

        self.sorted_frontier_states = None
        self.idx_next_state = 0
        self.is_finish = False

        self.declare_parameter('active', True)
        self.nav_success_sub = self.create_subscription(Bool,'/nav_success',self.nav_success_callback,10)
        self.map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.state_sub = self.create_subscription(TurtleBotState,'/state',self.state_callback,10)
        self.detector_sub = self.create_subscription(Bool,'/detector_bool',self.detector_callback,10)
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, '/cmd_nav', 10)

        self.get_logger().info("Exploration Node initialized.")

    @property
    def active(self) -> bool:
        return self.get_parameter('active').get_parameter_value().bool_value
    
    def publish_goal(self, x: float, y: float, theta: float):
        """Publish a goal state to the /cmd_nav topic."""
        if self.stop_detected:
            self.get_logger().info("Stop detected. Holding movement.")
            return

        goal = TurtleBotState()
        goal.x = x
        goal.y = y
        goal.theta = theta
        self.cmd_nav_pub.publish(goal)
        self.get_logger().info(f"Published new goal: x={x}, y={y}, theta={theta}")

    def start_exploration(self):
        """Start exploration by publishing a new goal."""
        if not self.active:
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.publish_goal(x=self.current_state.x, y=self.current_state.y, theta=0.0)
            if not self.stop_detected:
                self.stop_resume_time = current_time + self.stop_duration
            self.stop_detected = True
            if current_time >= self.stop_resume_time:
                self.stop_detected = False
                self.set_parameters([rclpy.Parameter('active', rclpy.Parameter.Type.BOOL, True)])
            return
        
        if not self.is_exploring or self.is_finish:
            return

        if self.occupancy is None:
            self.get_logger().warn("Occupancy map not available. Waiting for /map update.")
            return

        if self.current_state is None:
            self.get_logger().warn("Current state not available. Waiting for /state update.")
            return

        frontier_states = explore(self.occupancy)
        self.is_first_time = False

        if self.idx_next_state >= len(frontier_states):
            self.get_logger().info("All frontier states tested but no one can make A* work. \n Explorer terminated...")
            self.is_finish = True
            return

        if frontier_states.size == 0:
            self.get_logger().info("No unexplored frontier states found. \n Explorer terminated...")
            self.is_finish = True
            return

        current_position = np.array([self.current_state.x, self.current_state.y])
        distances = np.linalg.norm(frontier_states - current_position, axis=1)
        sorted_indices = np.argsort(distances)
        self.sorted_frontier_states = frontier_states[sorted_indices]
        closest_frontier = self.sorted_frontier_states[self.idx_next_state]

        self.is_exploring = False
        self.publish_goal(x=closest_frontier[0], y=closest_frontier[1], theta=0.0)
        self.get_logger().info(f"New exploration goal: x={closest_frontier[0]}, y={closest_frontier[1]}")

    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated

        Args:
            msg (OccupancyGrid): updated map message
        """
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )
        if not self.is_update_map_logged:
            self.get_logger().info("Grid Map updated.")
            self.is_update_map_logged = True

        if self.is_first_time:
            self.start_exploration()

    def nav_success_callback(self, msg: Bool) -> None:
        """
        Callback function for handling navigation success status.
        Args:
            msg (Bool): The message indicating navigation success (True) or failure (False).
        """
        if msg.data:
            self.get_logger().info("Navigation successful. Sending next goal.")
            self.is_exploring = True
            self.is_update_map_logged = False
            self.is_update_curr_state = False
            self.idx_next_state = 0
            self.start_exploration()
        else:
            self.get_logger().warning("Navigation failed. Re-planning...")
            self.is_exploring = True
            self.idx_next_state += 1
            self.start_exploration()

    def state_callback(self, msg: TurtleBotState):
        """Update the robot's current state."""
        self.current_state = msg
        if not self.is_update_curr_state:
            self.is_update_curr_state = True
            self.get_logger().info(f"Current state updated: x={msg.x}, y={msg.y}, theta={msg.theta}")

    def detector_callback(self, msg: Bool):
        """Handle stop sign detection."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        if msg.data:
            if self.ignore_until_time and current_time < self.ignore_until_time:
                self.get_logger().info("Stop sign detected but ignored due to ignore duration.")
                return

            self.stop_detected = True
            self.set_parameters([rclpy.Parameter('active', rclpy.Parameter.Type.BOOL, False)])

            self.ignore_until_time = current_time + self.ignore_stop_sign_duration
            self.stop_resume_time = current_time + self.stop_duration
            self.get_logger().info("Stop sign detected. Pausing exploration.")
        else:
            self.stop_detected = False
            self.ignore_until_time = None

def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()