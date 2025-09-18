#!/usr/bin/env python3
"""
Frontier Exploration Module for TurtleBot3

This module implements frontier-based exploration for autonomous SLAM,
identifying unexplored regions and planning paths to expand the map.
"""

from typing import Optional, Tuple, List
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from scipy.signal import convolve2d

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool

from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D


class FrontierExplorer(Node):
    """
    Frontier-based exploration node for autonomous mapping.

    This class implements frontier detection and selection algorithms to
    guide the robot towards unexplored areas for complete environment mapping.
    """

    # Exploration constants
    DEFAULT_WINDOW_SIZE = 13
    UNKNOWN_THRESHOLD = 0.2
    UNOCCUPIED_THRESHOLD = 0.3
    OCCUPIED_THRESHOLD = 0.0

    def __init__(self) -> None:
        """Initialize the frontier explorer node."""
        super().__init__('frontier_explore')

        # Publishers
        self.frontier_state_pub = self.create_publisher(
            TurtleBotState, "/cmd_nav", 10
        )

        # Subscribers
        self.nav_success_sub = self.create_subscription(
            Bool, '/nav_success', self.nav_success_callback, 10
        )
        self.state_sub = self.create_subscription(
            TurtleBotState, '/state', self.state_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # State variables
        self.robot_pose: Optional[TurtleBotState] = None
        self.state: Optional[np.ndarray] = None
        self.occupancy: Optional[StochOccupancyGrid2D] = None
        self.nav_success: bool = False
        self.is_first_exploration: bool = True
        self.window_size: int = self.DEFAULT_WINDOW_SIZE

        # Logger
        self._logger = get_logger(self.__class__.__name__)
        self._logger.info("Frontier explorer initialized")

    def state_callback(self, msg: TurtleBotState) -> None:
        """
        Callback for robot state updates.

        Args:
            msg: Current robot state message
        """
        self.robot_pose = msg
        self.state = np.array([msg.x, msg.y, msg.theta])
        self._logger.debug(f"Robot state updated: ({msg.x:.2f}, {msg.y:.2f}, {msg.theta:.2f})")

    def map_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback triggered when the map is updated.

        Args:
            msg: Occupancy grid message containing the current map
        """
        try:
            self.occupancy = StochOccupancyGrid2D(
                resolution=msg.info.resolution,
                size_xy=np.array([msg.info.width, msg.info.height]),
                origin_xy=np.array([
                    msg.info.origin.position.x,
                    msg.info.origin.position.y
                ]),
                window_size=self.window_size,
                probs=msg.data
            )
            self._logger.debug("Map updated successfully")

        except Exception as e:
            self._logger.error(f"Failed to process map update: {e}")

    def nav_success_callback(self, msg: Bool) -> None:
        """
        Callback for navigation success status.

        Args:
            msg: Boolean message indicating navigation success
        """
        if msg.data or self.is_first_exploration:
            self.is_first_exploration = False
            self._logger.info('Navigation successful. Planning next exploration.')
            self.nav_success = True
            self._plan_next_exploration()
        else:
            self._logger.warning('Navigation failed. Waiting for recovery...')
            self.nav_success = False

    def _plan_next_exploration(self) -> None:
        """Plan the next exploration target based on current map knowledge."""
        if not self._is_ready_for_exploration():
            return

        try:
            frontier_states = self._detect_frontiers()

            if len(frontier_states) == 0:
                self._logger.info("No frontiers found. Exploration may be complete.")
                return

            # Select the closest frontier
            closest_frontier = self._select_best_frontier(frontier_states)

            if closest_frontier is not None:
                self._publish_exploration_goal(closest_frontier)

        except Exception as e:
            self._logger.error(f"Error in exploration planning: {e}")

    def _is_ready_for_exploration(self) -> bool:
        """Check if the system is ready for exploration."""
        if self.occupancy is None:
            self._logger.warning("No map available for exploration")
            return False

        if self.state is None:
            self._logger.warning("No robot state available for exploration")
            return False

        return True

    def _detect_frontiers(self) -> np.ndarray:
        """
        Detect frontier cells in the occupancy grid.

        Returns:
            np.ndarray: Array of frontier states in world coordinates
        """
        if self.occupancy is None:
            return np.array([])

        # Create masks for different cell types
        unknown_mask = (self.occupancy.probs == -1).astype(int)
        occupied_mask = (self.occupancy.probs >= 0.5).astype(int)
        unoccupied_mask = (self.occupancy.probs < 0.5).astype(int)

        # Convolve with window kernel to count surrounding cells
        kernel = np.ones((self.window_size, self.window_size), dtype=int)

        unknown_count = convolve2d(
            unknown_mask, kernel, mode='same', boundary='fill', fillvalue=0
        )
        occupied_count = convolve2d(
            occupied_mask, kernel, mode='same', boundary='fill', fillvalue=0
        )
        unoccupied_count = convolve2d(
            unoccupied_mask, kernel, mode='same', boundary='fill', fillvalue=0
        )

        # Find valid frontier cells
        valid_cells = self._find_valid_frontier_cells(
            unknown_count, occupied_count, unoccupied_count
        )

        if len(valid_cells) == 0:
            return np.array([])

        # Convert grid coordinates to world coordinates
        frontier_cells = np.array([[j, i] for i, j in valid_cells])
        frontier_states = self.occupancy.grid2state(frontier_cells)

        return frontier_states

    def _find_valid_frontier_cells(
        self,
        unknown_count: np.ndarray,
        occupied_count: np.ndarray,
        unoccupied_count: np.ndarray
    ) -> List[Tuple[int, int]]:
        """
        Find cells that meet frontier criteria.

        Args:
            unknown_count: Count of unknown cells in each window
            occupied_count: Count of occupied cells in each window
            unoccupied_count: Count of unoccupied cells in each window

        Returns:
            List of valid frontier cell coordinates
        """
        valid_cells = []
        half_window = self.window_size // 2
        total_cells = self.window_size * self.window_size

        height, width = self.occupancy.size_xy[1], self.occupancy.size_xy[0]

        for i in range(half_window, height - half_window):
            for j in range(half_window, width - half_window):
                # Calculate percentages
                unknown_pct = unknown_count[i, j] / total_cells
                unoccupied_pct = unoccupied_count[i, j] / total_cells
                occupied_pct = occupied_count[i, j] / total_cells

                # Apply frontier criteria
                if (unknown_pct >= self.UNKNOWN_THRESHOLD and
                    occupied_pct <= self.OCCUPIED_THRESHOLD and
                    unoccupied_pct >= self.UNOCCUPIED_THRESHOLD):

                    valid_cells.append((i, j))

        return valid_cells

    def _select_best_frontier(self, frontier_states: np.ndarray) -> Optional[TurtleBotState]:
        """
        Select the best frontier from available options.

        Args:
            frontier_states: Array of potential frontier states

        Returns:
            Selected frontier state or None if no valid frontier
        """
        if len(frontier_states) == 0 or self.state is None:
            return None

        # Calculate distances to all frontiers
        robot_position = self.state[:2]  # x, y only
        frontier_positions = frontier_states[:, :2]  # x, y only

        distances = np.linalg.norm(frontier_positions - robot_position, axis=1)

        # Select closest frontier
        closest_index = np.argmin(distances)
        closest_position = frontier_states[closest_index]

        # Create TurtleBotState message
        frontier_goal = TurtleBotState()
        frontier_goal.x = float(closest_position[0])
        frontier_goal.y = float(closest_position[1])
        frontier_goal.theta = 0.0  # Let the planner decide orientation

        return frontier_goal

    def _publish_exploration_goal(self, goal: TurtleBotState) -> None:
        """
        Publish the selected exploration goal.

        Args:
            goal: Target frontier state for exploration
        """
        self.frontier_state_pub.publish(goal)
        self._logger.info(
            f"Published exploration goal: ({goal.x:.2f}, {goal.y:.2f})"
        )


def main(args: Optional[list] = None) -> None:
    """Main entry point for the frontier explorer node."""
    try:
        rclpy.init(args=args)

        node = FrontierExplorer()
        rclpy.spin(node)

    except KeyboardInterrupt:
        get_logger("frontier_explore").info("Shutdown requested by user")
    except Exception as e:
        get_logger("frontier_explore").error(f"Unexpected error: {e}")
    finally:
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()