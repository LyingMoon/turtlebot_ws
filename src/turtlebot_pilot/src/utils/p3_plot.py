#!/usr/bin/env python3
"""
Plotting Utilities for TurtleBot3

This module provides visualization capabilities for robot trajectory
tracking and performance analysis.
"""

from typing import Optional, List
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.math_utils import wrap_angle


class PlottingNode(Node):
    """
    Node for real-time trajectory plotting and analysis.

    This node records robot state data and generates plots for
    performance evaluation and system analysis.
    """

    # Plotting constants
    DEFAULT_GOAL_ANGLE = 1.274  # radians
    DEFAULT_MESSAGE_COUNT = 800
    DEFAULT_PUBLISH_RATE = 4.0  # Hz
    DEFAULT_OUTPUT_DIR = "data/plots"

    def __init__(
        self,
        goal_angle: float = DEFAULT_GOAL_ANGLE,
        total_message_count: int = DEFAULT_MESSAGE_COUNT
    ) -> None:
        """
        Initialize the plotting node.

        Args:
            goal_angle: Target angle for heading control analysis
            total_message_count: Number of messages to record before plotting
        """
        super().__init__("trajectory_plotter")

        # Configuration parameters
        self.goal_angle = goal_angle
        self.total_message_count = total_message_count
        self.publish_rate = self.DEFAULT_PUBLISH_RATE

        # State variables
        self.angles: List[float] = []
        self.timestamps: List[float] = []
        self.positions_x: List[float] = []
        self.positions_y: List[float] = []
        self.has_plotted = False

        # Publishers
        self.goal_pub = self.create_publisher(TurtleBotState, "/cmd_pose", 10)

        # Subscribers
        self.state_sub = self.create_subscription(
            TurtleBotState, "/state", self.state_callback, 10
        )

        # Timer for periodic goal publishing
        publish_period = 1.0 / self.publish_rate
        self.goal_timer = self.create_timer(publish_period, self.publish_goal)

        # Logger
        self._logger = get_logger(self.__class__.__name__)
        self._logger.info(
            f"Trajectory plotter initialized. Target angle: {goal_angle:.3f} rad"
        )
        self._logger.info(f"Recording {total_message_count} messages before plotting")

    def publish_goal(self) -> None:
        """Publish the goal pose periodically."""
        if not self.has_plotted:
            try:
                goal_msg = TurtleBotState()
                goal_msg.theta = self.goal_angle
                goal_msg.x = 0.0  # Can be extended for position goals
                goal_msg.y = 0.0

                self.goal_pub.publish(goal_msg)

            except Exception as e:
                self._logger.error(f"Failed to publish goal: {e}")

    def state_callback(self, msg: TurtleBotState) -> None:
        """
        Callback for robot state messages.

        Args:
            msg: Current robot state containing position and orientation
        """
        if not isinstance(msg, TurtleBotState):
            self._logger.error("Invalid message type received")
            return

        if len(self.angles) < self.total_message_count:
            # Record state data
            current_time = self.get_clock().now().nanoseconds / 1e9

            self.angles.append(wrap_angle(msg.theta))
            self.positions_x.append(msg.x)
            self.positions_y.append(msg.y)
            self.timestamps.append(current_time)

            # Log progress periodically
            if len(self.angles) % 100 == 0:
                progress = len(self.angles) / self.total_message_count * 100
                self._logger.info(f"Recording progress: {progress:.1f}%")

        elif not self.has_plotted:
            # Generate plots when enough data is collected
            self._generate_plots()
            self.has_plotted = True

    def _generate_plots(self) -> None:
        """Generate and save trajectory analysis plots."""
        try:
            self._logger.info("Generating trajectory plots...")

            # Prepare data
            angles_array = np.array(self.angles)
            times_array = np.array(self.timestamps)

            # Normalize time to start from 0
            times_array = times_array - times_array[0]

            # Create goal reference
            goal_array = self.goal_angle * np.ones_like(times_array)

            # Create subplots
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

            # Plot heading tracking
            self._plot_heading_tracking(ax1, times_array, angles_array, goal_array)

            # Plot trajectory
            self._plot_trajectory(ax2)

            # Save plot
            self._save_plot(fig)

        except Exception as e:
            self._logger.error(f"Failed to generate plots: {e}")
        finally:
            plt.close('all')

    def _plot_heading_tracking(
        self,
        ax: plt.Axes,
        times: np.ndarray,
        angles: np.ndarray,
        goal: np.ndarray
    ) -> None:
        """
        Plot heading tracking performance.

        Args:
            ax: Matplotlib axes object
            times: Time array
            angles: Measured heading angles
            goal: Goal heading angles
        """
        ax.plot(times, angles, linewidth=2, label="Measured θ", color="cornflowerblue")
        ax.plot(times, goal, linewidth=2, label="Goal θ", color="orange", linestyle="--")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Heading (rad)")
        ax.set_title("Heading Control Performance")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Calculate and display metrics
        final_error = abs(angles[-1] - goal[-1])
        ax.text(0.02, 0.98, f"Final Error: {final_error:.3f} rad",
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    def _plot_trajectory(self, ax: plt.Axes) -> None:
        """
        Plot robot trajectory in 2D space.

        Args:
            ax: Matplotlib axes object
        """
        if len(self.positions_x) > 0 and len(self.positions_y) > 0:
            x_array = np.array(self.positions_x)
            y_array = np.array(self.positions_y)

            ax.plot(x_array, y_array, linewidth=2, color="green", alpha=0.7)
            ax.scatter(x_array[0], y_array[0], color="red", s=100,
                      label="Start", marker="o", zorder=5)
            ax.scatter(x_array[-1], y_array[-1], color="blue", s=100,
                      label="End", marker="s", zorder=5)

            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.set_title("Robot Trajectory")
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
        else:
            ax.text(0.5, 0.5, "No position data available",
                   transform=ax.transAxes, ha='center', va='center')

    def _save_plot(self, fig: plt.Figure) -> None:
        """
        Save the generated plot to file.

        Args:
            fig: Matplotlib figure object
        """
        # Create output directory if it doesn't exist
        output_dir = Path(self.DEFAULT_OUTPUT_DIR)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Generate filename with timestamp
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = output_dir / f"trajectory_analysis_{timestamp}.png"

        try:
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            self._logger.info(f"Plot saved successfully: {filename.absolute()}")

        except OSError as e:
            self._logger.error(f"Failed to save plot to {filename}: {e}")

            # Try fallback location
            fallback_filename = Path("trajectory_plot.png")
            try:
                fig.savefig(fallback_filename, dpi=300, bbox_inches='tight')
                self._logger.info(f"Plot saved to fallback location: {fallback_filename.absolute()}")
            except OSError:
                self._logger.error("Failed to save plot to any location")


def main(args: Optional[list] = None) -> None:
    """Main entry point for the plotting node."""
    try:
        rclpy.init(args=args)

        # Parse command line arguments for customization
        goal_angle = PlottingNode.DEFAULT_GOAL_ANGLE
        message_count = PlottingNode.DEFAULT_MESSAGE_COUNT

        plotting_node = PlottingNode(
            goal_angle=goal_angle,
            total_message_count=message_count
        )

        get_logger("trajectory_plotter").info("Starting trajectory recording...")
        get_logger("trajectory_plotter").info("Press Ctrl+C to stop")

        rclpy.spin(plotting_node)

    except KeyboardInterrupt:
        get_logger("trajectory_plotter").info("Plotting node stopped by user")
    except Exception as e:
        get_logger("trajectory_plotter").error(f"Unexpected error: {e}")
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()