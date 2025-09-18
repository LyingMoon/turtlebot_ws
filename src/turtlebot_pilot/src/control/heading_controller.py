#!/usr/bin/env python3
"""
Heading Controller for TurtleBot3

This module implements a proportional controller for heading control
using gain-scheduled control for trajectory tracking.
"""

from typing import Optional
import numpy as np
import rclpy
from rclpy.logging import get_logger

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState


class HeadingController(BaseHeadingController):
    """
    Proportional controller for TurtleBot heading control.

    This controller implements a simple proportional control law for heading
    regulation, using real-time parameter updates for gain scheduling.
    """

    # Controller constants
    DEFAULT_KP = 2.0

    def __init__(self) -> None:
        """Initialize the heading controller with default parameters."""
        super().__init__()

        # Declare ROS2 parameters
        self.declare_parameter("kp", self.DEFAULT_KP)

        # Logger for debugging
        self._logger = get_logger(self.__class__.__name__)
        self._logger.info("Heading controller initialized")

    @property
    def kp(self) -> float:
        """
        Get real-time proportional gain parameter value.

        Returns:
            float: Current proportional gain value
        """
        try:
            return self.get_parameter("kp").value
        except Exception as e:
            self._logger.warn(f"Failed to get kp parameter: {e}, using default")
            return self.DEFAULT_KP

    def compute_control_with_goal(
        self,
        current_state: TurtleBotState,
        goal_state: TurtleBotState
    ) -> TurtleBotControl:
        """
        Compute control command to reach goal heading.

        Args:
            current_state: Current robot state (position and orientation)
            goal_state: Desired goal state (position and orientation)

        Returns:
            TurtleBotControl: Control command with angular velocity
        """
        if not isinstance(current_state, TurtleBotState):
            raise TypeError("current_state must be TurtleBotState")
        if not isinstance(goal_state, TurtleBotState):
            raise TypeError("goal_state must be TurtleBotState")

        # Calculate heading error with proper angle wrapping
        heading_error = wrap_angle(goal_state.theta - current_state.theta)

        # Apply proportional control law
        omega = self.kp * heading_error

        # Create and populate control message
        control_msg = TurtleBotControl()
        control_msg.omega = float(omega)

        return control_msg


def main(args: Optional[list] = None) -> None:
    """Main entry point for the heading controller node."""
    try:
        rclpy.init(args=args)

        heading_controller = HeadingController()

        rclpy.spin(heading_controller)

    except KeyboardInterrupt:
        get_logger("heading_controller").info("Shutdown requested by user")
    except Exception as e:
        get_logger("heading_controller").error(f"Unexpected error: {e}")
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()