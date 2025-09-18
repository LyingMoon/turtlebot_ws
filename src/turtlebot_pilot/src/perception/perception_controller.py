#!/usr/bin/env python3
"""
Perception Controller for TurtleBot3

This module implements a perception-based control system that reacts
to object detection (e.g., stop signs) using computer vision.
"""

from typing import Optional
import time
import rclpy
from rclpy.logging import get_logger
from rclpy.parameter import Parameter

from asl_tb3_lib.control import BaseController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class PerceptionController(BaseController):
    """
    Perception-based controller for reactive behaviors.

    This controller integrates computer vision detection results with
    robot control to enable reactive behaviors like stopping at stop signs.
    """

    # Control constants
    DEFAULT_ROTATION_SPEED = 0.5  # rad/s
    STOP_DURATION = 5.0  # seconds
    CONTROL_FREQUENCY = 5.0  # Hz

    def __init__(self) -> None:
        """Initialize the perception controller."""
        super().__init__('perception_controller')

        # Declare parameters
        self.declare_parameter("active", True)
        self.declare_parameter("rotation_speed", self.DEFAULT_ROTATION_SPEED)
        self.declare_parameter("stop_duration", self.STOP_DURATION)

        # Publishers
        self.control_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribers
        self.detector_sub = self.create_subscription(
            Bool, "/detector_bool", self.detector_callback, 10
        )

        # Timer for periodic control
        control_period = 1.0 / self.CONTROL_FREQUENCY
        self.control_timer = self.create_timer(control_period, self.control_callback)

        # State variables
        self.stop_start_time: Optional[float] = None
        self.detection_active: bool = False

        # Logger
        self._logger = get_logger(self.__class__.__name__)
        self._logger.info("Perception controller initialized")

    @property
    def active(self) -> bool:
        """
        Get the current active state of the controller.

        Returns:
            bool: True if controller is active, False if stopped
        """
        try:
            return self.get_parameter("active").get_parameter_value().bool_value
        except Exception as e:
            self._logger.warn(f"Failed to get active parameter: {e}")
            return True

    @property
    def rotation_speed(self) -> float:
        """
        Get the current rotation speed parameter.

        Returns:
            float: Rotation speed in rad/s
        """
        try:
            return self.get_parameter("rotation_speed").get_parameter_value().double_value
        except Exception as e:
            self._logger.warn(f"Failed to get rotation_speed parameter: {e}")
            return self.DEFAULT_ROTATION_SPEED

    @property
    def stop_duration(self) -> float:
        """
        Get the current stop duration parameter.

        Returns:
            float: Stop duration in seconds
        """
        try:
            return self.get_parameter("stop_duration").get_parameter_value().double_value
        except Exception as e:
            self._logger.warn(f"Failed to get stop_duration parameter: {e}")
            return self.STOP_DURATION

    def detector_callback(self, msg: Bool) -> None:
        """
        Callback for object detection messages.

        Args:
            msg: Boolean message indicating object detection status
        """
        if not isinstance(msg, Bool):
            self._logger.error("Invalid message type received in detector callback")
            return

        if msg.data and self.active:
            self._logger.info("Object detected! Initiating stop sequence.")
            self._initiate_stop()
        elif not msg.data:
            self._logger.debug("No object detected")

    def _initiate_stop(self) -> None:
        """Initiate the stop sequence when an object is detected."""
        try:
            # Set robot to inactive state
            inactive_param = Parameter("active", Parameter.Type.BOOL, False)
            self.set_parameters([inactive_param])

            # Record stop start time
            self.stop_start_time = time.time()
            self.detection_active = True

            self._logger.info(f"Stop initiated, will resume after {self.stop_duration}s")

        except Exception as e:
            self._logger.error(f"Failed to initiate stop: {e}")

    def _check_stop_completion(self) -> None:
        """Check if the stop duration has elapsed and resume if needed."""
        if (self.stop_start_time is not None and
            time.time() - self.stop_start_time >= self.stop_duration):

            try:
                # Resume normal operation
                active_param = Parameter("active", Parameter.Type.BOOL, True)
                self.set_parameters([active_param])

                self.stop_start_time = None
                self.detection_active = False

                self._logger.info("Stop sequence completed, resuming normal operation")

            except Exception as e:
                self._logger.error(f"Failed to resume operation: {e}")

    def compute_control(self) -> TurtleBotControl:
        """
        Compute control command based on current state.

        Returns:
            TurtleBotControl: Control command with appropriate velocities
        """
        control_msg = TurtleBotControl()

        try:
            if self.active:
                # Normal operation - rotate to scan environment
                control_msg.omega = self.rotation_speed
                control_msg.v = 0.0
            else:
                # Stopped state
                control_msg.omega = 0.0
                control_msg.v = 0.0

                # Check if stop duration has elapsed
                self._check_stop_completion()

        except Exception as e:
            self._logger.error(f"Error in compute_control: {e}")
            # Safe default: stop the robot
            control_msg.omega = 0.0
            control_msg.v = 0.0

        return control_msg

    def control_callback(self) -> None:
        """Timer callback for publishing control commands."""
        try:
            control_msg = self.compute_control()

            # Convert to Twist message for publishing
            twist_msg = Twist()
            twist_msg.linear.x = control_msg.v
            twist_msg.angular.z = control_msg.omega

            self.control_pub.publish(twist_msg)

        except Exception as e:
            self._logger.error(f"Error in control callback: {e}")


def main(args: Optional[list] = None) -> None:
    """Main entry point for the perception controller node."""
    try:
        rclpy.init(args=args)

        perception_controller = PerceptionController()
        rclpy.spin(perception_controller)

    except KeyboardInterrupt:
        get_logger("perception_controller").info("Shutdown requested by user")
    except Exception as e:
        get_logger("perception_controller").error(f"Unexpected error: {e}")
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()