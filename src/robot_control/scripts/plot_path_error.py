#!/usr/bin/env python3
"""
Real-time Path Tracking Error Visualization
Subscribes to /path_error topic and plots error curves in real-time
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np


class PathErrorPlotter(Node):
    """ROS 2 node for real-time path error visualization"""

    def __init__(self, max_points=500):
        super().__init__('path_error_plotter')

        # Subscribe to path error topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/path_error',
            self.error_callback,
            10)

        # Data storage (use deque for efficient append/pop)
        self.max_points = max_points
        self.time_data = deque(maxlen=max_points)
        self.x_error_data = deque(maxlen=max_points)
        self.y_error_data = deque(maxlen=max_points)
        self.cross_track_error_data = deque(maxlen=max_points)
        self.heading_error_data = deque(maxlen=max_points)

        self.start_time = self.get_clock().now()
        self.error_count = 0

        self.get_logger().info('Path Error Plotter initialized')
        self.get_logger().info('Waiting for /path_error messages...')

    def error_callback(self, msg):
        """Callback function for /path_error topic"""
        if len(msg.data) >= 4:
            # Get current time in seconds
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            # Store error data
            self.time_data.append(current_time)
            self.x_error_data.append(msg.data[0])
            self.y_error_data.append(msg.data[1])
            self.cross_track_error_data.append(msg.data[2])
            self.heading_error_data.append(msg.data[3])

            self.error_count += 1

            # Log every 50 messages to avoid spam
            if self.error_count % 50 == 0:
                self.get_logger().info(
                    f'Received {self.error_count} error messages. '
                    f'CTE: {msg.data[2]:.4f}m, Heading: {msg.data[3]:.4f}rad'
                )


def animate_plot(frame, plotter, lines, ax1, ax2):
    """Animation function for updating plots"""
    if len(plotter.time_data) > 0:
        time_array = np.array(plotter.time_data)
        x_error_array = np.array(plotter.x_error_data)
        y_error_array = np.array(plotter.y_error_data)
        cross_track_array = np.array(plotter.cross_track_error_data)
        heading_array = np.array(plotter.heading_error_data)

        # Update position error plots
        lines[0].set_data(time_array, x_error_array)
        lines[1].set_data(time_array, y_error_array)
        lines[2].set_data(time_array, cross_track_array)

        # Update heading error plot
        lines[3].set_data(time_array, heading_array)

        # Auto-scale axes
        for ax in [ax1, ax2]:
            ax.relim()
            ax.autoscale_view()

    return lines


def main(args=None):
    rclpy.init(args=args)

    # Create plotter node
    plotter = PathErrorPlotter(max_points=500)

    # Set up matplotlib figure
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Path Tracking Error Monitor', fontsize=16, fontweight='bold')

    # Subplot 1: Position Errors
    line1, = ax1.plot([], [], 'r-', linewidth=2, label='X Error')
    line2, = ax1.plot([], [], 'g-', linewidth=2, label='Y Error')
    line3, = ax1.plot([], [], 'b-', linewidth=2, label='Cross-Track Error')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Error (m)', fontsize=12)
    ax1.set_title('Position Errors', fontsize=14)
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Subplot 2: Heading Error
    line4, = ax2.plot([], [], 'm-', linewidth=2, label='Heading Error')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Error (rad)', fontsize=12)
    ax2.set_title('Heading Error', fontsize=14)
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    lines = [line1, line2, line3, line4]

    # Create animation
    ani = animation.FuncAnimation(
        fig,
        animate_plot,
        fargs=(plotter, lines, ax1, ax2),
        interval=100,  # Update every 100ms
        blit=False
    )

    plt.tight_layout()

    # Show plot in non-blocking mode
    plt.ion()
    plt.show()

    # ROS spin loop
    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            rclpy.spin_once(plotter, timeout_sec=0.01)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
