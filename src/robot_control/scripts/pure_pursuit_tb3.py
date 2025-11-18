#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import threading
import math
from math import cos, sin, tan, atan, atan2, pi

observation = np.array([0.0, 0.0, 0.0], float) # x, y, yaw angle

path = []
vel = 0

# Parameters
k = 0.1  # look forward gain
Lfc = 0.5  # [m] look-ahead distance (reduced for TurtleBot3)
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick

class State:

    def __init__(self):
        global observation, vel
        self.yaw = observation[2]
        self.v = 0.0
        self.x = observation[0]
        self.y = observation[1]

    def update(self, a, time_interval):
        global observation
        self.yaw = observation[2]
        self.v += a * time_interval
        self.x = observation[0]
        self.y = observation[1]

        return self.v

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            if(ind > len(self.cx) - 1):
                ind = len(self.cx) - 1
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                if ind + 1 >= len(self.cx):
                    break
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break

                if (ind < len(self.cx) - 2):
                    ind = ind + 1
                else:
                    break

                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

class Commander(Node):

    def __init__(self):
        super().__init__('commander')

        # TurtleBot3 uses cmd_vel (Twist message)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.time_interval = 0.02
        self.t = 0

        self.target_creation = False

        self.timer = self.create_timer(self.time_interval, self.timer_callback)

        self.target_speed = 0.3  # (m/s) - reduced for TurtleBot3
        self.max_angular_vel = 1.5  # (rad/s)

        # initial state
        self.state = State()

    def pure_pursuit_steer_control(self, state, trajectory, pind):
        global observation, path
        ind, Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        # Calculate angle to target
        dx = tx - state.x
        dy = ty - state.y

        target_angle = math.atan2(dy, dx)
        alpha = target_angle - state.yaw

        # Normalize angle to [-pi, pi]
        while alpha > math.pi:
            alpha -= 2.0 * math.pi
        while alpha < -math.pi:
            alpha += 2.0 * math.pi

        return alpha, ind, Lf

    def proportional_control(self, target, current):
        a = Kp * (target - current)
        return a

    def timer_callback(self):
        global observation, vel, path

        if(len(path) > 0):

            if(self.target_creation == False):
                cx_list = []
                cy_list = []
                for i in range(len(path)):
                    cx_list.append(path[i][0])
                cx = np.array(cx_list)
                for i in range(len(path)):
                    cy_list.append(path[i][1])
                cy = np.array(cy_list)
                self.target_course = TargetCourse(cx, cy)
                self.target_ind, _ = self.target_course.search_target_index(self.state)
                self.target_creation = True

            # Calc control input
            ai = self.proportional_control(self.target_speed, self.state.v)
            alpha, self.target_ind, Lf = self.pure_pursuit_steer_control(
                self.state, self.target_course, self.target_ind)

            state_vel = self.state.update(ai, self.time_interval)

            cmd = Twist()

            if(self.target_ind < len(path) - 2):
                # Pure pursuit control for differential drive
                if abs(Lf) > 0.01:
                    omega = 2.0 * state_vel * math.sin(alpha) / Lf
                else:
                    omega = 0.0

                # Limit angular velocity
                if omega > self.max_angular_vel:
                    omega = self.max_angular_vel
                elif omega < -self.max_angular_vel:
                    omega = -self.max_angular_vel

                cmd.linear.x = state_vel
                cmd.angular.z = omega
            else:  # goal reached
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            self.publisher_cmd_vel.publish(cmd)

class Get_odom(Node):

    def __init__(self):
        super().__init__('get_odom')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        global observation, vel

        # Get position
        observation[0] = msg.pose.pose.position.x
        observation[1] = msg.pose.pose.position.y

        # Get orientation (quaternion to euler)
        orientation = msg.pose.pose.orientation
        q0 = orientation.x
        q1 = orientation.y
        q2 = orientation.z
        q3 = orientation.w

        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q3 * q2 + q0 * q1)
        cosy_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        observation[2] = math.atan2(siny_cosp, cosy_cosp)

        # Get velocity
        vel = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

class Get_path(Node):

    def __init__(self):
        super().__init__('get_path')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/path',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        global path

        path_num = np.array(msg.data)
        path = path_num.reshape([int(path_num.shape[0]/2), 2])
        print(f"Received path with {len(path)} points")

if __name__ == '__main__':
    rclpy.init(args=None)

    commander = Commander()
    get_odom = Get_odom()
    get_path = Get_path()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(get_odom)
    executor.add_node(get_path)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
