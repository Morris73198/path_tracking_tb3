#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import threading
import math
from math import cos, sin, atan2, pi
import cvxpy

# Global variables
observation = np.array([0.0, 0.0, 0.0], float)  # x, y, yaw angle
path = []

# TurtleBot3 parameters
MAX_LINEAR_VEL = 0.22  # [m/s] TurtleBot3 Burger max linear velocity
MAX_ANGULAR_VEL = 2.84  # [rad/s] TurtleBot3 Burger max angular velocity
MAX_LINEAR_ACC = 0.5   # [m/s^2] maximum linear acceleration
MAX_ANGULAR_ACC = 3.0  # [rad/s^2] maximum angular acceleration

# MPC parameters
NX = 4  # state vector: [x, y, v, yaw]
NU = 2  # input vector: [a, omega] (linear acceleration, angular velocity)
T = 10  # prediction horizon length
DT = 0.1  # [s] time step

# Cost matrices
R = np.diag([0.01, 0.5])  # input cost matrix [a, omega] - higher omega cost prevents spinning
Rd = np.diag([0.01, 1.0])  # input difference cost matrix - smooth angular changes
Q = np.diag([1.0, 1.0, 0.5, 0.3])  # state cost matrix [x, y, v, yaw] - lower yaw weight
Qf = Q  # terminal state cost matrix

# Control parameters
TARGET_SPEED = 0.2  # [m/s] target speed for TurtleBot3
GOAL_DIS = 0.3  # [m] goal distance threshold
STOP_SPEED = 0.02  # [m/s] stop speed threshold
N_IND_SEARCH = 10  # search index number

# Iteration parameters
MAX_ITER = 5  # max iteration for MPC (increased for better convergence)
DU_TH = 0.5  # iteration finish threshold (relaxed for faster convergence)


class State:
    """TurtleBot3 state class"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.prev_yaw = yaw
        self.v = v

    def update(self, a, omega):
        """Update state with differential drive model"""
        global observation

        # Update from odometry
        self.x = observation[0]
        self.y = observation[1]
        self.yaw = observation[2]

        # Handle yaw angle discontinuity (prevents unwanted spinning)
        dyaw = self.yaw - self.prev_yaw
        while dyaw >= math.pi / 2.0:
            self.yaw -= math.pi * 2.0
            dyaw = self.yaw - self.prev_yaw
        while dyaw <= -math.pi / 2.0:
            self.yaw += math.pi * 2.0
            dyaw = self.yaw - self.prev_yaw

        self.prev_yaw = self.yaw

        # Update velocity with acceleration
        self.v += a * DT

        # Limit velocity
        if self.v > MAX_LINEAR_VEL:
            self.v = MAX_LINEAR_VEL
        elif self.v < -MAX_LINEAR_VEL:
            self.v = -MAX_LINEAR_VEL

        return self.v


def pi_2_pi(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


def get_linear_model_matrix(v, yaw, omega):
    """
    Get linearized discrete-time model matrices for differential drive robot

    State: [x, y, v, yaw]
    Input: [a, omega] (linear acceleration, angular velocity)

    Returns:
        A: State transition matrix
        B: Input matrix
        C: Constant term
    """
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * cos(yaw)
    A[0, 3] = -DT * v * sin(yaw)
    A[1, 2] = DT * sin(yaw)
    A[1, 3] = DT * v * cos(yaw)

    B = np.zeros((NX, NU))
    B[2, 0] = DT  # velocity affected by acceleration
    B[3, 1] = DT  # yaw affected by angular velocity

    C = np.zeros(NX)
    C[0] = DT * v * sin(yaw) * yaw
    C[1] = -DT * v * cos(yaw) * yaw
    C[3] = -DT * omega

    return A, B, C


def predict_motion(x0, oa, od, xref):
    """Predict future motion based on input sequence"""
    xbar = xref * 0.0
    for i in range(NX):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])

    for (ai, omegai, i) in zip(oa, od, range(1, T + 1)):
        # Simple prediction using differential drive model
        state.v += ai * DT
        state.yaw += omegai * DT
        state.yaw = pi_2_pi(state.yaw)
        state.x += state.v * cos(state.yaw) * DT
        state.y += state.v * sin(state.yaw) * DT

        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def get_nparray_from_matrix(x):
    """Convert cvxpy matrix to numpy array"""
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
    """Calculate nearest index on the trajectory"""
    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)
    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def calc_ref_trajectory(state, cx, cy, cyaw, sp, dl, pind):
    """Calculate reference trajectory"""
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    # Normalize all reference yaw angles relative to current state yaw
    for i in range(T + 1):
        xref[3, i] = pi_2_pi(xref[3, i] - state.yaw) + state.yaw

    return xref, ind, dref


def linear_mpc_control(xref, xbar, x0, dref):
    """
    Linear MPC control for differential drive robot

    Args:
        xref: reference trajectory
        xbar: operational point trajectory
        x0: current state
        dref: reference input (not used for diff drive, kept for compatibility)

    Returns:
        oa: optimal acceleration sequence
        oomega: optimal angular velocity sequence
        ox, oy, oyaw, ov: predicted optimal trajectory
    """
    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], 0.0)
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            # Angular acceleration constraint
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_ANGULAR_ACC * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_LINEAR_VEL]
    constraints += [x[2, :] >= -MAX_LINEAR_VEL]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_LINEAR_ACC]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_ANGULAR_VEL]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        oomega = get_nparray_from_matrix(u.value[1, :])
    else:
        print("Error: Cannot solve MPC problem")
        oa, oomega, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, oomega, ox, oy, oyaw, ov


def iterative_linear_mpc_control(xref, x0, dref, oa, oomega):
    """
    Iterative linear MPC control

    Args:
        xref: reference trajectory
        x0: current state
        dref: reference input
        oa: previous acceleration sequence
        oomega: previous angular velocity sequence

    Returns:
        oa, oomega: optimal control sequences
        ox, oy, oyaw, ov: predicted optimal trajectory
    """
    if oa is None or oomega is None:
        oa = [0.0] * T
        oomega = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, oomega, xref)
        poa, poomega = oa[:], oomega[:]
        oa, oomega, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)

        if oa is None:
            return poa, poomega, None, None, None, None

        du = sum(abs(np.array(oa) - np.array(poa))) + sum(abs(np.array(oomega) - np.array(poomega)))
        if du <= DU_TH:
            # Converged successfully
            break
    # Note: Removed "max iterations" message as it's normal during steady-state tracking

    return oa, oomega, ox, oy, oyaw, ov


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """Calculate speed profile along the trajectory"""
    speed_profile = [target_speed] * len(cx)
    return speed_profile


def smooth_yaw(yaw):
    """Smooth yaw angle sequence to avoid discontinuities"""
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= pi / 2.0:
            yaw[i + 1] -= pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -pi / 2.0:
            yaw[i + 1] += pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def calc_yaw(cx, cy):
    """Calculate yaw angle from trajectory points"""
    yaw = []
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        yaw.append(atan2(dy, dx))
    yaw.append(yaw[-1])  # duplicate last yaw
    return yaw


def check_goal(state, goal, tind, nind):
    """Check if robot reached the goal"""
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False


class MPCController(Node):
    """MPC Controller Node for TurtleBot3"""

    def __init__(self):
        super().__init__('mpc_controller')

        # Publisher for TurtleBot3 velocity commands
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for path tracking errors
        self.publisher_path_error = self.create_publisher(Float64MultiArray, '/path_error', 10)

        self.time_interval = DT
        self.timer = self.create_timer(self.time_interval, self.timer_callback)

        self.dl = 0.2  # course resolution [m]
        self.target_creation = False

        # MPC control sequences
        self.oa = None
        self.oomega = None

        self.get_logger().info('MPC Controller for TurtleBot3 initialized')

    def timer_callback(self):
        global path, observation

        if len(path) > 0:
            if not self.target_creation:
                # Create target course from path
                cx_list = []
                cy_list = []
                for i in range(len(path)):
                    cx_list.append(path[i][0])
                    cy_list.append(path[i][1])

                self.cx = cx_list
                self.cy = cy_list
                self.cyaw = calc_yaw(self.cx, self.cy)
                self.cyaw = smooth_yaw(self.cyaw)
                self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED)

                # Initialize state
                self.state = State(x=observation[0], y=observation[1],
                                 yaw=observation[2], v=0.0)

                # Initial yaw compensation to prevent discontinuity
                if self.state.yaw - self.cyaw[0] >= math.pi:
                    self.state.yaw -= math.pi * 2.0
                elif self.state.yaw - self.cyaw[0] <= -math.pi:
                    self.state.yaw += math.pi * 2.0

                self.state.prev_yaw = self.state.yaw

                # Find nearest point
                self.target_ind, _ = calc_nearest_index(
                    self.state, self.cx, self.cy, self.cyaw, 0)

                self.target_creation = True
                self.get_logger().info(f'Path received with {len(path)} points')

            # Get reference trajectory
            xref, self.target_ind, dref = calc_ref_trajectory(
                self.state, self.cx, self.cy, self.cyaw, self.sp,
                self.dl, self.target_ind)

            # Calculate and publish path tracking errors
            _, cross_track_error = calc_nearest_index(
                self.state, self.cx, self.cy, self.cyaw, self.target_ind)

            # Calculate position errors relative to the nearest point
            x_error = self.cx[self.target_ind] - self.state.x
            y_error = self.cy[self.target_ind] - self.state.y
            heading_error = pi_2_pi(self.cyaw[self.target_ind] - self.state.yaw)

            # Publish errors: [x_error, y_error, cross_track_error, heading_error]
            error_msg = Float64MultiArray()
            error_msg.data = [float(x_error), float(y_error),
                            float(cross_track_error), float(heading_error)]
            self.publisher_path_error.publish(error_msg)

            x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]

            # Solve MPC
            self.oa, self.oomega, ox, oy, oyaw, ov = iterative_linear_mpc_control(
                xref, x0, dref, self.oa, self.oomega)

            # Publish control command
            cmd = Twist()

            if self.oa is not None and self.oomega is not None:
                ai = self.oa[0]
                omega_i = self.oomega[0]

                # Update state
                self.state.update(ai, omega_i)

                # Check if goal reached
                goal = [self.cx[-1], self.cy[-1]]
                if check_goal(self.state, goal, self.target_ind, len(self.cx)):
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info('Goal reached!')
                else:
                    cmd.linear.x = float(self.state.v)
                    cmd.angular.z = float(omega_i)
            else:
                # MPC solver failed, stop the robot
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            self.publisher_cmd_vel.publish(cmd)


class OdometrySubscriber(Node):
    """Subscribe to odometry for TurtleBot3"""

    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global observation

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
        observation[2] = atan2(siny_cosp, cosy_cosp)


class PathSubscriber(Node):
    """Subscribe to path topic"""

    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/path',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global path
        path_num = np.array(msg.data)
        path = path_num.reshape([int(path_num.shape[0] / 2), 2])
        self.get_logger().info(f"Received path with {len(path)} points")


def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    mpc_controller = MPCController()
    odom_subscriber = OdometrySubscriber()
    path_subscriber = PathSubscriber()

    # Use multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mpc_controller)
    executor.add_node(odom_subscriber)
    executor.add_node(path_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = mpc_controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()