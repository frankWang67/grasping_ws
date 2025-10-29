#!/usr/bin/env python3

import time

import leap_hand_utils.leap_hand_utils as lhu
import numpy as np
import rclpy

# Changed import from DynamixelClient to FTClient
from FT_client import *
from ft_leap.srv import LeapPosition, LeapPosVelEff, LeapVelocity
from rclpy.node import Node
from sensor_msgs.msg import JointState

# LEAP hand conventions:
# 180 is flat out home pose for the index, middle, ring, finger MCPs.
# Applying a positive angle closes the joints more and more to curl closed.
# The MCP is centered at 180 and can move positive or negative to that.
# This convention aligns well with FT motors using a [0, 360] degree range.

# The joint numbering goes from Index (0-3), Middle(4-7), Ring(8-11) to Thumb(12-15) and from MCP Side, MCP Forward, PIP, DIP for each finger.
# For instance, the MCP Side of Index is ID 0, the MCP Forward of Ring is 9, the DIP of Ring is 11

# I recommend you only query when necessary and below 90 samples a second.  Used the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.
# The services allow you to always have the latest data when you want it, and not spam the communication lines with unused data.


class LeapNode(Node):
    def __init__(self):
        super().__init__("leaphand_node")
        # Some parameters to control the hand
        self.kP = self.declare_parameter("kP", 25.0).get_parameter_value().double_value
        self.kI = self.declare_parameter("kI", 0.0).get_parameter_value().double_value
        self.kD = self.declare_parameter("kD", 50.0).get_parameter_value().double_value
        # NOTE: Current limit is not supported by the FT_client API and is no longer set.
        self.curr_lim = self.declare_parameter("curr_lim", 350.0).get_parameter_value().double_value
        self.ema_amount = 0.2
        # Position is assumed to be in degrees, matching the LEAP hand convention.
        self.curr_pos = np.ones(16) * np.pi
        self.curr_pos[13] += 1.61
        self.prev_pos = self.pos = self.curr_pos

        # Subscribes to a variety of sources that can command the hand
        self.create_subscription(JointState, "cmd_leap", self._receive_pose, 10)
        self.create_subscription(JointState, "cmd_allegro", self._receive_allegro, 10)
        self.create_subscription(JointState, "cmd_ones", self._receive_ones, 10)

        # Creates services that can give information about the hand out
        self.create_service(LeapPosition, "leap_position", self.pos_srv)
        self.create_service(LeapVelocity, "leap_velocity", self.vel_srv)
        # self.create_service(LeapEffort, "leap_effort", self.eff_srv)
        # self.create_service(LeapPosVelEff, "leap_pos_vel_eff", self.pos_vel_eff_srv)
        self.create_service(LeapPosVelEff, "leap_pos_vel", self.pos_vel_srv)
        # You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your LEAP Hand. Then use the result.
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0
        self.motors = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        self.perf_totaltime = time.perf_counter()
        self.perf_totaltime = self.perf_totaltime - self.perf_totaltime  # Reset to zero
        self.perf_count = 0

        # --- FTClient Instantiation ---
        # Search for the hand on the first 3 USB ports
        for i in range(3):
            port = f"/dev/ttyUSB{i}"
            try:
                self.get_logger().info(f"Attempting to connect to LEAP Hand on port {port}...")
                # Instantiate FTClient, using degrees to match hand conventions
                self.ft_client = FTClient(self.motors, port, baudrate=1000000, use_degrees=False)
                self.ft_client.connect()
                self.get_logger().info(f"Successfully connected to LEAP Hand on port {port}.")
                break
            except Exception as e:
                self.get_logger().warn(f"Failed to connect on {port}: {e}")
                if i == 2:
                    self.get_logger().error("Could not connect to LEAP Hand on any port.")
                    raise

        # Enable torque
        self.ft_client.set_torque_enabled(self.motors, True)

        # Set PID gains. Note: FTClient sets the same gains for all motors.
        # The original special gains for side-to-side motors are not applied.
        self.ft_client.set_pid_gains(p=int(self.kP), i=int(self.kI), d=int(self.kD))

        # NOTE: Setting current limit is not supported in the provided FT_client.py

        # Set initial position
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Receive LEAP pose and directly control the robot.
    def _receive_pose(self, msg):
        pose = msg.position
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Allegro compatibility, first read the allegro publisher and then convert to leap
    def _receive_allegro(self, msg):
        pose = lhu.allegro_to_LEAPhand(msg.position, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Sim compatibility, first read the sim publisher and then convert to leap
    def _receive_ones(self, msg):
        pose = lhu.sim_ones_to_LEAPhand(np.array(msg.position))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Service that reads and returns the pos of the robot in degrees.
    def pos_srv(self, request, response):
        start = time.perf_counter()
        response.position = self.ft_client.read_pos().tolist()
        # positions = self.ft_client.read_pos()
        # print("Position:")
        # positions -= 3.14
        # for i in range(0, len(positions), 4):
        #     print(
        #         f"{positions[i]:.2f}\t{positions[i+1]:.2f}\t{positions[i+2]:.2f}\t{positions[i+3]:.2f}"
        #     )
        # positions += 3.14
        elapsed_time = time.perf_counter() - start
        self.perf_count += 1
        self.perf_totaltime += elapsed_time
        print(f"Average latency: {self.perf_totaltime / self.perf_count:.4f} seconds")

        return response

    # Service that reads and returns the vel of the robot in deg/s.
    def vel_srv(self, request, response):
        response.velocity = self.ft_client.read_vel().tolist()
        return response

    # Service that reads and returns the effort/current of the robot.
    # # NOTE: FTClient does not support current reading, returns zeros.
    # def eff_srv(self, request, response):
    #     response.effort = np.zeros(len(self.motors)).tolist()
    #     return response

    # Use these combined services to save latency if you need multiple datapoints
    def pos_vel_srv(self, request, response):
        pos, vel = self.ft_client.read_pos_vel()
        response.position = pos.tolist()
        response.velocity = vel.tolist()
        response.effort = np.zeros_like(pos).tolist()
        return response

    # Use these combined services to save latency if you need multiple datapoints
    # NOTE: FTClient does not support current reading, effort will be zeros.
    # def pos_vel_eff_srv(self, request, response):
    #     pos, vel = self.ft_client.read_pos_vel()
    #     response.position = pos.tolist()
    #     response.velocity = vel.tolist()
    #     response.effort = np.zeros_like(pos).tolist()
    #     return response


def main(args=None):
    rclpy.init(args=args)
    leaphand_node = LeapNode()
    rclpy.spin(leaphand_node)
    leaphand_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
