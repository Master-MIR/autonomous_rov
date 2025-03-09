#!/usr/bin/env python

import rclpy
import traceback
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from struct import pack, unpack
from std_msgs.msg import Int16, Float64, Empty, Float64MultiArray, String
from sensor_msgs.msg import Joy, Imu, FluidPressure, LaserScan
from mavros_msgs.srv import CommandLong, SetMode, StreamRate
from mavros_msgs.msg import OverrideRCIn, Mavlink
from mavros_msgs.srv import EndpointAdd
from geometry_msgs.msg import Twist


from autonomous_rov.PIDController import PIDController
from autonomous_rov.CubicTrajectory import CubicTrajectory
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class MyPythonNode(Node):
    def __init__(self):
        super().__init__("listenerMIR")
        self.get_logger().info("This node is named listenerMIR")

        self.ns = self.get_namespace()
        self.get_logger().info("namespace =" + self.ns)
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.pub_angle_degre = self.create_publisher(Twist, 'angle_degree', 10)
        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_angular_velocity = self.create_publisher(Twist, 'angular_velocity', 10)
        self.pub_linear_velocity = self.create_publisher(Twist, 'linear_velocity', 10)

        self.get_logger().info("Publishers created.")

        self.get_logger().info("ask router to create endpoint to enable mavlink/from publication.")

        self.armDisarm(False)  # Not automatically disarmed at startup
        rate = 25  # 25 Hz
        self.setStreamRate(rate)

        self.subscriber()

        # for control
        self.control_rate = 20.0  # Hz
        self.control_period = 1.0 / self.control_rate
        time_tupple = self.get_clock().now().seconds_nanoseconds()
        self.time = time_tupple[0] + (time_tupple[1] * 10**-9)

        # set timer if needed
        timer_period = 0.05  # 50 msec - 20 Hz
        self.timer = self.create_timer(self.control_period, self.timer_callback)
        self.i = 0
        
        # variables
        # mode -> array
        self.set_mode = [0] * 3
        self.set_mode[0] = True  # Mode manual
        self.set_mode[1] = False  # Mode automatic without correction
        self.set_mode[2] = False  # Mode with correction

        # Conditions
        self.init_a0 = True
        self.init_p0 = True
        self.arming = False

        self.angle_roll_ajoyCallback0 = 0.0
        self.angle_pitch_a0 = 0.0
        self.angle_yaw_a0 = 0.0
        self.depth_wrt_startup = 0
        self.depth_p0 = 0

        self.pinger_confidence = 0
        self.pinger_distance = 0

        self.Vmax_mot = 1900
        self.Vmin_mot = 1100

        # corrections for control
        # assume neutral buoyancy + water bottle
        # ~ 1.5 kgf -> 15 N
        # TODO: see the mapping function for 
        self.Correction_yaw = 1500
        self.Correction_depth = 1500 # need to calculate using water bottle + flotability

        # controller parameters
        self.config = {}
        self.declare_and_set_params()

        self.pid_depth = PIDController(type='linear')
        self.pid_yaw = PIDController(type='angular')

        # create parameter callback
        self.set_parameters_callback(self.callback_params)

        self.desired_depth = 0.0
        self.desired_yaw = 0.0

        # call trajectory generation
        self.time_final = 20
        self.traj = CubicTrajectory(self.time, self.time_final)
        self.generated_z_des, self.generated_z_dot_des = self.traj.generateCubicTrajectory()

    def pid_to_pwm(self, pid):
        """
        Convert pid output to pwm signal
        """
        if pid > 0:
            pwm = 1430 + 120 * pid
        else:
            pwm = 1540 - 100 * pid
        if pwm > 1900:
            pwm = 1900
        elif pwm < 1100:
            pwm = 1100
        return pwm
    
    def RelAltCallback(self, data):
        """
        Get depth sensor data from this function
        """
        if (self.init_p0):
            # 1st execution, init
            self.depth_p0 = data
            self.init_p0 = False

        # TODO: 
        # setup depth servo control here
        # ...
        # upcomment to generate trajectory

        # TODO:
        # setup for trajectory control

        i = 0
        traj = self.generated_z_des
        self.desired_depth = traj[i]
        depth_control = self.pid_depth.calculate_pid(self.desired_depth, self.depth_p0, self.time)
        depth_control = self.pid_to_pwm(depth_control)  
        self.Correction_depth = int(depth_control)

        # tolerance for depth
        if np.abs(self.depth_p0 - self.desired_depth) < 0.02:
            if i < len(traj):
                self.desired_depth = traj[i]
                i += 1
        
        
        depth_control = self.pid_depth.calculate_pid(self.desired_depth, self.depth_p0, self.time)
        depth_control = self.pid_to_pwm(depth_control)

        # update Correction_depth
        # Correction_depth = 1500

        # Correction_depth = self.Correction_depth + depth_control #??

        # chagnge the correction depth -> pid control output
        self.Correction_depth = int(depth_control)
        # Send PWM commands to motors in timer

    def OdoCallback(self, data):
        """
        Get imu data from this function
        """
        orientation = data.orientation
        angular_velocity = data.angular_velocity

        # extraction of roll, pitch, yaw angles
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        sinp = 2.0 * (w * y - z * x)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        angle_roll = np.arctan2(sinr_cosp, cosr_cosp)
        angle_pitch = np.arcsin(sinp)
        angle_yaw = np.arctan2(siny_cosp, cosy_cosp)

        if (self.init_a0):
            # at 1st execution, init
            self.angle_roll_a0 = angle_roll
            self.angle_pitch_a0 = angle_pitch
            self.angle_yaw_a0 = angle_yaw
            self.init_a0 = False

        angle_wrt_startup = [0] * 3
        angle_wrt_startup[0] = ((angle_roll - self.angle_roll_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[1] = ((angle_pitch - self.angle_pitch_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[2] = ((angle_yaw - self.angle_yaw_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi

        angle = Twist() # orientation in degrees but using twist msg for some reason
        angle.angular.x = angle_wrt_startup[0]
        angle.angular.y = angle_wrt_startup[1]
        angle.angular.z = angle_wrt_startup[2]

        self.pub_angle_degre.publish(angle)

        # Extraction of angular velocity
        p = angular_velocity.x
        q = angular_velocity.y
        r = angular_velocity.z

        vel = Twist() # angular velocity
        vel.angular.x = p
        vel.angular.y = q
        vel.angular.z = r

        # publish velocity
        self.pub_angular_velocity.publish(vel)

        # TODO: setup pid control
        # Only continue if manual_mode is disabled
        if (self.set_mode[0]):
            return

        # yaw control
        yaw_control = self.pid_yaw.calculate_pid(self.desired_yaw, angle.angular.z, self.time)

        # Send PWM commands to motors
        # yaw command to be adapted using sensor feedback
        # self.Correction_yaw = 1500
        Correction_yaw = self.Correction_yaw + yaw_control

        self.Correction_yaw = int(Correction_yaw)

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1

        if self.set_mode[0]:  # commands sent inside joyCallback()
            return
        elif self.set_mode[
            1]:  # Arbitrary velocity command can be defined here to observe robot's velocity, zero by default
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
            return
        elif self.set_mode[2]: # dis mode
            # send commands in correction mode
            self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, 1500, 1500)
        else:  # normally, never reached
            pass

    def armDisarm(self, armed):
        # This functions sends a long command service with 400 code to arm or disarm motors
        if (armed):
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(CommandLong, 'cmd/command')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info("arming requested, wait_for_service, timeout, result :" + str(result))
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400
            req.confirmation = 0
            req.param1 = 1.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            self.get_logger().info("just before call_async")
            resp = cli.call_async(req)
            self.get_logger().info("just after call_async")
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("Arming Succeeded")
        else:
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(CommandLong, 'cmd/command')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "disarming requested, wait_for_service, (False if timeout) result :" + str(result))
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400
            req.confirmation = 0
            req.param1 = 0.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("Disarming Succeeded")

    def manageStabilize(self, stabilized):
        # This functions sends a SetMode command service to stabilize or reset
        if (stabilized):
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(SetMode, 'set_mode')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "stabilized mode requested, wait_for_service, (False if timeout) result :" + str(result))
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = "0"
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("set mode to STABILIZE Succeeded")

        else:
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            result = False
            cli = self.create_client(SetMode, 'set_mode')
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "manual mode requested, wait_for_service, (False if timeout) result :" + str(result))
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = "19"
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("set mode to MANUAL Succeeded")

    def setStreamRate(self, rate):
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(StreamRate, 'set_stream_rate')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info("stream rate requested, wait_for_service, (False if timeout) result :" + str(result))

        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = rate
        req.on_off = True
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("set stream rate Succeeded")

    def addEndPoint(self):
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(EndpointAdd, 'mavros_router/add_endpoint')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info(
                "add endpoint requesRelAltCallbackted, wait_for_service, (False if timeout) result :" + str(result))

        req = EndpointAdd.Request()
        req.url = "udp://@localhost"
        req.type = 1  # TYPE_GCS
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("add endpoint rate Succeeded")

    def joyCallback(self, data):
        # Joystick buttons
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button
        btn_manual_mode = data.buttons[3]  # Y button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_corrected_mode = data.buttons[0]  # A button

        # Disarming when Back button is pressed
        if (btn_disarm == 1 and self.arming == True):
            self.arming = False
            self.armDisarm(self.arming)

        # Arming when Start button is pressed
        if (btn_arm == 1 and self.arming == False):
            self.arming = True
            self.armDisarm(self.arming)

        # Switch manual, auto anset_moded correction mode
        if (btn_manual_mode and not self.set_mode[0]):
            self.set_mode[0] = True
            self.set_mode[1] = False
            self.set_mode[2] = False
            self.get_logger().info("Mode manual")
        if (btn_automatic_mode and not self.set_mode[1]):
            self.set_mode[0] = False
            self.set_mode[1] = True
            self.set_mode[2] = False
            self.get_logger().info("Mode automatic")
        if (btn_corrected_mode and not self.set_mode[2]):
            self.init_a0 = True
            self.init_p0 = True
            # set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
            self.set_mode[0] = False
            self.set_mode[1] = False
            self.set_mode[2] = True
            self.get_logger().info("Mode correction")

    def velCallback(self, cmd_vel):
        # Only continue if manual_mode is enabled
        if (self.set_mode[1] or self.set_mode[2]):
            return
        else:
            self.get_logger().info("Sending...")

        # Extract cmd_vel message
        roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
        yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
        ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
        forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
        lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
        pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)

        self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                             lateral_left_right)

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral):
        # This function replaces setservo for motor commands.
        # It overrides Rc channels inputs and simulates motor controls.
        # In this case, each channel manages a group of motors not individually as servo set

        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)  # pulseCmd[4]--> pitch
        msg_override.channels[1] = np.uint(channel_roll)  # pulseCmd[3]--> roll
        msg_override.channels[2] = np.uint(channel_throttle)  # pulseCmd[2]--> heave
        msg_override.channels[3] = np.uint(channel_yaw)  # pulseCmd[5]--> yaw
        msg_override.channels[4] = np.uint(channel_forward)  # pulseCmd[0]--> surge
        msg_override.channels[5] = np.uint(channel_lateral)  # pulseCmd[1]--> sway
        msg_override.channels[6] = 1500
        msg_override.channels[7] = 1500

        self.pub_msg_override.publish(msg_override)

    def mapValueScalSat(self, value):
        # Correction_Vel and joy between -1 et 1
        # scaling for publishing with setOverrideRCIN values between 1100 and 1900
        # neutral point is 1500
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)

    def subscriber(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subjoy = self.create_subscription(Joy, "joy", self.joyCallback, qos_profile=qos_profile)
        self.subjoy  # prevent unused variable warning
        self.subcmdvel = self.create_subscription(Twist, "cmd_vel", self.velCallback, qos_profile=qos_profile)
        self.subcmdvel  # prevent unused variable warning
        self.subimu = self.create_subscription(Imu, "imu/data", self.OdoCallback, qos_profile=qos_profile)
        self.subimu  # prevent unused variable warning

        self.subrel_alt = self.create_subscription(Float64, "global_position/rel_alt", self.RelAltCallback,
                                                   qos_profile=qos_profile)
        self.subrel_alt  # prevent unused variable warning

        self.get_logger().info("Subscriptions done.")


    ### For PID Controller ###

    def update_control_param(self):
        self.pid_depth.reconfig_param(self.config['k_p_depth'], self.config['k_i_depth'], self.config['k_d_depth'])
        self.pid_yaw.reconfig_param(self.config['k_p_yaw'], self.config['k_i_yaw'], self.config['k_d_yaw'])

    def callback_params(self, params):
        for param in params:
            self.config[param.name] = param.value
        self.update_control_param()
        return SetParametersResult(successful=True)

    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(
            key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value

    def declare_and_set_params(self):
        self.config = {}
        self._declare_and_fill_map('k_p_depth', 0.0, "K P of depth", self.config)
        self._declare_and_fill_map('k_i_depth', 0.0, "K I of depth", self.config)
        self._declare_and_fill_map('k_d_depth', 0.0, "K D of depth", self.config)

        self._declare_and_fill_map('k_p_yaw', 0.0, "K P of yaw", self.config)
        self._declare_and_fill_map('k_i_yaw', 0.0, "K I of yaw", self.config)
        self._declare_and_fill_map('k_d_yaw', 0.0, "K D of yaw", self.config)

        self.update_control_param()


def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
