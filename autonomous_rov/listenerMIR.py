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
from time import sleep


# from waterlinked_a50_ros_driver.msg import DVL
# from waterlinked_a50_ros_driver.msg import DVLBeam

class MyPythonNode(Node):
    def __init__(self):
        super().__init__("listenerMIR")
        
        self.declare_parameter("run_initialization_test", False)
        self.run_initialization_test = self.get_parameter("run_initialization_test").value
        
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
        # self.addEndPoint()

        self.armDisarm(False)  # Not automatically disarmed at startup
        rate = 25  # 25 Hz
        self.setStreamRate(rate)
        # self.manageStabilize(False)

        self.subscriber()

        # set timer if needed
        timer_period = 0.05  # 50 msec - 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # variables
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
        
        #light values 
        self.light_pin = 13.0  # float between 0 and 15
        self.light_max = 1900.0
        self.light_min = 1100.0
        self.light = 1100.0
        self.light_int = 1100.0
        
        # camera servo 
        self.camera_servo_pin = 15.0 # float between 0 and 15
        self.servo_max = 1850.0
        self.servo_min = 1100.0
        self.tilt_int = 1450.0
        self.tilt = 1450.0

        ## Intail test for the system
        if self.run_initialization_test:
            self.initialization_test()
        
        # corrections for control
        self.Correction_yaw = 1500
        self.Correction_depth = 1500
        
        ## TODO ##
        # Task 1 : Calculate the flotability of the ROV
        self.flotability = None
        
    def initialization_test(self):
        """Tests the light by flashing it and tests the camera servo by moving it to max/min limits before starting the sytsem."""
        self.get_logger().info("Testing light and camera servo...")

        # Flash the light
        self.light = self.light_int
        self.send_servo_comand(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_max
        self.send_servo_comand(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_min
        self.send_servo_comand(self.light_pin, self.light)

        # Move the camera servo to max and min
        self.tilt = self.tilt_int
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_max
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_min
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.tilt_int  # Reset camera tilt to neutral
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        
        self.get_logger().info("Light and camera servo test completed.")  
        
    def send_servo_comand(self, pin_number, value):
        '''
        Sends a command to the navigator to adjust servo pins pwm using Mavros service
        pin_number (float) --> the servo number in the navigator board (13 for lights and 15 for camera servo)
        value (float) --> The pwm value sent to the servo between 1100 and 1900
        '''
        client = self.create_client(CommandLong, 'cmd/command')
        result = False
        while not result:
                result = client.wait_for_service(timeout_sec=4.0)
        # Create a request object for CommandLong service
        request = CommandLong.Request()

        # Set the parameters for the command (command 183: MAV_CMD_DO_SET_SERVO)
        request.command = 183       # Command 183: MAV_CMD_DO_SET_SERVO
        request.param1 = pin_number           # Servo number (param1)
        request.param2 = value         # Desired servo position (param2)
        request.param3 = 0.0             
        request.param4 = 0.0             

        # Send the service request and wait for the response
        future = client.call_async(request)

        # Check the result
        if future.result() is not None:
            self.get_logger().info('Change Completed')
        else:
            self.get_logger().error('Failed to preform the change ')
        
    
            
    def timer_callback(self):
        '''
        Time step at a fixed rate (1 / timer_period = 20 Hz) to execute control logic.
        '''
        if self.set_mode[0]:  # commands sent inside joyCallback()
            return
        elif self.set_mode[
            1]:  # Arbitrary velocity command can be defined here to observe robot's velocity, zero by default
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
            return
        elif self.set_mode[2]:
            # send commands in correction mode
            self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, 1500, 1500)
        else:  # normally, never reached
            pass

    def armDisarm(self, armed):
        """Arms or disarms the vehicle motors using MAVROS command 400."""
        cli = self.create_client(CommandLong, 'cmd/command')  # Create MAVROS service client
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)  # Wait for service to be available
            self.get_logger().info(f"{'Arming' if armed else 'Disarming'} requested, waiting for service: {result}")
        
        # Create request object for arming/disarming
        req = CommandLong.Request()
        req.broadcast = False  # Command is not broadcasted
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0  # No confirmation required
        req.param1 = 1.0 if armed else 0.0  # 1.0 = Arm, 0.0 = Disarm
        req.param2 = 0.0  
        req.param3 = 0.0  
        req.param4 = 0.0  
        req.param5 = 0.0  
        req.param6 = 0.0  
        req.param7 = 0.0 
        
        self.get_logger().info("Sending command...")
        resp = cli.call_async(req)  # Send command asynchronously
        
        # Log the result
        self.get_logger().info(f"{'Arming' if armed else 'Disarming'} Succeeded")

    # def manageStabilize(self, stabilized):
    #     # This functions sends a SetMode command service to stabilize or reset
    #     if (stabilized):
    #         traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #         cli = self.create_client(SetMode, 'set_mode')
    #         result = False
    #         while not result:
    #             result = cli.wait_for_service(timeout_sec=4.0)
    #             self.get_logger().info(
    #                 "stabilized mode requested, wait_for_service, (False if timeout) result :" + str(result))
    #         req = SetMode.Request()
    #         req.base_mode = 0
    #         req.custom_mode = "0"
    #         resp = cli.call_async(req)
    #         # rclpy.spin_until_future_complete(self, resp)
    #         self.get_logger().info("set mode to STABILIZE Succeeded")

    #     else:
    #         traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #         result = False
    #         cli = self.create_client(SetMode, 'set_mode')
    #         while not result:
    #             result = cli.wait_for_service(timeout_sec=4.0)
    #             self.get_logger().info(
    #                 "manual mode requested, wait_for_service, (False if timeout) result :" + str(result))
    #         req = SetMode.Request()
    #         req.base_mode = 0
    #         req.custom_mode = "19"
    #         resp = cli.call_async(req)
    #         # rclpy.spin_until_future_complete(self, resp)
    #         self.get_logger().info("set mode to MANUAL Succeeded")

    def setStreamRate(self, rate):
        ''' Set the Mavros rate for reading the senosor data'''
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

    # def addEndPoint(self):
    #     traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #     cli = self.create_client(EndpointAdd, 'mavros_router/add_endpoint')
    #     result = False
    #     while not result:
    #         result = cli.wait_for_service(timeout_sec=4.0)
    #         self.get_logger().info(
    #             "add endpoint requesRelAltCallbackted, wait_for_service, (False if timeout) result :" + str(result))

    #     req = EndpointAdd.Request()
    #     req.url = "udp://@localhost"
    #     req.type = 1  # TYPE_GCS
    #     resp = cli.call_async(req)
    #     rclpy.spin_until_future_complete(self, resp)
    #     self.get_logger().info("add endpoint rate Succeeded")

    def joyCallback(self, data):
        ''' Map the Joystick buttons according the bluerov configuration as descriped at
        https://bluerobotics.com/wp-content/uploads/2023/02/default-button-layout-xbox.jpg
        **Note: the lights are set to be in RT and LT button instead of the cross buttons'''
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button
        btn_manual_mode = data.buttons[3]  # Y button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_corrected_mode = data.buttons[0]  # A button
        btn_camera_servo_up = data.buttons[4] # LB button 
        btn_camera_servo_down = data.buttons[5] # RB button 
        btn_camera_rest = data.buttons[9] # R3 button 
        btn_light_down = data.axes[2] # LT button
        btn_light_up = data.axes[5] # RT button
        

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
            
        
        #### Control light intensity####
        if (btn_light_up == -1 and self.light < self.light_max):
            self.light = min(self.light + 100.0, self.light_max)
            self.send_servo_comand(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")
            
        elif (btn_light_down == -1 and self.light > self.light_min):
            self.light = max(self.light_min,self.light - 100)
            self.send_servo_comand(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")

        ### Control Camera tilt angle ###
        if (btn_camera_servo_up and not btn_camera_servo_down and self.tilt < self.servo_max):
            self.tilt = min(self.servo_max, self.tilt + 100)
            self.send_servo_comand(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_servo_down and self.tilt > self. servo_min):
            self.tilt = max(self.servo_min, self.tilt - 100)
            self.send_servo_comand(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_rest):
            self.tilt = self.tilt_int
            self.send_servo_comand(self.camera_servo_pin,self.tilt)
            self.get_logger().info(f"Camera tilt has been reseted")
            
            
            

    def velCallback(self, cmd_vel):
        ''' Used in manual mode to read the values of the analog and map it pwm then send it the thrusters'''
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
        
        # send the commands to the mthrusters 
        self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                             lateral_left_right)
        

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral):
        ''' This function replaces setservo for motor commands.
            It overrides Rc channels inputs and simulates motor controls.
            In this case, each channel manages a group of motors (DOF) not individually as servo set '''

        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)  # pulseCmd[4]--> pitch
        msg_override.channels[1] = np.uint(channel_roll)  # pulseCmd[3]--> roll
        msg_override.channels[2] = np.uint(channel_throttle)  # pulseCmd[2]--> heave
        msg_override.channels[3] = np.uint(channel_yaw)  # pulseCmd[5]--> yaw
        msg_override.channels[4] = np.uint(channel_forward)  # pulseCmd[0]--> surge
        msg_override.channels[5] = np.uint(channel_lateral)  # pulseCmd[1]--> sway
        msg_override.channels[6] = 1500 # camera pan servo motor speed 
        msg_override.channels[7] = 1500 #camers tilt servo motro speed

        self.pub_msg_override.publish(msg_override)

    def mapValueScalSat(self, value):
        ''' Map the value of the joysteck analog form -1 to 1 to a pwm value form 1100 to 1900
            where 1500 is the stop value 1100 is maximum negative and 1900 is maximum positive'''
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)

    def OdoCallback(self, data):
        ''' Read the Imu data angular velocities and angles and convert the angles from quaternion angles 
            to roll, pitch and yaw then publish them in sperate new topics '''
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

        angle = Twist()
        angle.angular.x = angle_wrt_startup[0]
        angle.angular.y = angle_wrt_startup[1]
        angle.angular.z = angle_wrt_startup[2]

        self.pub_angle_degre.publish(angle)

        # Extraction of angular velocity
        p = angular_velocity.x
        q = angular_velocity.y
        r = angular_velocity.z

        vel = Twist()
        vel.angular.x = p
        vel.angular.y = q
        vel.angular.z = r
        self.pub_angular_velocity.publish(vel)

        # Only continue if manual_mode is disabled
        if (self.set_mode[0]):
            return

        # Send PWM commands to motors
        # yaw command to be adapted using sensor feedback
        self.Correction_yaw = 1500

    def RelAltCallback(self, data):
        ## TODO ## 
        # Implement the control logic to maintain the vehicle at the same depth  
        # as when depth hold mode was activated (depth_p0).

        if (self.init_p0):
            # 1st execution, init
            self.depth_p0 = data
            self.init_p0 = False
        
        # set servo depth control here
        
        # update Correction_depth
        Correction_depth = 1500
        self.Correction_depth = int(Correction_depth)
        # Send PWM commands to motors in timer

    # def DvlCallback(self, data):
    #     u = data.velocity.x  # Linear surge velocity
    #     v = data.velocity.y  # Linear sway velocity
    #     w = data.velocity.z  # Linear heave velocity
    #     Vel = Twist()
    #     Vel.linear.x = u
    #     Vel.linear.y = v
    #     Vel.linear.z = w
    #     self.pub_linear_velocity.publish(Vel)


    def pingerCallback(self, data):
        self.pinger_distance = data.data[0]
        self.pinger_confidence = data.data[1]

    # self.get_logger().info("pinger_distance =" + str(self.pinger_distance))

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
       
        # self.sub = self.create_subscription(DVL, "/dvl/data", DvlCallback)
        self.subping = self.create_subscription(Float64MultiArray, "ping1d/data", self.pingerCallback,
                                                qos_profile=qos_profile)
        self.subping  # prevent unused variable warning

        self.get_logger().info("Subscriptions done.")


def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
