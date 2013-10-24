#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('wheeled_robin_node')

import rospy
import sys
import time
import select
from math import sin, cos

from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from wheeled_robin_driver import WheeledRobin
from wheeled_robin_node.msg import WheeledRobinSensorState
from wheeled_robin_node.srv import SetWheeledRobinMode,SetWheeledRobinModeResponse, SetDigitalOutputs, SetDigitalOutputsResponse
from wheeled_robin_node.diagnostics import WheeledRobinDiagnostics
from wheeled_robin_node.wheeled_robin_sensor_handler import WheeledRobinSensorHandler;
from create_driver import DriverError
from create_node.covariances import \
     ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2

#dynamic reconfigure
import dynamic_reconfigure.server
from wheeled_robin_node.cfg import WheeledRobinConfig

class WheeledRobin_Node(object):
    _SENSOR_READ_RETRY_COUNT = 5

    def __init__(self, default_port='/dev/ttyUSB0', default_update_rate=500.0):

        """
        @param default_port: default tty port to use for establishing
        connection to WheeledRobin. This will be overriden by ~port ROS
        param if available.
        """
        self.default_port = default_port
        self.default_update_rate = default_update_rate

        self.robot = WheeledRobin()
        self.sensor_handler = None
        self.sensor_state = WheeledRobinSensorState()
        self.req_cmd_vel = None

        rospy.init_node('wheeled_robin')
        self._init_params()
        self._init_pubsub()
        
        self._pos2d = Pose2D() # 2D pose for odometry
        self._diagnostics = WheeledRobinDiagnostics()
        if self.has_gyro:
            #from create_node.gyro import TurtlebotGyro
            #self._gyro = TurtlebotGyro()
	    self._gyro = None
        else:
            self._gyro = None
            
        dynamic_reconfigure.server.Server(WheeledRobinConfig, self.reconfigure)

    def _init_params(self):
        self.port = rospy.get_param('~port', self.default_port)
        self.update_rate = rospy.get_param('~update_rate', self.default_update_rate)
        self.drive_mode = rospy.get_param('~drive_mode', 'drive')
        self.has_gyro = rospy.get_param('~has_gyro', False)
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.stop_motors_on_bump = rospy.get_param('~stop_motors_on_bump', True)
        self.min_abs_yaw_vel = rospy.get_param('~min_abs_yaw_vel', None)
        self.max_abs_yaw_vel = rospy.get_param('~max_abs_yaw_vel', None)
        self.publish_tf = rospy.get_param('~publish_tf', False)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_footprint_frame = rospy.get_param('~base_footprint_frame', 'base_footprint')
	self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')
        self.operate_mode = rospy.get_param('~operation_mode', 3)

        rospy.loginfo("serial port: %s"%(self.port))
        rospy.loginfo("update rate: %s"%(self.update_rate))
        rospy.loginfo("drive mode: %s"%(self.drive_mode))
        rospy.loginfo("has_gyro: %s"%(self.has_gyro))

    def _init_pubsub(self):
        self.joint_states_pub = rospy.Publisher('joint_states', JointState)
        self.odom_pub = rospy.Publisher('odom', Odometry)

        self.sensor_state_pub = rospy.Publisher('~sensor_state', WheeledRobinSensorState)
        self.operating_mode_srv = rospy.Service('~set_operation_mode', SetWheeledRobinMode, self.set_operation_mode)
        self.digital_output_srv = rospy.Service('~set_digital_outputs', SetDigitalOutputs, self.set_digital_outputs)

        if self.drive_mode == 'direct_drive':
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
            self.drive_cmd = self.robot.direct_drive
        elif self.drive_mode == 'drive':
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
            self.drive_cmd = self.robot.drive
        else:
            rospy.logerr("unknown drive mode :%s"%(self.drive_mode))

        self.transform_broadcaster = None
        if self.publish_tf:
            self.transform_broadcaster = tf.TransformBroadcaster()

    def reconfigure(self, config, level):
        self.update_rate = config['update_rate']
        self.drive_mode = config['drive_mode']
        self.has_gyro = config['has_gyro']
        if self.has_gyro:
            self._gyro.reconfigure(config, level)
        self.odom_angular_scale_correction = config['odom_angular_scale_correction']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.cmd_vel_timeout = rospy.Duration(config['cmd_vel_timeout'])
        self.stop_motors_on_bump = config['stop_motors_on_bump']
        self.min_abs_yaw_vel = config['min_abs_yaw_vel']
        self.max_abs_yaw_vel = config['max_abs_yaw_vel']
        return config

    def start(self):
        log_once = True
        while not rospy.is_shutdown():
            try:
                self.robot.start(self.port)
                break
            except serial.serialutil.SerialException as ex:
                msg = "Failed to open port %s. Please make sure the Wheeled_Robin cable is plugged into the computer. \n"%(self.port)
                self._diagnostics.node_status(msg,"error")
                if log_once:
                    log_once = False
                    rospy.logerr(msg)
                else:
                    sys.stderr.write(msg)
                time.sleep(1.0)

        self.sensor_handler = WheeledRobinSensorHandler(self.robot);
        self.robot.safe = True

        self.robot.control()

        # Startup readings from WheeledRobin can be incorrect, discard first values
        s = WheeledRobinSensorState()
        try:
            self.sense(s)
        except Exception:
            # packet read can get interrupted, restart loop to
            # check for exit conditions
            pass

    def spin(self):

        # state
        s = self.sensor_state
        odom = Odometry(header=rospy.Header(frame_id=self.odom_frame), child_frame_id=self.base_footprint_frame)
        js = JointState(name = ["left_wheel_joint", "right_wheel_joint"], position=[0,0], velocity=[0,0], effort=[0,0])

        r = rospy.Rate(self.update_rate)
        last_cmd_vel = 0, 0
        last_cmd_vel_time = rospy.get_rostime()
        last_js_time = rospy.Time(0)
        # We set the retry count to 0 initially to make sure that only
        # if we received at least one sensor package, we are robust
        # agains a few sensor read failures. For some strange reason,
        # sensor read failures can occur when switching to full mode
        # on the Roomba.
        sensor_read_retry_count = 0


        while not rospy.is_shutdown():
            last_time = s.header.stamp
            curr_time = rospy.get_rostime()

            # SENSE/COMPUTE STATE
            try:
                self.sense(s)
                transformFootprintBase = self.compute_odom(s, last_time, odom)
                # Future-date the joint states so that we don't have
                # to publish as frequently.
                js.header.stamp = curr_time + rospy.Duration(1)
            except select.error:
                # packet read can get interrupted, restart loop to
                # check for exit conditions
                continue

            except DriverError:
                if sensor_read_retry_count > 0:
                    rospy.logwarn('Failed to read sensor package. %d retries left.' % sensor_read_retry_count)
                    sensor_read_retry_count -= 1
                    continue
                else:
                    raise
            sensor_read_retry_count = self._SENSOR_READ_RETRY_COUNT

            # PUBLISH STATE
            self.sensor_state_pub.publish(s)
            self.odom_pub.publish(odom)
            if self.publish_tf:
                self.publish_odometry_transform(odom)
		self.transform_broadcaster.sendTransform(transformFootprintBase)
		
            # publish future-dated joint state at 1Hz
            if curr_time > last_js_time + rospy.Duration(1):
                self.joint_states_pub.publish(js)
                last_js_time = curr_time
            self._diagnostics.publish(s, self._gyro)
            #if self._gyro:
	    #    self._gyro.publish(s, last_time)

            # WRITE DESIRED VELOCITY
            if self.req_cmd_vel is not None:
                # check for velocity command and set the robot into full mode if necessary
                if s.mode != self.operate_mode:
                    if self.operate_mode == 2:
                        self._robot_run_safe()
                    else:
                        self._robot_run_full()

                # do some checks on the velocity : TODO
                req_cmd_vel = self.req_cmd_vel

                # Set to None so we know it's a new command
                self.req_cmd_vel = None
                # reset time for timeout
                last_cmd_vel_time = last_time

            else:
                #zero commands on timeout
                if last_time - last_cmd_vel_time > self.cmd_vel_timeout:
                    last_cmd_vel = 0,0
		    rospy.loginfo("A Timout in cmd_vel occured")

                # do some checks on the velocity : TODO  --> check bumpers
                req_cmd_vel = last_cmd_vel

            # send command
            self.drive_cmd(*req_cmd_vel)
            # record command
            last_cmd_vel = req_cmd_vel

            r.sleep()

    def _robot_run_passive(self):
        """
        Set WheeldRobin into passive run mode
        """
        rospy.loginfo("Setting WheeldRobin to passive mode.")
        #setting all the digital outputs to 0
        self._set_digital_outputs([0, 0, 0, 0, 0, 0, 0, 0])
        self.robot.passive()

    def _robot_run_safe(self):
        """
        Set WheeldRobin into safe run mode
        """
        rospy.loginfo("Setting WheeldRobin to safe mode.")
        self.robot.safe = True
        self.robot.control()
        b1 = (self.sensor_state.digital_inputs & 2)/2
        b2 = (self.sensor_state.digital_inputs & 4)/4
        self._set_digital_outputs([1, b1, b2, 0, 0, 0, 0, 0])

    def _robot_run_full(self):
        """
        Set WheeldRobin into full run mode
        """
        rospy.loginfo("Setting WheeldRobin to full mode.")
        self.robot.safe = False
        self.robot.control()
        b1 = (self.sensor_state.digital_inputs & 2)/2
        b2 = (self.sensor_state.digital_inputs & 4)/4
        self._set_digital_outputs([1, b1, b2, 0, 0, 0, 0, 0])

    def _set_digital_outputs(self, outputs):
        assert len(outputs) == 8, 'Expecting 8 output states.'
        byte = 0
        for output, state in enumerate(outputs):
            byte += (2 ** output) * int(state)
        self.robot.set_digital_outputs(byte)
        #self.sensor_state.digital_outputs = byte

    def set_digital_outputs(self,req):
        if not self.robot.sci:
            rospy.logwarn("WheeledRobin: robot not connected yet!")
            return SetDigitalOutputsResponse(False)
            
        outputs = [req.digital_out_0,req.digital_out_1, req.digital_out_2, req.digital_out_3, req.digital_out_4, req.digital_out_5, req.digital_out_6, req.digital_out_7]
        self._set_digital_outputs(outputs)
        return SetDigitalOutputsResponse(True)
	
    def set_operation_mode(self,req):
        if not self.robot.sci:
            rospy.logwarn("WheeledRobin: robot not connected yet!")
            return SetTurtlebotModeResponse(False)

        self.operate_mode = req.mode

        if req.mode == 1: #passive
            self._robot_run_passive()
        elif req.mode == 2: #safe
            self._robot_run_safe()
        elif req.mode == 3: #full
            self._robot_run_full()
        else:
            rospy.logerr("Requested an invalid mode.")
            return SetTurtlebotModeResponse(False)
        return SetTurtlebotModeResponse(True)

    def sense(self, sensor_state):
        self.sensor_handler.get_all(sensor_state)
        #if self._gyro:
        #    self._gyro.update_calibration(sensor_state)
	pass

    def cmd_vel(self, msg):
        # Clamp to min abs yaw velocity, to avoid trying to rotate at low speeds, which doesn't work well.
        if self.min_abs_yaw_vel is not None and msg.angular.z != 0.0 and abs(msg.angular.z) < self.min_abs_yaw_vel:
            msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
        # Limit maximum yaw
        if self.max_abs_yaw_vel is not None and self.max_abs_yaw_vel > 0.0 and msg.angular.z != 0.0 and abs(msg.angular.z) > self.max_abs_yaw_vel:
            msg.angular.z = self.max_abs_yaw_vel if msg.angular.z > 0.0 else -self.max_abs_yaw_vel

        if self.drive_mode == 'direct_drive':
            # TODO 
            # convert twist to direct_drive args
            #ts = msg.linear.x * 1000 # m -> mm
            #tw = msg.angular.z * (robot_types.ROBOT_TYPES[self.robot_type].wheel_separation / 2) * 1000
            # Prevent saturation at max wheel speed when a compound command is sent.
            #if ts > 0:
            #    ts = min(ts, MAX_WHEEL_SPEED - abs(tw))
            #else:
            #    ts = max(ts, -(MAX_WHEEL_SPEED - abs(tw)))
            #self.req_cmd_vel = int(ts - tw), int(ts + tw)
            self.req_cmd_vel = msg.linear.x * 1000, msg.angular.z * 1000
        elif self.drive_mode == 'drive':
            # convert twist to drive args, m->mm (velocity_x, velocity_g)
            self.req_cmd_vel = msg.linear.x * 1000, msg.angular.z * 1000

    def compute_odom(self, sensor_state, last_time, odom):
        """
        Compute current odometry.  Updates odom instance and returns tf
        transform. compute_odom() does not set frame ids or covariances in
        Odometry instance.  It will only set stamp, pose, and twist.

        @param sensor_state: Current sensor reading
        @type  sensor_state: WheeledRobinSensorState
        @param last_time: time of last sensor reading
        @type  last_time: rospy.Time
        @param odom: Odometry instance to update.
        @type  odom: nav_msgs.msg.Odometry

        @return: transform
        @rtype: ( (float, float, float), (float, float, float, float) )
        """
        # based on otl_roomba by OTL <t.ogura@gmail.com>

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()

	delta_x = sensor_state.linear_velocity * cos(self._pos2d.theta) * dt;
	delta_y = sensor_state.linear_velocity * sin(self._pos2d.theta) * dt;
	delta_th = sensor_state.angular_velocity  * dt;
	
        self._pos2d.x += delta_x
        self._pos2d.y += delta_y
        self._pos2d.theta += delta_th

        # Turtlebot quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))

        # construct the transform
        # transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(sensor_state.linear_velocity, 0, 0), Vector3(0, 0, sensor_state.angular_velocity))
        #if sensor_state.requested_right_velocity == 0 and \
        #       sensor_state.requested_left_velocity == 0 and \
	#       sensor_state.distance == 0:
        #    odom.pose.covariance = ODOM_POSE_COVARIANCE2
	#odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        #else:
	#odom.pose.covariance = ODOM_POSE_COVARIANCE
        #    odom.twist.covariance = ODOM_TWIST_COVARIANCE

	odom.pose.covariance = ODOM_POSE_COVARIANCE
        odom.twist.covariance = ODOM_TWIST_COVARIANCE
	
        # construct the transform from footprint to base
	base_quat = (0., sin(sensor_state.pitch/2.), 0., cos(sensor_state.pitch/2.))
        transformFootprintBase = (0,0,0.055), base_quat

        return transformFootprintBase

    def publish_odometry_transform(self, odometry):
        self.transform_broadcaster.sendTransform(
            (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z),
            (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
             odometry.pose.pose.orientation.w),
             odometry.header.stamp, odometry.child_frame_id, odometry.header.frame_id)

def wheeled_robin_main(argv):
    node = WheeledRobin_Node()
    while not rospy.is_shutdown():
        try:
            # This sleep throttles reconnecting of the driver. It
            # appears that pyserial does not properly release the file
            # descriptor for the USB port in the event that the Create is
            # unplugged from the laptop. This file desecriptor prevents
            # the create from reassociating with the same USB port when it
            # is plugged back in. The solution, for now, is to quickly
            # exit the driver and let roslaunch respawn the driver until
            # reconnection occurs. However, it order to not do bad things
            # to the Create bootloader, and also to keep relaunching at a
            # minimum, we have a 3-second sleep.
            time.sleep(1.0)
            
            node.start()
            node.spin()

        except Exception as ex:
            msg = "Failed to contact device with error: [%s]. Please check that the Wheeled_Robin is powered on and that the connector is plugged into the Wheeled_Robin."%(ex)
            node._diagnostics.node_status(msg,"error")
            rospy.logerr(msg)

        finally:
            try: pass
            except Exception: pass


if __name__ == '__main__':
    wheeled_robin_main(sys.argv)