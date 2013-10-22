# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

import roslib.message
import struct
import logging
import time
import rospy

from math import radians
from wheeled_robin_driver import SENSOR_GROUP_PACKET_LENGTHS

#_struct_I = roslib.message.struct_I
_struct_Group0 = struct.Struct(">8hB5h2B")
_struct_Group1 = struct.Struct(">8hB")
_struct_Group7 = struct.Struct(">h")

def deserialize(msg, buff, timestamp, packed_id):
    """
    unpack serialized message in str into this message instance
    @param buff: byte array of serialized message
    @type  buff: str
    """
    try:
        _x = msg
	
	msg.header.stamp = rospy.Time.from_seconds(timestamp)        
	msg.linear_velocity = float(msg.linear_velocity[0]) /1000.
		
	# set everything to default values
	msg.linear_velocity = 0
	msg.angular_velocity = 0
	msg.pitch = 0
	msg.pich_rate = 0
	msg.left_wheel = 0
	msg.right_wheel = 0
	msg.left_wheel_rate = 0
	msg.right_wheel_rate = 0
	msg.mode = 0;
	msg.left_current = 0
	msg.right_current = 0
	msg.voltage = 0
	msg.imu_acc_x = 0
	msg.imu_acc_y = 0

	
	if packed_id == 0:
		(_x.linear_velocity, _x.angular_velocity, _x.pitch, _x.pich_rate, _x.left_wheel, _x.right_wheel, _x.left_wheel_rate, _x.right_wheel_rate, _x.mode, _x.left_current, _x.right_current, _x.voltage, _x.imu_acc_x, _x.imu_acc_y, _x.digital_outputs, _x.digital_inputs,) = _struct_Group0 .unpack(buff[0:SENSOR_GROUP_PACKET_LENGTHS[packet_id]])
		
		# do unit conversions
		msg.linear_velocity = float(msg.linear_velocity) /1000.
		msg.angular_velocity = float(msg.angular_velocity) /1000.
		msg.pitch = float(msg.pitch) /1000.
		msg.pich_rate = float(msg.pich_rate) /1000.
		msg.left_wheel = float(msg.left_wheel) /1000.
		msg.right_wheel = float(msg.right_wheel) /1000.
		msg.left_wheel_rate = float(msg.left_wheel_rate) /1000.
		msg.right_wheel_rate = float(msg.right_wheel_rate) /1000.
		msg.left_current = float(msg.left_current) /1000.
		msg.right_current = float(msg.right_current) /1000.
		msg.voltage = float(msg.voltage) /1000.
		msg.imu_acc_x = float(msg.imu_acc_x) /1000.
		msg.imu_acc_y = float(msg.linear_velocity) /1000.
	elif packed_id == 7:
		(_x.linear_velocity) = _struct_Group7 .unpack(buff[0:SENSOR_GROUP_PACKET_LENGTHS[packet_id]])
		
		# do unit conversions
		msg.linear_velocity = float(msg.linear_velocity[0]) /1000.
		msg.mode = 3 #this is just a workaround so that the robot works even if mode is not updated

        return msg
    except struct.error, e:
	raise roslib.message.DeserializationError(e)

class WheeledRobinSensorHandler(object):
    def __init__(self, robot):
        self.robot = robot

    def request_packet(self, packet_id):
        """Request a sensor packet."""
        with self.robot.sci.lock:
            self.robot.sci.flush_input()
            self.robot.sci.sensors(packet_id)
            #kwc: there appears to be a 10-20ms latency between sending the
            #sensor request and fully reading the packet.  Based on
            #observation, we are preferring the 'before' stamp rather than
            #after.
            stamp = time.time()
            length = SENSOR_GROUP_PACKET_LENGTHS[packet_id]
            return self.robot.sci.read(length), stamp

    def get_all(self, sensor_state):
        buff, timestamp = self.request_packet(0)
        if buff is not None:
            deserialize(sensor_state, buff, timestamp,0)
