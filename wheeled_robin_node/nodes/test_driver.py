#!/usr/bin/env python
import rospy
import struct
import sys
from std_msgs.msg import String
from wheeled_robin_driver import WheeledRobin
#from create_driver import SerialCommandInterface

def printf(format, *args):
    sys.stdout.write(format % args)

def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    
    robot = WheeledRobin()
    #serial = SerialCommandInterface('/dev/ttyUSB0', 38400)
    
    robot.start()
    #rospy.sleep(2.5)
    #robot.sci.start()
    robot.control()
    rospy.sleep(0.5)
 
    #robot.sci.control_params(180,18,110,50,50,30)
    #rospy.sleep(1.0)
    #robot.control()
    #robot.drive(130,0)
    #rospy.sleep(3.0)
    #robot.drive(0,0)
    #rospy.sleep(5.0)
    #robot.drive(-130,0)
    #rospy.sleep(3.0)
    #robot.drive(0,0)
    #rospy.sleep(1.0)
    #robot.sci.start()
    #r = rospy.Rate(100)   
    #while not rospy.is_shutdown():
        #rospy.loginfo("Driving with -200 mm/s")
    #robot.drive(0,1000)
    #rospy.sleep(1.0)
    #robot.drive(0,0)
    #rospy.sleep(1.0)
    #robot.drive(0,-1000)
    #rospy.sleep(1.0)
    #robot.drive(0,0)
        #rospy.loginfo(" ")
        #robot.sci.sensors(1)
        #a = robot.sci.ser.read(17)
        #robot.sci.flush_input()
        #rospy.sleep(0.5)
        #robot.sci.sensors(22)
        #b = robot.sci.ser.read(1)
        # PacketID:0
        #(e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16) = struct.unpack('>8h1B5h2B',a)
	#printf('res: %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r\n',e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16)
        # PacketID:1
        #(e1,e2,e3,e4,e5,e6,e7,e8,e9) = struct.unpack('>8h1B',a)
        #printf('res: %r, %r, %r, %r, %r, %r, %r, %r, %r\n',e1,e2,e3,e4,e5,e6,e7,e8,e9)
        # PacketID:7-22
	#(e2) = struct.unpack('>h',a)
	#printf('res: %r\n', e2)
	#rospy.loginfo(a)
        #r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
