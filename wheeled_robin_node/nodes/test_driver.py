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
    #robot.control()
    #rospy.sleep(1.0)
    
    while not rospy.is_shutdown():
        #rospy.loginfo("Driving with -200 mm/s")
        #robot.drive(-200, 0)
        robot.sci.sensors(0)
        rospy.loginfo("Velocity vL:")
        a = robot.sci.ser.read(2)
        (e) = struct.unpack('>h',a)
        c = e
        printf('res: %r\n',c)
        #rospy.loginfo(a)
        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
