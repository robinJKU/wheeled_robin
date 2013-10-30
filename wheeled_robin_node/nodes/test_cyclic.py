#!/usr/bin/env python
import rospy
import struct
import sys
from std_msgs.msg import String
from wheeled_robin_driver import WheeledRobin

def printf(format, *args):
    sys.stdout.write(format % args)

def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    
    robot = WheeledRobin()
    
    robot.start()
    rospy.sleep(1)
    #robot.control()
    robot.sci.safe()
    #rospy.sleep(2)
    #robot.sci.full()
    rospy.sleep(2)
 

    r = rospy.Rate(10)   
    while not rospy.is_shutdown():
	robot.drive(0,0)
        robot.sci.flush_input()
	robot.sci.sensors(7)
        #a = robot.sci.ser.read(17)
	a = robot.sci.ser.read(2)
	#(e1,e2,e3,e4,e5,e6,e7,e8,e9) = struct.unpack('>8hB', a[0:17])
	(e1) = struct.unpack('>h', a)
	printf('res: %r \n',e1)
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
