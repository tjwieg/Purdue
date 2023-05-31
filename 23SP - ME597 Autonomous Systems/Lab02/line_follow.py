#!/usr/bin/env python3
import rospy
import time
from aut_sys.msg import motors, lines

def drive(l, r):
    # Motor saturation
    if l >  1.0: l =  1.0
    if r >  1.0: r =  1.0
    if l < -1.0: l = -1.0
    if r < -1.0: r = -1.0

    # Create and return motors() message
    output = motors()
    output.leftSpeed = l
    output.rightSpeed = r
    return(output)

def lineFollow(lines):
    if   lines.midLine:
        pubMotors.publish(drive(0.25, 0.25))
    elif lines.leftLine:
        pubMotors.publish(drive(0.0, 0.25))
    elif lines.rightLine:
        pubMotors.publish(drive(0.25, 0.0))
    else:
        pubMotors.publish(drive(0.1, 0.1))

def lab2_linefollow():
    rospy.Subscriber("lines", lines, lineFollow)
    rospy.init_node('uctronics', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
      pubMotors = rospy.Publisher('motors', motors, queue_size=1)
      lab2_linefollow()
    except rospy.ROSInterruptException:
	    pass
