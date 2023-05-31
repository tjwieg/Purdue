#!/usr/bin/env python3
import rospy
import time
from aut_sys.msg import motors, lines

lastOrientation = 0

def drive(l, r):
    # Motor saturation
    l = min(1, max(-1, l))
    r = min(1, max(-1, r))

    # Create and return motors() message
    output = motors()
    output.leftSpeed = l
    output.rightSpeed = r
    return(output)

def lineFollow(lines):
    global lastOrientation

    if lines.midLine:
        pubMotors.publish(drive(0.25, 0.25))
        lastOrientation = 0
        
    elif lines.leftLine:
        pubMotors.publish(drive(0.0, 0.25))
        lastOrientation = 1
    elif lines.rightLine:
        pubMotors.publish(drive(0.25, 0.0))
        lastOrientation = -1

    else:
        if lastOrientation == -1:
            pubMotors.publish(drive(0.25, 0))
        elif lastOrientation == 1:
            pubMotors.publish(drive(0, 0.25))
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
