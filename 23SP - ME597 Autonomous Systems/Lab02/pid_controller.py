#!/usr/bin/env python3
import rospy
import time
from aut_sys.msg import motors, distance

lastError = 0
integralError = 0
targetSpeed = 0
lastCalcTime = 0

def reader_node(dist):

   global lastError
   global integralError
   global targetSpeed
   global lastCalcTime

   if lastCalcTime == 0:
      lastCalcTime = time.time()-0.1
   duration = time.time() - lastCalcTime

   rospy.loginfo('Received ctr:{}'.format(dist.distance))

   idealDist = 0.2
   kp = 1.4
   ki = 0.01
   kd = 0.0

   proportionalError = dist.distance - idealDist
   integralError += proportionalError * duration
   derivativeError = (lastError - proportionalError) / duration

   targetSpeed = kp * proportionalError + ki * integralError + kd * derivativeError


   rospy.loginfo('target speed:{}'.format(targetSpeed))

   lastError = proportionalError
   integralError *= 0.9

def controller_node():
   # Create a publisher object
   rospy.Subscriber("distance", distance, reader_node)
   rospy.loginfo('starting')

   # Declare the node, and register it with a unique name
   rospy.init_node('controller', anonymous=True)
   # Define the execution rate object (10Hz)
   rate = rospy.Rate(10)
   '''
   This is the main node loop
   '''
   while not rospy.is_shutdown():
      # Create message object with a specific type
      rospy.loginfo('targetSpeed: {}'.format(targetSpeed))

      output = motors()
      maxSpeed = 0.5
      output.rightSpeed, output.leftSpeed = min(max(targetSpeed,-maxSpeed), maxSpeed), min(max(targetSpeed,-maxSpeed), maxSpeed)
      
      rospy.loginfo('speed: {}'.format(output.leftSpeed))

      pubMotor.publish(output)

      # Sleep the necessary amount of time to keep a 10Hz execution rate
      rate.sleep()


if __name__ == '__main__':
    try:
      pubMotor=rospy.Publisher('motors', motors, queue_size=1)
      controller_node()
    except rospy.ROSInterruptException:
            pass
