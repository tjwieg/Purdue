#!/usr/bin/env python3
import rospy
import time
from aut_sys.msg import motors, distance

startTime = 0
began = False
def e1_read(dist):

   global startTime
   if startTime == 0:
      startTime = time.time()

   timeSpent = time.time() - startTime


#   path = [(0.5,0.5), (-0.5,0.5), (0.5,0.5),(-0.5,0.5), (0.5,0.5), (-0.5,0.5), (0.5,0.5), (-0.5,0.5), (0,0), (0,0), (0,0)]
 #  times = [1, 2, 2, 4, 2, 4, 2, 4, 2, 4] # for part 1
 
   path = [(0.5,0.5), (0.5,-0.5),  (0.5,0.5), (0.5,-0.5),  (0.5,0.5),  (0.5,-0.5), (0.5,0.5), (0.5,-0.5), (0,0), (0,0), (0,0)]
   times = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2]

   speed = (0,0)
   timeSum = 0
   pathNum = 0
   i = 0
   while i < len(path)-1:
      timeSum += times[i]
      if timeSum > timeSpent:
         break
      i+=1
   
   if True: # loop
      if i == len(path)-1:
         startTime = 0
   if abs(path[i][0]- path[i][1]) > 0.2:
      speed = (path[i][0]*1, path[i][1]*1)
   else:
      speed = (path[i][0]*0.7, path[i][1]*0.7)
   rospy.loginfo(f"speed {speed}.")

   output = motors()
   output.rightSpeed, output.leftSpeed = speed
   #rospy.loginfo(f"Wall {round(dist.distance,4)}m away. Driving {vel} power.")
   pubLeft.publish(output)

def e1_drive():
   rospy.Subscriber("distance", distance, e1_read)
   rospy.init_node('uctronics', anonymous=True)
   rospy.spin()


if __name__ == '__main__':
    try:
      pubLeft=rospy.Publisher('motors', motors, queue_size=1)
      e1_drive()
    except rospy.ROSInterruptException:
	    pass

