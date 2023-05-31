#!/usr/bin/env python3
import os
os.environ["NUMPY_EXPERIMENTAL_ARRAY_FUNCTION"] = "0"

import rospy, cv2, rospkg
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion, quaternion_from_euler

tau = 2 * np.pi
# convert from +/- pi reference to 2pi reference
def circulizer(angle):
    # positive angles
    if angle > 0:
        while angle >= tau: angle = angle - tau
        return(angle)
    # negative/zero angles
    else:
        while angle <= -tau: angle = angle + tau
        if angle == 0: return(0)
        else: return(angle + tau)

# convert from 2pi reference to +/- pi reference
def decirculizer(angle):
    angle = circulizer(angle)
    if angle > np.pi: return(angle - tau)
    else: return(angle)

# Low-pass filter for a circular 360-tuple
def circLPF(data: tuple):
    circles = data + data + data
    output = []
    for i in range(360):
        output.append(np.mean(circles[((i+360)-2):((i+360)+3)]))
    return(output)

class MapCell:
    def __init__(self, coord: tuple, source: tuple, weight = 1) -> None:
        self.coord = coord      # map coordinates    
        self.source = source    # map coordinates
        self.weight = weight    # a numeric giving cost from source to coord
    def __str__(self) -> str:
        return(f"({self.coord[0]},{self.coord[1]})")    
    def aslist(self):
        return([
            self.coord[0],  # 0
            self.coord[1],  # 1
            self.source[0], # 2
            self.source[1], # 3
            self.weight     # 4
        ])

class MapQueue:
    def __init__(self, cells: list, goal: tuple) -> None:
        # cells is a list of MapCells
        # goal is map coordinate of end goal
        N = len(cells)
        self.goal = goal
        output = np.zeros((N,8))
        for i in range(N):
            newcell = cells[i].aslist()
            dist = ((self.goal[0] - newcell[0])**2 +
                    (self.goal[1] - newcell[1])**2)
            newcell = newcell + [dist, dist + newcell[4], 0]
            output[i,] = newcell
        # queue columns are: coord (0,1), source (2,3), weight (4),
        #                    dist (5), rank (6), and tombstone (7)
        self.__hsort(output)
    
    # Sort array (default: self.queue) by rank, ascending (ignoring tombstones)
    def __hsort(self, array) -> None:
        self.queue = array[np.lexsort((array[:,6], array[:,7]))]

    def __len__(self) -> int:
        return(len(self.queue))
    def __str__(self) -> str:
        return(np.array2string(self.queue))

    # Inserts newcell into queue at appropriate rank (ignores downgrades)
    def add(self, newcell: MapCell) -> None:
        if newcell.coord == self.goal:
            newcell = newcell.aslist()
            newcell = newcell + [0,0,0] # Zero Rank = maximum priority
            self.queue = np.array(self.queue.tolist() + [newcell])
        else:
            # Update an existing cell in queue
            search = ((self.queue[:,0] == newcell.coord[0]) * 
                      (self.queue[:,1] == newcell.coord[1]))
            if np.any(search):
                row = np.where(search)[0].squeeze()
                if self.queue[row,7] != 0:
                    pass # ignore if tombstoned
                elif self.queue[row,4] > newcell.weight:
                    dist = self.queue[row,5]
                    newcell = newcell.aslist()
                    newcell = newcell + [dist, dist + newcell[4], 0]
                    self.queue[row] = newcell
            # Add a novel cell to queue
            else:
                newcell = newcell.aslist()
                dist = ((self.goal[0] - newcell[0])**2 +
                        (self.goal[1] - newcell[1])**2)
                newcell = newcell + [dist, dist + newcell[4], 0]
                self.queue = np.array(self.queue.tolist() + [newcell])
    
    # Remove an existing cell from the queue
    def remove(self, coord: tuple) -> None:
        if type(coord) == MapCell: coord = coord.coord
        search = ((self.queue[:,0] == coord[0]) * 
                  (self.queue[:,1] == coord[1]))
        if np.any(search):
            self.queue[np.where(search),7] = 1
    
    # Get a list of tombstoned cells
    def explored(self) -> list:
        tombstoned = self.queue[self.queue[:,7] != 0]
        output = []
        for i in range(len(tombstoned)):
            output.append((int(tombstoned[i,0]),int(tombstoned[i,1])))
        return(output)

    # Retrieve the next cell in queue (None if queue empty)
    def next(self, idx = 0, showTombstone = False) -> MapCell:
        self.__hsort(self.queue)
        if idx >= len(self.queue):
            raise IndexError
        elif (self.queue[idx,7] != 0) and (not showTombstone):
            rospy.logerr("Queue empty!")
            return None
        else:
            outcell = MapCell(
                coord = (int(self.queue[idx,0]), int(self.queue[idx,1])),
                source = (int(self.queue[idx,2]), int(self.queue[idx,3])),
                weight = self.queue[idx,4]
            )
            return(outcell)
    
    # Get a cell by coordinate
    def get(self, coord: tuple) -> MapCell:
        if type(coord) == MapCell: coord = coord.coord
        search = ((self.queue[:,0] == coord[0]) * 
                  (self.queue[:,1] == coord[1]))
        if np.any(search):
            row = int(np.where(search)[0])
            return(self.next(idx = row, showTombstone = True))
        else:
            rospy.logerr(f"Cell {coord} not found!")
            return None

class Explorer:
    def __init__(self) -> None:
        # Some constants
        self.fast = 0.5
        self.med = 0.25
        self.slow = 0.1
        
        self.start = None
        self.wdflag = False
        self.stop = False
        self.update = False
    
    # Set up nodes
    def initialize(self) -> None:
        rospy.init_node('explore_node', anonymous=True)

        # Initial values for explorer
        self.goal = (0,0)
        self.ori = None

        # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.mapMaker)
        rospy.Subscriber("/clock", Clock, self.timeout)
        rospy.Subscriber("/odom", Odometry, self.localizer)
        rospy.Subscriber("global_plan", Path, self.explore)
        rospy.Subscriber("plan_ping", Bool, self.planner)
        rospy.Subscriber("/scan", LaserScan, self.wallstop)
        
        # Publishers
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        self.pathpub = rospy.Publisher("global_plan", Path, queue_size = 1)
        self.pingmap = rospy.Publisher("plan_ping", Bool, queue_size = 1)

    # Update internal map data
    def mapMaker(self, data: OccupancyGrid) -> None:
        # Save metadata
        self.cols = data.info.width
        self.rows = data.info.height
        self.res = data.info.resolution
        self.ori = data.info.origin
        
        # Save map image & unexplored areas
        self.map = np.array(data.data, np.int8).reshape((self.rows, self.cols))
        self.unexplored = np.where(self.map == -1)
        
        # Boost contrast and convert to uint8
        self.map[self.map < 0] = 0
        self.map = self.map.astype(np.uint8)
        self.map[self.map > 0] = 255
        
        # Inflate boundaries
        kernel = np.ones((11,11), np.uint8)
        self.map = cv2.dilate(self.map, kernel, iterations = 1)
        rospy.logwarn("Updating map!")
        self.pingmap.publish(True)
    
    # Prevent running into walls
    def wallstop(self, data: LaserScan) -> None:
        def crash():
            self.stop = True
            rospy.logwarn("Avoiding obstacle!")

        fRanges = circLPF(data.ranges)
        dir = np.argmin(fRanges)
        if ((dir < 30) or (dir > 330)) and fRanges[dir] < 0.3:
            crash()
            self.drive(-self.slow)
        elif (dir > 40) and (dir < 50) and fRanges[dir] < 0.4:
            crash()
            self.drive(self.slow, -self.slow)
        elif (dir > 310) and (dir < 320) and fRanges[dir] < 0.4:
            crash()
            self.drive(self.slow, self.slow)
        
    # Update route plan
    def planner(self, data: Bool) -> None:
        # Publish new path
        if self.ori != None:
            path, explored = self.astar((self.r, self.c), self.goal)

            self.map[self.unexplored] = 64

            if explored != None:
                for cell in explored:
                    self.map[cell[0],cell[1]] = 128
            
            if path != None:
                output = []
                for i in range(len(path)-1):
                    self.map[path[i][0], path[i][1]] = 192
                    # Get coordinates & heading
                    xy = self.__cell2xy(path[i])
                    nxy = self.__cell2xy(path[i+1])
                    dx, dy = nxy[0] - xy[0], nxy[1] - xy[1]
                    heading = np.arctan2(dy, dx)
                    
                    # Create posestamped and add to list
                    addpose = PoseStamped()
                    addpose.pose = self.__xy2pose(xy, heading)
                    output.append(addpose)
                # Add final pose
                addpose = PoseStamped()
                addpose.pose = self.__xy2pose(nxy, heading)
                output.append(addpose)
                
                # Publish path of poses
                path = Path()
                path.header.frame_id = "map"
                path.poses = output
                rospy.logwarn("Publishing a new path!")
                self.pathpub.publish(path)
                self.newpath = True
                self.stop = False

            cv2.imshow("Map", self.map)
            cv2.waitKey(1)
        else:
            rospy.logwarn("No map origin found!")
    
    # Convert a map coordinate to an XY coordinate
    def __cell2xy(self, coord: tuple) -> tuple:
        # coord is map coordinate (row,col)
        x = (coord[1] * self.res) + self.ori.position.x
        y = (coord[0] * self.res) + self.ori.position.y
        return( (x,y) )
    
    # Convert an XY coordinate to a map coordinate
    def __xy2cell(self, xy: tuple) -> tuple:
        # xy is an XY coordinate (x,y)
        c = round((xy[0] - self.ori.position.x)/self.res)
        r = round((xy[1] - self.ori.position.y)/self.res)
        return( (r,c) )
    
    # Convert an XY coordinate and heading to a pose
    def __xy2pose(self, xy: tuple, heading: float) -> PoseStamped:
        output = Pose()

        output.position.x = xy[0]
        output.position.y = xy[1]

        heading = decirculizer(heading)
        ex, ey, ez, ew = quaternion_from_euler(0,0,heading)
        output.orientation.x = ex
        output.orientation.y = ey
        output.orientation.z = ez
        output.orientation.w = ew
        
        return(output)

    # Get a list of map coordinates for unoccupied/unexplored neighbors
    def neighbors(self, cell: tuple) -> list:
        # cell is a map coordinate
        row, col = cell
        nlist = []
        for r in range(max(0,row-2), min(row+3,self.rows),2):
            for c in range(max(0,col-2), min(col+3,self.cols),2):
                if not (r == row and c == col):
                    if self.map[r,c] < 50:
                        nlist.append((r,c))
        return(nlist)
    
    # Get a list of map coordinates giving a path from start to end
    def astar(self, start: tuple, end: tuple) -> list:
        # start and end are map coordinates (row,col)
        self.wdflag = True
        
        # Initialize queue
        nn = self.neighbors(start)
        if len(nn) > 0:
            newcells = []
            for coord in nn:
                newcells.append(MapCell(coord,start))
            queue = MapQueue(newcells, end)
            node = queue.next()

            # Search through map via A* algorithm
            goal = False
            self.update = False
            while not (goal or self.update):
                # Search neighbors of current node
                nn = self.neighbors(node.coord)
                for coord in nn:
                    weight = node.weight + 1
                    newcell = MapCell(coord, node.coord, weight)
                    queue.add(newcell)
                queue.remove(node.coord)

                # Pick next node to explore
                node = queue.next()
                if node == None:
                    rospy.logerr("*** Path not found!")
                    output = (None, queue.explored())
                    return output
                elif node.coord == end:
                    rospy.logwarn("Path created successfully!")
                    goal = True

            # Generate path from queue
            path = [node.coord]
            goal = False
            while not (goal or self.update):
                node = queue.get(node.source)
                path.append(node.coord)
                if node.source == start:
                    path.append(node.source)
                    goal = True
            path.reverse()
            if self.update:
                rospy.logerr("Breaking A* loop")
                return(None, None)
            return (path, queue.explored())
        else: return(None, None)

    # Updates current map coordinates and heading
    def localizer(self, data: Odometry) -> None:
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        
        ox = data.pose.pose.orientation.x
        oy = data.pose.pose.orientation.y
        oz = data.pose.pose.orientation.z
        ow = data.pose.pose.orientation.w
        _,_, yaw = euler_from_quaternion([ox, oy, oz, ow])

        if not self.ori == None:
            self.r, self.c = self.__xy2cell((self.x, self.y))
        else: self.r, self.c = 200,200
        self.heading = circulizer(yaw)
    
    # Wrapper for motor publisher
    def drive(self, speed: float, dir = 0.0) -> None:
        output = Twist()
        output.linear.x = speed
        output.angular.z = dir
        self.cmd_vel.publish(output)
    
    # Point a given direction
    def point(self, heading: float) -> None:
        offset = heading - self.heading
        while abs(offset) > 0.02:
            while offset < -np.pi:
                offset = offset + (2*np.pi)
            self.drive(0, 2*offset)
            offset = heading - self.heading
            if self.newpath: break

    # Linear navigator
    def driveto(self, coord: tuple) -> None:
        # Get XY coordinates & heading
        newx, newy = self.__cell2xy(coord)
        dx, dy = newx - self.x, newy - self.y
        speed = dx**2 + dy**2

        # Go there, if we aren't already there
        if (speed > 0.02) and (not self.stop):
            # Point in the right direction
            heading = circulizer(np.arctan2(dy, dx))
            self.point(heading)
            
            # Drive in right direction
            while speed > 0.02:
                if self.stop: break
                self.drive(2*speed)
                dx, dy = newx - self.x, newy - self.y
                speed = dx**2 + dy**2

            while self.stop:
                pass
            # Stop at destination
            self.drive(0,0)
            
    # Try to get to given coord
    def explore(self, data: Path):
        # Try to follow exploration path
        rospy.logwarn("Exploring a new path!")
        self.newpath = False
        mindex = 0
        mindist = 10000
        for i in range(len(data.poses)):
            ps = data.poses[i]
            x, y = ps.pose.position.x, ps.pose.position.y
            dist = (x - self.x)**2 + (y - self.y)**2
            if dist < mindist:
                mindex = i
                mindist = dist
        
        for i in range(mindex, len(data.poses)):
            if self.newpath: break
            ps = data.poses[i]
            x, y = ps.pose.position.x, ps.pose.position.y
            self.driveto(self.__xy2cell((x,y)))

    # Save the map when clock hits 10 minutes
    def timeout(self, data: Clock):
        sec = data.clock.secs
        if self.start == None:
            self.start = sec
            self.wd = sec
        if self.wdflag:
            self.wd = sec
            self.wdflag = False
        if sec - self.wd > 120: 
            rospy.logerr("A* timed out, restarting it!")
            self.update = True # A* watchdog kills it if lasts more than 2 min
        if sec - self.start == 240: # map towards other corner
            self.goal = (self.rows, self.cols)
            self.update = True
        if sec - self.start == 600:
            with open("../maps/map.npy", mode="wb") as file:
                self.map[self.unexplored] = 1
                np.save(file, self.map, False)
            with open("../maps/map.txt", mode = "w") as file:
                ori = f"{str(self.ori.position.x)},{str(self.ori.position.y)}\n"
                rc = f"{str(self.rows)},{str(self.cols)}\n"
                file.write(ori)
                file.write(rc)
                file.write(str(self.res))
            rospy.signal_shutdown("Time expended.")


if __name__ == '__main__':
    mapper = Explorer()
    try:
        mapper.initialize()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
