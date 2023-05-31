#!/usr/bin/env python3
import os
os.environ["NUMPY_EXPERIMENTAL_ARRAY_FUNCTION"] = "0"

import rospy, cv2, rospkg
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
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
        output.append(np.mean(circles[((i+360)-1):((i+360)+2)]))
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
        self.stop = False
    
    # Set up nodes
    def initialize(self) -> None:
        # Import map from files
        rospack = rospkg.RosPack()
        filepath = rospack.get_path("final_project")
        with open(filepath + "/maps/map.npy", "rb") as file:
            self.map = np.load(file, None, False)
        with open(filepath + "/maps/map.txt", "r") as file:
            x, y = file.readline().split(",")
            r, c = file.readline().split(",")
            self.res = float(file.readline())
            self.rows, self.cols = int(r), int(c)
            ori = Pose()
            ori.position.x, ori.position.y = float(x), float(y)
            self.ori = ori
        
        rospy.init_node('explore_node', anonymous=True)

        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.localizer)
        rospy.Subscriber("global_plan", Path, self.explore)
        rospy.Subscriber("/scan", LaserScan, self.wallstop)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.planner)
        
        # Publishers
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        self.pathpub = rospy.Publisher("global_plan", Path, queue_size = 1)
    
    # Prevent running into walls
    def wallstop(self, data: LaserScan) -> None:
        fRanges = circLPF(data.ranges)
        dir = np.argmin(fRanges)
        if fRanges[dir] < 0.36:
            if (dir > 35) and (dir < 55):
                self.stop = True
                rospy.logwarn("Veering around obstacle (left)!")
                self.drive(self.slow, -self.slow)
            elif (dir > 305) and (dir < 325):
                self.stop = True
                rospy.logwarn("Veering around obstacle (right)!")
                self.drive(self.slow, self.slow)
            elif (dir < 35) or (dir > 325):
                self.stop = True
                rospy.logwarn("Backing away from obstacle...")
                self.drive(-self.slow)
        else: self.stop = False

    # Update route plan
    def planner(self, data: PoseStamped) -> None:
        x, y = data.pose.position.x, data.pose.position.y
        self.goal = self.__xy2cell((x,y))
        
        # Publish new path
        rospy.loginfo(f"Planning path to {self.goal}")
        path, _ = self.astar((self.r, self.c), self.goal)
        
        if path != None:
            output = []
            for i in range(len(path)-1):
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
            rospy.loginfo("Publishing a new path!")
            self.pathpub.publish(path)
            self.newpath = True
            self.stop = False
        else:
            rospy.logerr("No path found!")
    
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
    
    # Convert a pose to an XY coordinate and a heading
    def __pose2xy(self, pose: Pose):
        x, y = pose.position.x, pose.position.y

        ox = pose.orientation.x
        oy = pose.orientation.y
        oz = pose.orientation.z
        ow = pose.orientation.w
        _,_, yaw = euler_from_quaternion([ox, oy, oz, ow])

        return (x, y), circulizer(yaw)

    # Get a list of map coordinates for unoccupied/unexplored neighbors
    def neighbors(self, cell: tuple) -> list:
        # cell is a map coordinate
        row, col = cell
        nlist = []
        for r in range(max(0,row-1), min(row+2,self.rows)):
            for c in range(max(0,col-1), min(col+2,self.cols)):
                if not (r == row and c == col):
                    if self.map[r,c] < 50:
                        nlist.append((r,c))
        return(nlist)
    
    # Get a list of map coordinates giving a path from start to end
    def astar(self, start: tuple, end: tuple) -> list:
        # start and end are map coordinates (row,col)
        
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
        pose = data.pose.pose
        (self.x, self.y), self.heading = self.__pose2xy(pose)
        self.r, self.c = self.__xy2cell((self.x, self.y))
    
    # Wrapper for motor publisher
    def drive(self, speed: float, dir = 0.0) -> None:
        output = Twist()
        output.linear.x = speed
        output.angular.z = dir
        self.cmd_vel.publish(output)
    
    # Point a given direction
    def point(self, heading: float, unforce = True) -> None:
        offset = heading - self.heading
        while abs(offset) > 0.1:
            offset = heading - self.heading
            while offset < -np.pi:
                offset = offset + (2*np.pi)
            while offset > np.pi:
                offset = offset - (2*np.pi)
            self.drive(0, 2*offset)
            
            if self.newpath or (self.stop and unforce): break

    # Linear navigator
    def driveto(self, coord: tuple) -> None:
        # Initial trajectory
        newx, newy = coord
        dx, dy = newx - self.x, newy - self.y
        speed = dx**2 + dy**2
        
        # Follow trajectory
        while (speed > 0.02) and not self.newpath:
            # Hold when needed
            while self.stop:
                pass
            
            # Point in the right direction
            heading = circulizer(np.arctan2(dy, dx))
            self.point(heading)
            
            # Go there
            if speed > 0:
                self.drive(max(self.slow, speed/5))
            else:
                self.drive(min(-self.slow, speed/5))
            
            # Recalculate trajectory
            dx, dy = newx - self.x, newy - self.y
            speed = dx**2 + dy**2
        
        # Stop at destination
        self.drive(0,0)
            
    # Try to get to given coord
    def explore(self, data: Path):
        # Try to follow exploration path
        rospy.loginfo("Following a new path!")
        self.newpath = False

        # Find starting point
        mindex = 0
        mindist = 10000
        for i in range(len(data.poses)):
            ps = data.poses[i]
            x, y = ps.pose.position.x, ps.pose.position.y
            dist = (x - self.x)**2 + (y - self.y)**2
            if dist < mindist:
                mindex = i
                mindist = dist
        
        # Skip intermediate points in straight lines
        ox, oy, oh = self.x, self.y, self.heading
        stages = []
        for i in range(mindex, len(data.poses)):
            if self.newpath: break
            ps = data.poses[i]
            x, y = ps.pose.position.x, ps.pose.position.y
            dx, dy = x - ox, y - oy
            heading = circulizer(np.arctan2(dy, dx))
            if (heading != oh) or (i - stages[-1] > 2/self.res):
                stages.append(i)
            ox, oy, oh = x, y, heading

        # Drive to each point
        for i in stages:
            if self.newpath: break
            ps = data.poses[i]
            x, y = ps.pose.position.x, ps.pose.position.y
            self.driveto((x,y))
        
        # Sit on the final pose
        (x, y), heading = self.__pose2xy(data.poses[-1].pose)
        self.driveto((x,y))
        self.point(heading)
        rospy.loginfo("Arrived at goal!")
        self.drive(0, self.slow)

if __name__ == '__main__':
    driver = Explorer()
    try:
        driver.initialize()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

