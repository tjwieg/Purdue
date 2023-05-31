import rospy
from queue import PriorityQueue
from itertools import count

class MapCell:
    def __init__(self, coord: tuple, source: tuple, weight = 1) -> None:
        self.coord = coord      # map coordinates    
        self.source = source    # map coordinates
        self.weight = weight    # a numeric giving cost from source to coord
    def __str__(self) -> str:
        return(f"({self.coord[0]},{self.coord[1]})")    

class MapQueue(PriorityQueue):
    def __init__(self, start: tuple, end: tuple, maxsize: int = 0) -> None:
        super().__init__(maxsize)
        self.goal = end
        self.burned = [start]
        self.history = [""]
        self.unique = count()
    
    # Add a new cell to the queue
    def add(self, newcell: MapCell) -> None:
        dist = ((self.goal[0] - newcell.coord[0])** 2 +
                (self.goal[1] - newcell.coord[1])**2)
        if dist == 0: priority = 0
        else: priority = newcell.weight + dist
        tiebreak = next(self.unique)
        self.put((priority, tiebreak, (newcell, dist)))
    
    # Pull the next item out of the queue
    def next(self) -> MapCell:
        if self.empty():
            rospy.logerr("Queue is empty!")
            return None
        else:
            oldcell = self.get()[2][0]
            self.history.append(oldcell)
            self.burned.append(oldcell.coord)
            return oldcell