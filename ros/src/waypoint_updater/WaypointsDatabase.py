import numpy as np
from scipy.spatial import KDTree

class WaypointsDatabase:
    """This class can be used to query the closest waypoint to a given (x,y) point"""
    def __init__(self, waypoints):
        self.waypoints = waypoints
        
    def get_next_closest_idx(self, pose):
        # Find the closest waypoints to pose *that comes after pose on the track*
        # If pose is between x0 and x1, closer to x0, this should still return the index/distance of/to x1
        return 0., 0
