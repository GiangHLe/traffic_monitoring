import numpy as np
import cv2
import dlib
import imutils

from vehicle_speed import *


class vehicle:
    def __init__(self, ID, centroid, frame_appear, fps, scale, tuple_cam, all_lanes, **kwags):
        '''
        watch ignore list in tracking
        tuple_cam include vp1,vp2,pp
        '''
        self.ID = ID
        self.centroids = [centroid]
        self.frame = [frame_appear]
        self.fps = fps
        self.scale = scale
        self.speed = [0]

        self.called = 0
        self.disappear = 10
        self.clock = 0
        self.destruction = False
    def update_for_speed(self, new_centroid, new_frame, computeCameraCalibration(tuple_cam)):
        # update all element and calculate speed, instead of all the other fault
        # except cross line
        
        vp1, vp2, vp3, pp, roadPlane, focal = tuple_cam
        laneDivLines = all_lanes
        self.centroids.append(new_centroid)
        self.frame.append(new_frame)
        if np.linalg.norm(centroids[-1]-centroids[-2]) <10:
            # check if vehicle stand
            self.clock += 1
            return None
        if self.clock == 10:
            # Boom
            self.destruction = True
            return None
        if not(getLaneForPoint(centroids[-1], laneDivLines) is None or \
               getLaneForPoint(centroids[-2], laneDivLines) is None):
            frame_diff = frame[-1]-frame[-2]
            self.speed.append(calculateSpeeds(centroids[-1],centroids[-2], self.fps, 
                                self.scale, frame_diff, tuple_cam))
        self.called += 1            
    def update_for_cross_line(centroid, frame_appear)
        













