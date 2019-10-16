import numpy as np
import cv2

from vehicle_speed import *
from cross_red_line import *

class Vehicle:
    def __init__(self, ID, centroid, frame_appear, fps, scale, tuple_cam, bbox, 
                    allow_speed, allow_lanes, all_lanes, **kwags):
        '''
        watch ignore list in tracking
        tuple_cam include vp1,vp2, vp3, pp, roadPlane, focal (put this into main)
        calculate speed, measure lane, cross line if fault, save bbox into directionary
        delete vehicle in N time disappear
        check if thresh car problem in 1 time -> traffic jam
        # fix to have bbox in main #
        '''
        '''
        for cross_line:
            accept_line
            right_ditection = False
            area of interest
            
        '''
        self.ID = ID
        self.centroids = [centroid, None]
        self.frame = [frame_appear, None]
        self.bbox  = bbox # image
        self.lane = getLaneForPoint[centroid]
        self.catch_cross_lane = False
        self.fps = fps
        self.scale = scale
        self.speed = 0
        self.allow_lanes = allow_lanes
        self.allow_speed = allow_speed
        # self.called = 0
        self.disappear = 10
        self.clock = 0
        self.stayed = False
        self.problem = False
        self.tuple_cam = tuple_cam
        self.mode = 'speed'
        self.overSpeed_path = './OverSpeed/'
        self.crossLane_path = './CrossLane/'
        self.crossRedLine_path = './crossRedLine/'
        self.problem_path = './carWithProblem/'

        if self.catch_cross_lane and (lane not in self.allow_lanes):
            self.catch_fault_vehicle(self.crossLane_path)

    def update_for_highway(self, new_bbox, new_centroid, new_frame):
        # update all element and calculate speed, instead of all the other fault
        # except cross line
        # this function update in mode 'highway' in main, can measure speed and 
        # detect the vehicle with problem

        vp1, vp2, vp3, pp, roadPlane, focal = self.tuple_cam
        laneDivLines = all_lanes
        self.centroids[1] = new_centroid
        self.frame[1] = new_frame
        self.bbox = new_bbox
        if np.linalg.norm(centroids[-1]-centroids[-2]) <10:
            # check if vehicle stand
            self.clock += 1
            return None
        if self.clock == 10:
            # Boom
            self.stayed = True
            return None

        if self.catch_cross_lane and (lane not in self.allow_lanes):
            self.catch_fault_vehicle(self.crossLane_path)

        if not(getLaneForPoint(centroids[-1], laneDivLines) is None or \
               getLaneForPoint(centroids[-2], laneDivLines) is None):
            frame_diff = frame[-1]-frame[-2]
            self.speed = calculateSpeeds(centroids[-1],centroids[-2], self.fps, 
                                self.scale, frame_diff, tuple_cam)
            if self.speed > self.allow_speed:
                self.catch_fault_vehicle(self.overSpeed_path)
        # self.called += 1
        self.centroids[0] = new_centroid
        self.frame[0] = new_frame
            
    def update_for_cross_way(self, centroid, frame_appear):
        
        

    def check_lane(self, lane):
        if catch_cross_lane and (lane not in self.allow_lanes):
            self.catch_cross_lane() 

    def catch_fault_vehicle(self, src):
        # mode: 'speed', 'other'
        ID, frame, bbox , mode = self.ID, self.frame[-1], self.bbox, self.mode
        if mode == 'speed':
            cv2.imwrite(mode + '_' + str(frame)+ "_vehicle_" + str(ID) + ".jpg", bbox)
        else:
            cv2.imwrite(str(frame)+ "_vehicle_" + str(ID) + ".jpg", bbox)











