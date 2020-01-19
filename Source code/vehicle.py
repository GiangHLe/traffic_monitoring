
import numpy as np
import cv2

from vehicle_speed import *
from cross_red_line import *

class Vehicle:
    def __init__(self, ID, centroid, frame_appear,  bbox, 
                     allow_lanes, all_lanes, image, **kwags):
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

        #### need another network to regconize the license plate for both speed and
        #### cross red line (especially this)
        self.ID = ID
        centroid = self.swapCentroid(centroid)
        self.centroids = [centroid, None]
        self.frame = [frame_appear, None]
        self.bbox  = bbox # image
        self.lane = getLaneForPoint(centroid,all_lanes)
        
        # this thing
        self._catch_cross_lane = False
        
        
        self.allow_lanes = allow_lanes
        self.all_lanes = all_lanes
        
        # this part 's belong to **kwags, should not change it
        self._overSpeed_path = kwags.pop('overSpeed_path','./OverSpeed/')
        self._crossLane_path = kwags.pop('crossLane_path','./CrossLane/')
        self._crossRedLine_path = kwags.pop('crossRedLine_path','./crossRedLine/')
        self._problem_path = kwags.pop('problem_path','./carWithProblem/')
        self._crossLane = kwags.pop('crossLane',False)
        self._problem   = kwags.pop('problem',False)
        
        self.done = False
        self._called = 0
        if self._catch_cross_lane and (self.lane not in self.allow_lanes) and not self._crossLane:
            self.catch_fault_vehicle(self._crossLane_path, image)
            self._crossLane = True

    def swapCentroid(self, c):
        return [c[1],c[0]]

    def setParemeter4speedMeasure(self, fps, scale, tuple_cam, allow_speed, 
                                    best_performance_range):
        # set parameter if mode is speed
        self.speed = 0
        self.fps = fps
        self.scale = scale
        self.allow_speed = allow_speed
        self.speed_avarage = []
        self.tuple_cam = tuple_cam
        self.best_performance_range = best_performance_range
        self._overSpeed = False
        
        
    def update_for_highway(self, new_bbox, new_centroid, new_frame, image):
        # update all element and calculate speed, instead of all the other fault
        # except cross line
        # this function update in mode 'highway' in main, can measure speed and 
        # detect the vehicle with problem
        # Assume that there will be no traffic jam on highway

        new_centroid = self.swapCentroid(new_centroid)


        vp1, vp2, vp3, pp, roadPlane, focal = self.tuple_cam
        laneDivLines = self.all_lanes
        self.centroids[1] = new_centroid
        self.frame[1] = new_frame
        self.bbox = new_bbox
        self.lane = getLaneForPoint(new_centroid, laneDivLines)
        
        
        self._called += 1
        time_appear = self._called/self.fps
        
        # On highway, which car appears more than 20 second and doesn't move
        # => gets in trouble
        if time_appear >= 20 and np.average(np.array(self.speed)) <= 5 and not self._problem:
            self._problem = True
            self.catch_fault_vehicle(self._problem_path, image)

        # On highway, lane is count from left to right, start at 1, only catch 
        # fault if this road is not allow to cross lane.
        if self._catch_cross_lane and (self.lane not in self.allow_lanes) and not self._crossLane:
            self.catch_fault_vehicle(self._crossLane_path, image)
            self._crossLane = True

        frame_diff = self.frame[1]-self.frame[0]
        if frame_diff < 10:
            return None

        # Make sure car in the area with best camera calibration for best measurement
        # take avarage speed for problem car
        ### lack of counting time to clean up whenever a car disappear for 3s
        if  (not(getLaneForPoint(self.centroids[0], laneDivLines) is None or \
               getLaneForPoint(self.centroids[1], laneDivLines) is None)) \
               and new_centroid[1]>= self.best_performance_range[0] \
               and new_centroid[1]<= self.best_performance_range[1]:
            
            
            self.speed = calculateSpeeds(self.centroids[0],self.centroids[1], self.fps, 
                                self.scale, frame_diff, self.tuple_cam)
            self.speed_avarage.append(self.speed)

            if (self.speed > self.allow_speed) and not self._overSpeed:
                self.catch_fault_vehicle(self._overSpeed_path, image)
                self._overSpeed = True
        self.centroids[0] = new_centroid
        self.frame[0] = new_frame

    def setParemeter4crossRedLine(self, deadline, traffic_status):
        self._crossRedLine = False
        self._right_direction = False
        self.deadline = deadline
        self.traffic_status = traffic_status
        
        # rediculous

        self.catched = False
        self.text = '' 
    def update_for_cross_redline(self, new_centroid, frame_appear, traffic_status,
                                 bbox2D_position, mask, image):
        # this function update for cross redline only
        self.frame[1] = frame_appear
        frame_diff = self.frame[1]-self.frame[0]
        # a little delay for make sure the camera can see motorbike lincense plate
        # if self.catched and (not self._crossRedLine) and (self._called == 30):
        #     self.catch_fault_vehicle(self._crossRedLine_path, image)
        #     self._crossRedLine = True

        # only count when the vehicle get catched
        if self.catched:
            self._called += 1
        elif frame_diff < 5:
            return None
        else:
            self.centroids[1] = new_centroid
            # take movement vector of vehicle
            v = np.array([self.centroids[1][0]-self.centroids[0][0], \
                        self.centroids[1][1]-self.centroids[0][0]])
            # normal vector of deadline
            n = np.array([self.deadline[0],self.deadline[1]])
            # check for intersection vehicle
            cosine_phase = cosineVectorPhase(v,n)
            if cosine_phase < 1 and cosine_phase > 0:
                self._right_direction = True
            # red light and true direction
            if self._right_direction and (self.traffic_status == 'red') and not self.catched: 
                vehicle = getBbox(bbox2D_position)
                # Correlation of bbox and maskm set theshold
                prob = round(getProbability2Shape(vehicle, mask),2)
                if prob >= 30:
                    self.catched = True
                    self.text = 'Catched'
                    self.catch_fault_vehicle(self._crossRedLine_path, image)
                    self._crossRedLine = True
        
            self.centroids[0]=new_centroid

    def catch_fault_vehicle(self, src, image):
        ID, frame,  = self.ID, self.frame[-1]
        cv2.imwrite(src+str(frame)+ "_vehicle_" + str(ID) + ".jpg",image)

