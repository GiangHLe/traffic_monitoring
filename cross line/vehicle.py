'''
need to fix:
 + change cross lane permission into parameter.
 + set time disappear to clean up while running, for higher speed and for rtsp.
 + cross red line, deadline 28/10/2019.
 + build a basic model to recognize color in traffic light area.
finish:
 + debug for speed.
 + debug for fps.
 + debug for saving path of fault vehicle.
 + debug for cross Lane.
 + devide 2 part for this class, private update for each situation.
'''





import numpy as np
import cv2

from vehicle_speed import *
from cross_red_line import *

class Vehicle:
    def __init__(self, ID, centroid, frame_appear,  bbox, 
                     allow_lanes, all_lanes, **kwags):
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
        self.centroids = [centroid, None]
        self.frame = [frame_appear, None]
        self.bbox  = bbox # image
        self.lane = getLaneForPoint(centroid,all_lanes)
        
        # this thing
        self._catch_cross_lane = False
        
        
        self.allow_lanes = allow_lanes
        self.all_lanes = all_lanes
        
        # this part 's belong to **kwags, should not change it
        self._called = 0
        # Use for save the speed of fault car
        self.mode = 'speed'
        self._overSpeed_path = kwags.pop('overSpeed_path','./OverSpeed/')
        self._crossLane_path = kwags.pop('crossLane_path','./CrossLane/')
        self._crossRedLine_path = kwags.pop('crossRedLine_path','./crossRedLine/')
        self._problem_path = kwags.pop('problem_path','./carWithProblem/')
        self._crossLane = kwags.pop('crossLane',False)
        self._problem   = kwags.pop('problem',False)

        if self._catch_cross_lane and (self.lane not in self.allow_lanes) and not self._crossLane:
            self.catch_fault_vehicle(self._crossLane_path)
            self._crossLane = True

    def setParemeter4speedMeasure(self, fps, scale, tuple_cam, allow_speed, 
                                    best_performance_line):
        # set parameter if mode is speed
        self.speed = 0
        self.fps = fps
        self.scale = scale
        self.allow_speed = allow_speed
        self.speed_avarage = []
        self.tuple_cam = tuple_cam
        self.best_performance_line = best_performance_line
        self._overSpeed = False

        
    def update_for_highway(self, new_bbox, new_centroid, new_frame):
        # update all element and calculate speed, instead of all the other fault
        # except cross line
        # this function update in mode 'highway' in main, can measure speed and 
        # detect the vehicle with problem
        # Assume that there will be no traffic jam on highway


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
            self.catch_fault_vehicle(self._problem_path)

        # On highway, lane is count from left to right, start at 1, only catch 
        # fault if this road is not allow to cross lane.
        if self._catch_cross_lane and (self.lane not in self.allow_lanes) and not self._crossLane:
            self.catch_fault_vehicle(self._crossLane_path)
            self._crossLane = True

        # Make sure car in the area with best camera calibration for best measurement
        # take avarage speed for problem car
        ### lack of counting time to clean up whenever a car disappear for 3s
        if  (not(getLaneForPoint(self.centroids[0], laneDivLines) is None or \
               getLaneForPoint(self.centroids[1], laneDivLines) is None)) \
               and new_centroid[0]>= self.best_performance_line:
            frame_diff = self.frame[1]-self.frame[0]
            
            self.speed = calculateSpeeds(self.centroids[0],self.centroids[1], self.fps, 
                                self.scale, frame_diff, self.tuple_cam)
            self.speed_avarage.append(self.speed)

            if (self.speed > self.allow_speed) and not self._overSpeed :
                self.catch_fault_vehicle(self._overSpeed_path)
                self._overSpeed = True
        self.centroids[0] = new_centroid
        self.frame[0] = new_frame

    def setParemeter4crossRedLine(self, deadline, traffic_status):
        self._crossRedLine = False
        self._right_direction = False
        self.deadline = deadline
        self.traffic_status = traffic_status
        
        # rediculous

        self.catched = ""
        

    def update_for_cross_redline(self, new_centroid, frame_appear, traffic_status,
                                 bbox2D_position, mask):
        self.centroids[1] = new_centroid
        
#         a = distanceFromPoint2Line(new_centroid, self.deadline)
#         print("ID: {}, dist: {}".format(self.ID, a))
        
#         if distanceFromPoint2Line(new_centroid, self.deadline) <= 300:
        v = np.array([self.centroids[1][0]-self.centroids[0][0], \
                    self.centroids[1][1]-self.centroids[0][0]])
        n = np.array([self.deadline[0],self.deadline[1]])
        cosine_phase = cosineVectorPhase(v,n)
#             print("ID: {}, cosine: {}".format(self.ID,cosine_phase))    
        if cosine_phase < 1 and cosine_phase > 0:
            self._right_direction = True
        
        #[507,498],[1199,472] from GIMP
#         if self.ID ==1:
#             print(new_centroid)
#             print(checkFromTop(new_centroid, [507,498],[1199,472]))
        if self._right_direction and (self.traffic_status == 'red') 
        # and (not checkFromTop(new_centroid, [507,498],[1199,472])):
            vehicle = getBbox(bbox2D_position)
#                 print(vehicle)
            prob = round(getProbability2Shape(vehicle, mask),2)
            # if self.ID == 1:
            #     print("ID: {}, prop: {}, position: {}".format(self.ID, prob, new_centroid[0]))
#                 print(type(vehicle),type(mask))
#                 print(len(vehicle), len(mask))

            if prob >= 30:
                self.catch_fault_vehicle(self._crossRedLine_path)

                self.catched = "Get Fault"

        self.centroids[0]=new_centroid

    def catch_fault_vehicle(self, src):
        # mode: 'speed', 'other'
        ID, frame, bbox , mode = self.ID, self.frame[-1], self.bbox, self.mode
#         if self.mode == 'speed':
#             cv2.imwrite(src + str(frame) + '_' + str(round(self.speed,2))+ "_vehicle_" + str(ID) + ".jpg", bbox)
#         else:
        cv2.imwrite(src+str(frame)+ "_vehicle_" + str(ID) + ".jpg", bbox)

