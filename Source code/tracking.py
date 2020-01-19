from sort import *
from utils import *
from vehicle import *
# from cross_red_line import *


from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from PIL import Image, ImageFont, ImageDraw

from stream import *
import os
import utilities
import numpy as np
import cv2


import time

def bbox2necess(image, bbox,frame,shape):
    """
    return a list, each element is a list contain [posX,posY,ID,frame_no, image bbox]
    """
    final_res=[]
    width = shape[0]
    height = shape[1]
    for box in bbox:
        x = int(round(box[0]))
        y = int(round(box[1]))
        w = int(round(box[2]-box[0]))
        h = int(round(box[3]))-int(round(box[1]))
        x_plus_w = x + w
        y_plus_h = y + h
        bbox2d = image[x:x_plus_w,y:y_plus_h]
        x_centroid = x + w/2
        y_centroid = y + h/2
        bbox2d_position = (y ,x, y_plus_h, x_plus_w)
        res=[x_centroid,y_centroid,(box[4]), frame, bbox2d, bbox2d_position]
        final_res.append(res)
    return final_res


def detect_video(yolo, video_type, video_path, output_path, mask_path, mode, 
                    scale, vp1, vp2, pp, allow_speed, best_performance_range,
                    deadline4Red, allow_lanes, all_lanes, mask_point, show):
    '''
    - Purpose:
        + Take the video as input, capture the violation vehicle into special folder and the video output if necessary
    - Input:
        + yolo: yolo model
        + video_type: 'local' when use video in PC and 'stream' for....stream
        + video_path: path of video
        + output_path: write down if u wanna something more clearly
        + mask_path: directory to mask for crossRedLine mode
        + mode: 'speed' or 'crossRedLine'
        + scale: finding by divide the projective of system to real distance
        + vp1,vp2,pp: result from camera camlibration
        + allow_speed: the limit speed to decide which car is overspeed
        + best_performence_range: the top and bottom range of best camera calibration parameters
                                  region. The side limits are decide by line of street.
        + deadline4Red: in 'crossRedLine' mode, while red light turn on, if vehicle cross this line -> violation
        + allow_lanes: to detect cross lane violation
        + all_lanes: all lines of one angle of area, in pkl file.
        + mask_point: mask where camera calibration work best
        + show: show the result or not.
    - Output:
        + None
    '''

    tuple_cam = computeCameraCalibration(vp1,vp2,pp)
    
    # vid = cv2.VideoCapture(video_path)
    
    if video_type == 'stream':
        link_stream = input('Please enter video link: ')
        vid = VideoStreamer(link_stream)
        # if not vid.more():
        #     raise IOError("Couldn't open stream")

    elif video_type == 'local':
        vid = cv2.VideoCapture(video_path)
        fps = vid.get(5)
        if not vid.isOpened():
            raise IOError("Couldn't open video")

    isOutput = True if output_path != "" else False
    if isOutput:
        video_FourCC = cv2.VideoWriter_fourcc('M','J','P','G')
        if video_type == 'local':
            video_size   = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                            int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        elif video_type == 'stream':
            video_size = (1280,720)
            fps = 30
        out = cv2.VideoWriter(output_path, video_FourCC, fps, video_size, 1)

    tracker=Sort()
    frame_num=0
    all_vehicle={}
    ignore_set = set()
    if mode == 'crossRedLine':
        mask = getMask(mask_path)
        capture_light = True
    time.sleep(5)
    t1 = time.time()
    cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('frame', 1200,800)

    while True:      
        
        if video_type == 'local':
            return_value, pic = vid.read()
            if not return_value:
                break
        elif video_type == 'stream':
            if vid.more():
                pic = vid.read()
                fps = frame_num/(time.time()-5-t1)
                if fps < 10:
                    frame_num+=1
                    continue
            elif not vid.more():
                continue
        image = Image.fromarray(pic)
        if yolo.model_image_size != (None, None):
            assert yolo.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert yolo.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(yolo.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                            image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.
        
        # detect for bbox right here
        out_boxes, out_scores, out_classes = yolo.sess.run( 
            [yolo.boxes, yolo.scores, yolo.classes],  
            feed_dict={
                yolo.yolo_model.input: image_data,
                yolo.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })
        final_box=[]
        label=[]
        scores=[]           

        # clear this
        if show:
#             and mode == 'crossRedLine':
            cv2.line(pic, tuple(mask_point[0]) , tuple(mask_point[1]) , (0,255,255), 3) 
            cv2.line(pic, tuple(mask_point[1]) , tuple(mask_point[2]) , (0,255,255), 3) 
            cv2.line(pic, tuple(mask_point[2]) , tuple(mask_point[3]) , (0,255,255), 3) 
            cv2.line(pic, tuple(mask_point[3]) , tuple(mask_point[0]) , (0,255,255), 3)                                     
        
        for b,lb,sc in zip(out_boxes,out_classes,out_scores):
            if lb in [2,3,5,7]:
                final_box.append(b)
                label.append(lb)
                scores.append(sc)
            if lb == 9 and mode == 'crossRedLine' and capture_light:                    
                t_x, t_y, b_x, b_y = np.int_(np.round(np.array(b)))
                capture_light = False
                

        
        out_boxes=np.array(final_box)

        out_scores=np.array(scores)

        final_boxes=np.column_stack((out_boxes,out_scores,label))
        if final_boxes.shape[0]==0:
            if isOutput:
                out.write(pic)
            if show:
                cv2.imshow('frame', pic)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break 
            frame_num+=1
            continue
        final_boxes = final_boxes[np.logical_and(final_boxes[:, 4] > 0.3, final_boxes[:, 2] -
                                                    final_boxes[:, 0] < 600)]
        # Apply NMS
        indices = utilities.non_max_suppression(final_boxes, 0.9, final_boxes[:, 4])
        
        
        
        out_boxes = [final_boxes[i][:5] for i in indices]
        
        vehicle_label = [final_boxes[i][5] for i in indices]
        
        
        res_track=tracker.update(np.array(out_boxes))


        one_frame = bbox2necess(image = pic, bbox = res_track,frame =frame_num,
                                    shape = video_size)
        
        if mode == 'crossRedLine' and video_type =='stream':
            traffic_bbox = pic[143:295,984:1014,:]
            traffic_status = detect_red(traffic_bbox)
            if traffic_status:
                text_traffic = 'red'

            else:
                text_traffic = 'green'
            
        if mode == 'crossRedLine'and video_type =='local':
            try:
                traffic_bbox = pic[t_x:b_x, t_y:b_y, :]
                traffic_status = detect_red(traffic_bbox)
                if traffic_status:
                    text_traffic = 'red'
                    
                else:
                    text_traffic = 'green'
            except:
                print('can not find traffic light')
                break
        
        
        for idx, vehicle in enumerate(one_frame):
            ID =  int(round(vehicle[2]))
            label = vehicle_label[idx]
            centroid = [vehicle[0],vehicle[1]]
            frame_appear = frame_num
            bbox = vehicle[4]
            bbox2d_position = vehicle[5]
            if label not in [2,3,5,7]:
                continue
            x,y,x_plus_w,y_plus_h = bbox2d_position

            # set font and color
            font                   = cv2.FONT_HERSHEY_SIMPLEX
            fontScale              = 1
            fontColor              = (0,255,0)
            thickness              = 3
            linetype               = cv2.LINE_AA

            # round the centroid to show result
            c_0 = int(round(centroid[0]))
            c_1 = int(round(centroid[1]))

            # car
            if label == 2:
                image_vehicle = np.array(image.crop(((c_1-150),(c_0-150), \
                                    (c_1+150),(c_0+150))))
                label_text = "car"
            # motorbike
            elif label == 3 :
                image_vehicle = np.array(image.crop(((c_1-150),(c_0-150), \
                                    (c_1+150),(c_0+150))))
                label_text = "motorbike"
            # bus
            elif label == 5:
                image_vehicle = np.array(image.crop(((c_1-300),(c_0-300), \
                                    (c_1+300),(c_0+300))))
                label_text = "bus"
            # truck
            elif label == 7:
                image_vehicle = np.array(image.crop(((c_1-200),(c_0-200), \
                                    (c_1+200),(c_0+200))))
                label_text = "truck"

            if show:
                cv2.putText(pic,str(ID), 
                                (c_1 - 10 , c_0 -10), 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                linetype)
                cv2.putText(pic,label_text, 
                                (c_1 - 10 , c_0 +10), 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                linetype)
                cv2.rectangle(pic, (x,y), (x_plus_w ,y_plus_h), (0,255,0), 4)

            if ID in ignore_set:
                continue
                
            if ID not in all_vehicle.keys():
                all_vehicle[ID] = Vehicle(ID, centroid, frame_appear, bbox, allow_lanes, 
                                            all_lanes, image_vehicle, mode = mode)
                if mode == 'speed':
                    all_vehicle[ID].setParemeter4speedMeasure(fps, scale, tuple_cam,
                                                        allow_speed, best_performance_range)
                elif mode == 'crossRedLine':                          
                    if not checkFromTop(centroid, [704,680] , [1222,1070]):
                        if tuple(centroid) not in mask:
                            ignore_set.add(ID)
                    all_vehicle[ID].setParemeter4crossRedLine(deadline = deadline4Red, 
                                        traffic_status = text_traffic)
                continue

            if mode =='speed':
                if show:

                    t2 = time.time()
                    fps_temp = round(frame_num/(t2-t1),2)
                    cv2.putText(pic, "fps: "+str(fps_temp), (30, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)

                    cv2.rectangle(pic, (x,y), (x_plus_w ,y_plus_h), (0,255,0), 4)
                    cv2.putText(pic,str(round(all_vehicle[ID].speed,2)), 
                                (c_1 -10 , c_0 +30), 
                                font, 
                                fontScale,
                                (255,255,0),
                                thickness,
                                linetype)
#                     cv2.line(pic,(0,best_performance_range[0]),(video_size[0],best_performance_range[0]),(0,255,0),thickness)
#                     cv2.line(pic,(0,best_performance_range[1]),(video_size[0],best_performance_range[1]),(0,255,0),thickness)
                all_vehicle[ID].update_for_highway(bbox, centroid, frame_appear, image_vehicle)


            elif mode == 'crossRedLine':
                if show:
                    # cv2.rectangle(pic, (x,y), (x_plus_w ,y_plus_h), (255,255,0), 4)
                    cv2.putText(pic, all_vehicle[ID].text, 
                                (c_1 - 10 , c_0 +30), 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                linetype)
                

                    t2 = time.time()
                    fps_temp = round(frame_num/(t2-t1),2)
                    cv2.putText(pic, "fps: "+str(fps_temp), (30, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)
                    cv2.putText(pic, "Status: "+str(text_traffic), (30, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)
                all_vehicle[ID].update_for_cross_redline(centroid, frame_appear, 
                                text_traffic, bbox2d_position, mask,image_vehicle)
                         
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break  
        if isOutput:
            out.write(pic)
        
        if show:
            cv2.imshow('frame', pic)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break 
        frame_num+=1
    if video_type == 'local': 
        vid.release()
    if isOutput:
        out.release()
    cv2.destroyAllWindows()
KalmanBoxTracker.count = 0