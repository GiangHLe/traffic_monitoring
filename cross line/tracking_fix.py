from sort import *
from utils import *
# from vehicle import *
# from cross_red_line import *


from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from PIL import Image, ImageFont, ImageDraw

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
#     print(width,height)
    for box in bbox:
#         print(box)
        x = int(round(box[0]))
        y = int(round(box[1]))
        w = int(round(box[2]-box[0]))
        h = int(round(box[3]))-int(round(box[1]))
#         print(x,y,w,h)
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
                    deadline4Red, allow_lanes, all_lanes, thresh_frame, show):
    '''
    - Input:
        + yolo: yolo model
        + video_type: 'local' when use video in PC and 'stream' for....stream
        + video_path: path of video
        + output_path: write down if u wanna something more clearly
    - Output:
        + A tensor [x,y,ID,frame_num]
    '''
    











    tuple_cam = computeCameraCalibration(vp1,vp2,pp)
    
    vid = cv2.VideoCapture(video_path)
    if not vid.isOpened():
        raise IOError("Couldn't open webcam or video")
    if video_type == 'stream':
        fps = vid.get(cv2.CAP_PROP_FPS)
    elif video_type == 'local':
        fps = vid.get(5)

    

    isOutput = True if output_path != "" else False
    if isOutput:
        video_FourCC = cv2.VideoWriter_fourcc('M','J','P','G')
        video_size      = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                           int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        out = cv2.VideoWriter(output_path, video_FourCC, fps, video_size, 1)

    tracker=Sort()
    frame_num=0
    all_vehicle={}
    ignore_set = set()

    mask = getMask(mask_path)
    capture_light = True
    t1 = time.time()
    while True:      
        
        return_value, pic = vid.read()
        if not return_value:
            break
        image = Image.fromarray(pic)
        if frame_num%thresh_frame==0:
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
            if show and mode == 'crossRedLine':
                cv2.line(pic, (704,680) , (1165,432) , (0,255,255), 3) 
                cv2.line(pic, (1165,432) , (1900,756) , (0,255,255), 3) 
                cv2.line(pic, (1900,756) , (1222,1070) , (0,255,255), 3) 
                cv2.line(pic, (1222,1070) , (704,680) , (0,255,255), 3) 
            
            
            
            
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

            final_boxes=np.column_stack((out_boxes,out_scores))

            final_boxes = final_boxes[np.logical_and(final_boxes[:, 4] > 0.3, final_boxes[:, 2] -
                                                        final_boxes[:, 0] < 600)]
            # Apply NMS
            indices = utilities.non_max_suppression(final_boxes, 0.9, final_boxes[:, 4])

            out_boxes = [final_boxes[i] for i in indices]

            res_track=tracker.update(np.array(out_boxes))

            # add label in this


            one_frame = bbox2necess(image = pic, bbox = res_track,frame =frame_num,
                                        shape = video_size)
            if mode == 'crossRedLine':
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
            for vehicle in one_frame:
                ID =  int(round(vehicle[2]))
                centroid = [vehicle[0],vehicle[1]]
                frame_appear = frame_num
                bbox = vehicle[4]
                bbox2d_position = vehicle[5]

                x,y,x_plus_w,y_plus_h = bbox2d_position
                
                



                # set font and color
                font                   = cv2.FONT_HERSHEY_SIMPLEX
                fontScale              = 1
                fontColor              = (0,0,255)
                thickness              = 3
                linetype               = cv2.LINE_AA

                # round the centroid to show result
                c_0 = int(round(centroid[0]))
                c_1 = int(round(centroid[1]))




                # car
                if label == 2:
                    image_vehicle = pic[(c_0-150):(c_0+150), \
                                        (c_1-150):(c_1+150)]
                # motorbike
                elif label == 3 :
                    image_vehicle = pic[(c_0-100):(c_0+100), \
                                        (c_1-100):(c_1+100)]
                # bus
                elif label == 5:
                    image_vehicle = pic[(c_0-250):(c_0+250), \
                                        (c_1-250):(c_1+250)]
                # truck
                elif label == 2:
                    image_vehicle = pic[(c_0-200):(c_0+200), \
                                        (c_1-200):(c_1+200)]


                if show:
                    cv2.putText(pic,str(ID), 
                                    (c_1 - 10 , c_0 -10), 
                                    font, 
                                    fontScale,
                                    fontColor,
                                    thickness,
                                    linetype)
                    cv2.rectangle(pic, (x,y), (x_plus_w ,y_plus_h), (0,0,255), 4)

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

                        cv2.rectangle(pic, (x,y), (x_plus_w ,y_plus_h), (0,0,255), 4)
                        cv2.putText(pic,round(str(all_vehicle[ID].speed),2), 
                                    (c_1 -10 , c_0 +20), 
                                    font, 
                                    fontScale,
                                    fontColor,
                                    thickness,
                                    linetype)
                    all_vehicle[ID].update_for_highway(bbox, centroid, frame_appear, image_vehicle)


                elif mode == 'crossRedLine':
                    if show:
                        # cv2.rectangle(pic, (x,y), (x_plus_w ,y_plus_h), (255,255,0), 4)
                        cv2.putText(pic, all_vehicle[ID].text, 
                                    (c_1 - 10 , c_0 +20), 
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
        
    vid.release()
    if isOutput:
        out.release()
    cv2.destroyAllWindows()
KalmanBoxTracker.count = 0