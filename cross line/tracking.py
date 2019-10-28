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
        x_plus_w = x+w
        y_plus_h = y+h
        bbox2d = image[x:x_plus_w,y:y_plus_h]
        x_centroid = x + w/2
        y_centroid = y + h/2
        bbox2d_position = [x ,y,x_plus_w,y_plus_h]
        res=[x_centroid,y_centroid,(box[4]), frame, bbox2d, bbox2d_position]
        final_res.append(res)
    return final_res


def detect_video(yolo, video_type, video_path, output_path, mask_path, mode, 
                    scale, vp1, vp2, pp, allow_speed, best_performance_line,
                    deadline4Red, allow_lanes, all_lanes, thresh_frame):
    '''
    - Input:
        + yolo: yolo model
        + video_type: 'local' when use video in PC and 'stream' for....stream
        + video_path: path of video
        + output_path: write down if u wanna something more clearly
    - Output:
        + A tensor [x,y,ID,frame_num]
    '''
    # these thing should append into data file
    tuple_cam = computeCameraCalibration(vp1,vp2,pp)
    
    # show for debug
    
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 1000,600)

    
    
    
    
    # require opencv 3.2
    
    vid = cv2.VideoCapture(video_path)
    if not vid.isOpened():
        raise IOError("Couldn't open webcam or video")
    if video_type == 'stream':
        fps = vid.get(cv2.CAP_PROP_FPS)
    elif video_type == 'local':
        fps = vid.get(5)
    # get(7) is all number of frame in video
    # get(5) is get fps, fuck
    
    video_FourCC = cv2.VideoWriter_fourcc(*'XVID')
    video_size      = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    isOutput = True if output_path != "" else False
    if isOutput:
        print("!!! TYPE:", type(output_path), type(video_FourCC), type(fps), type(video_size))
        out = cv2.VideoWriter(output_path, video_FourCC, fps, video_size)

    tracker=Sort()
    frame_num=0
    all_vehicle={}
    ignore_set = set()
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
            for b,lb,sc in zip(out_boxes,out_classes,out_scores):
                if lb in [2,3,5,7]:
                    final_box.append(b)
                    label.append(lb)
                    scores.append(sc)

            out_boxes=np.array(final_box)
            out_classes=np.array(label)
            out_scores=np.array(scores)
            final_boxes=np.column_stack((out_boxes,out_scores))
            final_boxes = final_boxes[np.logical_and(final_boxes[:, 4] > 0.3, final_boxes[:, 2] -
                                                        final_boxes[:, 0] < 600)]

                # Apply NMS
            indices = utilities.non_max_suppression(final_boxes, 0.9, final_boxes[:, 4])
            
            out_boxes = [final_boxes[i] for i in indices]
            out_classes= [out_classes[i] for i in indices]
            rev=(reversed(out_classes))  # for display in order since yolo reverse the list 
            out_classes=[]
            for r in rev:
                out_classes.append(r)
            out_classes=np.array(out_classes)
            bf=out_boxes
            res_track=tracker.update(np.array(out_boxes))
            # res_track return [x,y, x+w, x+y, ID]
            one_frame = bbox2necess(image = pic, bbox = res_track,frame =frame_num,
                                        shape = video_size)
            for vehicle in one_frame:
                ID =  int(round(vehicle[2]))
                centroid = [vehicle[0],vehicle[1]]
                frame_appear = frame_num
                bbox = vehicle[4]
                bbox2d_position = vehicle[5]

                # for show video, will delete later
                font                   = cv2.FONT_HERSHEY_SIMPLEX
                fontScale              = 1
                fontColor              = (0,0,255)
                thickness              = 3
                linetype               = cv2.LINE_AA
                c_0 = int(round(centroid[0]))
                c_1 = int(round(centroid[1]))
                #end 
                traffic_status = 'red'

                if ID not in all_vehicle.keys():
                    all_vehicle[ID] = Vehicle(ID, centroid, frame_appear, bbox, allow_lanes, 
                                                all_lanes, mode = mode)
                    if mode == 'speed':
                        all_vehicle[ID].setParemeter4speedMeasure(fps, scale, tuple_cam,
                                                            allow_speed, best_performance_line)
                    elif mode == 'crossRedLine':
                        mask = getMask(mask_path)
                        all_vehicle[ID].setParemeter4crossRedLine(deadline = deadline4Red, 
                                            traffic_status = traffic_status,
                                            areaOfInterest = mask)
                    continue

                if mode =='speed':
                    all_vehicle[ID].update_for_highway(bbox, centroid, frame_appear)
                    #show for debug
                    
                    cv2.putText(pic,str(ID), 
                                (c_1 - 10 , c_0 -10), 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                linetype)
                    cv2.putText(pic,str(all_vehicle[ID].speed), 
                                (c_1 -10 , c_0 +20), 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                linetype)
                    cv2.imshow('image',pic)

                elif mode == 'crossRedLine':
                    all_vehicle[ID].update_for_cross_redline(centroid, frame_appear, 
                                    traffic_status, bbox2d_position)
                    cv2.putText(pic,str(ID), 
                                (c_1 - 10 , c_0 -10), 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                linetype)
                    cv2.putText(pic, all_vehicle[ID].catched, 
                                (c_1 - 10 , c_0 -10), 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                linetype)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break            
        frame_num+=1
        
    vid.release()
    out.release()
    cv2.destroyAllWindows()
KalmanBoxTracker.count = 0