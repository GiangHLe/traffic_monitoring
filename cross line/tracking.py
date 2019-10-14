from sort import *
from utils import *
from yolo import * 
import cv2

def bbox2necess(bbox,frame,shape):
    """
    Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
    """
    final_res=[]
    width = shape[0]
    height = shape[1]
    for box in bbox:
        res=[]
        # w = box[2]-box[0]
        # h = box[3]-box[1]
        x = ((box[0]+w/2.)/416)*width
        y = ((box[1]+h/2.)/410)*height
        res=np.array([x,y,(box[4]),frame])
        final_res.append(res)
    return np.array(final_res)


def detect_video(yolo, video_type= 'local', video_path, output_path="", thresh_frame = 10 ):
    '''
    - Input:
        + yolo: yolo model
        + video_type: 'local' when use video in PC and 'stream' for....stream
        + video_path: path of video
        + output_path: write down if u wanna something more clearly
    - Output:
        + A tensor [x,y,ID,frame_num]
    '''

    vid = cv2.VideoCapture(video_path)
    if not vid.isOpened():
        raise IOError("Couldn't open webcam or video")
    if video_type == 'stream':
        fps = vid.get(cv2.cv.CV_CAP_PROP_FPS)
    elif video_type == 'local':
        fps = vid.get(7)
    video_FourCC = cv2.VideoWriter_fourcc(*'XVID')
    video_size      = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    isOutput = True if output_path != "" else False
    if isOutput:
        print("!!! TYPE:", type(output_path), type(video_FourCC), type(fps), type(video_size))
        out = cv2.VideoWriter(output_path, video_FourCC, fps, video_size)

    tracker=Sort()
    frame_num=0
    result_track_all_frames=[]
    ignore_set = set()
    while True:
        return_value, frame = vid.read()
        if not return_value:
            break
        image = Image.fromarray(frame)
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
            
            '''
            detect for bbox right here
            '''
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
            result_track_all_frames.append(bbox2necess(res_track,frame_num,video_size))
            frame_num+=1
        
    return np.array(result_track_all_frames)



KalmanBoxTracker.count = 0