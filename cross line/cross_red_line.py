import cv2
import numpy as np
import itertools

def getMask(mask_path):
    # path to mask image
    mask = cv2.imread(mask_path,0)
    mask_position=set()
    for row, value in enumerate(mask):
        for column, element in enumerate(value):
            if element == 255:
                mask_position.add((column,row))
    return mask_position

def getBbox(bbox):
    x,y,x_plus_w,y_plus_h = bbox 
    vehicle_position = itertools.product(range(x,x_plus_w+1),range(y,y_plus_h+1))
    return set(vehicle_position)

def getProbability2Shape(vehicle, mask):
    match = len(vehicle)-len(vehicle.difference(mask))
    bbox  = len(vehicle)
    return (match*100)/bbox

def distanceFromPoint2Line(p, line):
    a,b,c = line[0],line[1],line[2]
    try:
        # if list or np.array shape 1
        x,y = p[0],p[1]
    except:
        # if np.array shape 2
        x,y = p[0][0], p[0][1]
    finally:
        top = abs(a*x+b*y+c)
        bottom = np.linalg.norm(np.array([a,b]))
        return top/bottom

def cosineVectorPhase(v1,v2):
    dotProduct = np.dot(v1,v2)
    normV1 = np.linalg.norm(v1)
    normV2 = np.linalg.norm(v2)
    return dotProduct/(normV1*normV2)

def checkFromTop(p, p_line1, p_line2):
    x,y = p
    x1 = p_line1[0]
    y1 = p_line1[1]
    x2 = p_line2[0]
    y2 = p_line2[1]
    return (x-x1)*(y2-y1)-(y-y1)*(x2-x1) < 0
