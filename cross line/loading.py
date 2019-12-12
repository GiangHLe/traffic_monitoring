import numpy as np
import json

def loadData(json_link):
    with open(json_link, 'r') as data:
        file = json.load(data)
        all_lanes = file["LanesMeasurement"]
        for k,v in enumerate(all_lanes):
            all_lanes[k] = np.array(all_lanes[k])
        best_performance_range = file["BestPerformanceRange"]
        vp1,vp2,pp,scale = file["CameraCamlibration"]
        mask_point = file["Mask"]
    return all_lanes, best_performance_range, mask_point, vp1,vp2,pp,scale

def dumpData(json_link, data):
    with open(json_link, 'w')as outfile:
        dict = {"LanesMeasurement": data[0],
                "BestPerformanceRange": data[1],
                "Mask":data[2],
                "CameraCamlibration": data[3]}
        json.dump(dict, outfile)
    