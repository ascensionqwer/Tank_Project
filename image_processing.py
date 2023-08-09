import cv2
import numpy as np
import torch

from phycv import VEVID_GPU, PST_GPU, PAGE_GPU

# Image Processing Variables (PhyCV)
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
vevid_gpu = VEVID_GPU(device=device)
pst_gpu = PST_GPU(device=device)
page_gpu = PAGE_GPU(direction_bins=10, device=device)

# Object Detection Variables
classNames = []
classFile = "./Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "./Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "./Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def getObjects(img, thres, nms, draw=True, objects=[]):
    # Object Detection
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    #print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
    return img,objectInfo

def image_process(frame, device, input_ctrl, turn_on_obj, movement, motor_speed, max_speed):
    retVal = frame
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_tensor = torch.from_numpy(np.transpose(frame_rgb, (2, 0, 1))).float().to(device) / 255.0
    if input_ctrl == "VEViD LITE":
        vevid_output = vevid_gpu.run_lite(img_array=img_tensor)
        output_np = (vevid_output.cpu().numpy().transpose(1, 2, 0) * 255).astype(np.uint8)
        retVal = cv2.cvtColor(output_np, cv2.COLOR_RGB2BGR)
    if input_ctrl == "PST":
        retVal = pst_gpu.run(img_array=img_tensor).numpy()
    if input_ctrl == "PAGE":
        retVal = page_gpu.run(img_array=img_tensor).numpy()
    if turn_on_obj:
        retVal, objectInfo = getObjects(retVal, 0.35, 0.2)
    # Draw the current mode and object detection status on the frame
    obj_status = "Object Detection: ON" if turn_on_obj else "Object Detection: OFF"
    cv2.putText(retVal, f"Mode: {input_ctrl}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) 
    cv2.putText(retVal, obj_status, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(retVal, f"Movement: {movement}", (800, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) 
    cv2.putText(retVal, f"Speed: [{motor_speed[0]}%, {motor_speed[1]}%]", (800, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(retVal, f"Max Speed: {max_speed}%", (800, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return retVal

def input_control(key, visual_ctrl, turn_on_obj):
    # Input Control for image processing
    if key == ord('0'):
        return "NORMAL", turn_on_obj
    if key == ord('1'):
        return "VEViD LITE", turn_on_obj
    if key == ord('2'):
        if turn_on_obj is True:
            return "PST", False
        return "PST", turn_on_obj
    if key == ord('3'):
        if turn_on_obj is True:
            return "PAGE", False
        return "PAGE", turn_on_obj
    if key == ord('o'):
        if visual_ctrl == "PAGE" or visual_ctrl == "PST":
            return visual_ctrl, False
        if turn_on_obj is True:
            return visual_ctrl, False
        else:
            return visual_ctrl, True
    return visual_ctrl, turn_on_obj
