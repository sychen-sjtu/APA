#!/usr/bin/env python

import rospy
import sys
sys.path.append("/home/sychen/Projects/APA/APA/src")
sys.path.append("/home/sychen/Projects/APA/APA/src/parking_slot_detection/scripts/gcn-parking-slot")
from parking_slot_detection.srv import gcn_parking

import cv2
import time
import torch
import pprint
import numpy as np
import math
import random

# model
from psdet.utils.config import get_config
from psdet.utils.common import get_logger
from psdet.models.builder import build_model

class gcn_detector_server():
    def __init__(self):
        print("start init gcn server")
        self.input_image_size = (512, 512, 3)
        self.cfg = get_config()
        print("start building model")
        self.model = build_model(self.cfg.model)
        
        print("start loading model params")
        self.model.load_params_from_file(filename=self.cfg.ckpt, logger=None, to_cpu=False)
        self.model.cuda()
        self.model.eval()

        print("start warming up")
        with torch.no_grad():
            warm_up_data_dict = {} 
            warm_up_image = np.zeros(self.input_image_size, dtype=np.uint8)/255.0
            warm_up_data_dict['image'] = torch.from_numpy(warm_up_image).float().permute(2, 0, 1).unsqueeze(0).cuda()
            self.model(warm_up_data_dict)
            print("model warm up finish")
        
        rospy.init_node('server_node')
        rospy.Service('gcn_service', gcn_parking, self.handle_request)
        rospy.loginfo("Server node is ready to receive requests.")
        

    def handle_request(self, req):
        # print("get request")
        start_time = time.time()
        point0_x = []
        point0_y = []
        point1_x = []
        point1_y = []
        types = []

        image = self.imgmsg_to_cv2(req.image_data)
        if image.shape[0] != self.input_image_size[0] or image.shape[1] != self.input_image_size[1] or image.shape[2] != self.input_image_size[2]:
            print("image size error!!! reciving image size is: ", image.shape)
            return point0_x, point0_y, point1_x, point1_y, types
        image = image/255.0
        data_dict = {}
        data_dict['image'] = torch.from_numpy(image).float().permute(2, 0, 1).unsqueeze(0).cuda()
        with torch.no_grad():
            pred_dicts, ret_dict = self.model(data_dict)
            
            slots_pred = pred_dicts['slots_pred']
            for parking_slot_idx in range(len(slots_pred[0])):
                position = slots_pred[0][parking_slot_idx][1]  # 车位对角线的位置
                p0_x = int(self.input_image_size[0] * position[0] - 0.5)
                p0_y = int(self.input_image_size[1] * position[1] - 0.5)
                p1_x = int(self.input_image_size[0] * position[2] - 0.5)
                p1_y = int(self.input_image_size[1] * position[3] - 0.5)
                point0_x.append(p0_x)
                point0_y.append(p0_y)
                point1_x.append(p1_x)
                point1_y.append(p1_y)
                types.append(1)

            sec_per_example = (time.time() - start_time)
            print('Parking Slot Detection time is : %.4f ms.' % (sec_per_example * 1000))
            
            return point0_x, point0_y, point1_x, point1_y, types
    
    @staticmethod
    def imgmsg_to_cv2(img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        # print("image_opencv: ", type(image_opencv))
        return image_opencv


if __name__ == '__main__':
    server = gcn_detector_server()
    rospy.spin()
