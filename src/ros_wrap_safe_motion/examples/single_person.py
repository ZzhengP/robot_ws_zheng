#!/usr/bin/env python
from __future__ import print_function
import argparse
import sys
sys.path.insert(0, '/home/zheng/lightweight-human-pose-estimation.pytorch')
import roslib
import rospy
import cv2
from std_msgs.msg import String, Float32MultiArray    
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
## ===========   stacked_hourglass ===========================
import torch
import torchvision
import numpy as np
from numpy import asarray
from models.with_mobilenet import PoseEstimationWithMobileNet
from modules.keypoints import extract_keypoints, group_keypoints
from modules.load_state import load_state
from modules.pose import Pose, track_poses
from val import normalize, pad_width
import time


def toTensor(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    img = torch.from_numpy(img.transpose((2,0,1)))
    return img.float().div(255).unsqueeze(0)



def infer_fast(net, img, net_input_height_size, stride, upsample_ratio, cpu,
               pad_value=(0, 0, 0), img_mean=(128, 128, 128), img_scale=1/256):
    height, width, _ = img.shape
    scale = net_input_height_size / height

    scaled_img = cv2.resize(img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    scaled_img = normalize(scaled_img, img_mean, img_scale)
    min_dims = [net_input_height_size, max(scaled_img.shape[1], net_input_height_size)]
    padded_img, pad = pad_width(scaled_img, stride, pad_value, min_dims)

    tensor_img = torch.from_numpy(padded_img).permute(2, 0, 1).unsqueeze(0).float()
   
    tensor_img = tensor_img.cuda()

    stages_output = net(tensor_img)

    stage2_heatmaps = stages_output[-2]
    heatmaps = np.transpose(stage2_heatmaps.squeeze().cpu().data.numpy(), (1, 2, 0))
    heatmaps = cv2.resize(heatmaps, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)

    stage2_pafs = stages_output[-1]
    pafs = np.transpose(stage2_pafs.squeeze().cpu().data.numpy(), (1, 2, 0))
    pafs = cv2.resize(pafs, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)

    return heatmaps, pafs, scale, pad


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.keypts_pub = rospy.Publisher('/keypoints',Float32MultiArray,queue_size=10)
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback) # For realsense
    # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback) # For Asus 
    self.image_sub = rospy.Subscriber("/pcl/output",Image,self.callback) # for point cloud 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
      ## Rescale Image size 
    rescale_factor = 1
    width = int(cv_image.shape[1] * rescale_factor)
    height = int(cv_image.shape[0] * rescale_factor)
    dim = (width,height)
    resized_img = cv2.resize(cv_image, dim)

  
    net = PoseEstimationWithMobileNet()
    checkpoint = torch.load("/home/zheng/lightweight-human-pose-estimation.pytorch/checkpoint_iter_370000.pth", map_location='cpu')
    load_state(net, checkpoint)
    height_size = 256
    net = net.eval()
    net = net.cuda()
    net.eval()

    stride = 8
    upsample_ratio = 4
    num_keypoints = Pose.num_kpts
    previous_poses = []
    delay = 33
    # img = cv2.imread("/home/zheng/lightweight-human-pose-estimation.pytorch/data/image_1400.jpg")
    img = asarray(cv_image)
    orig_img = img
    heatmaps, pafs, scale, pad = infer_fast(net, img, height_size, stride, upsample_ratio, cpu = "store_true")

    total_keypoints_num = 0
    all_keypoints_by_type = []
    for kpt_idx in range(num_keypoints):  # 19th for bg
          total_keypoints_num += extract_keypoints(heatmaps[:, :, kpt_idx], all_keypoints_by_type, total_keypoints_num)

    pose_entries, all_keypoints = group_keypoints(all_keypoints_by_type, pafs, demo=True)

    for kpt_id in range(all_keypoints.shape[0]):
          all_keypoints[kpt_id, 0] = (all_keypoints[kpt_id, 0] * stride / upsample_ratio - pad[1]) / scale
          all_keypoints[kpt_id, 1] = (all_keypoints[kpt_id, 1] * stride / upsample_ratio - pad[0]) / scale
    current_poses = []

    ##   Collect all keypoint in numpy array to send it to Ros"  
    pose_keypoints_ros_data  = np.zeros(16)
    my_array_for_publishing = Float32MultiArray()



    #### 
    pose_keypoints = np.ones((num_keypoints, 2), dtype=np.int32) * -1
    for kpt_id in range(8):
        if pose_entries[0][kpt_id] != -1.0:  # keypoint was found
            pose_keypoints[kpt_id, 0] = int(all_keypoints[int(pose_entries[0][kpt_id]), 0])
            pose_keypoints[kpt_id, 1] = int(all_keypoints[int(pose_entries[0][kpt_id]), 1])

        pose = Pose(pose_keypoints, pose_entries[0][18])
        current_poses.append(pose)
        pose_keypoints_ros_data[2*kpt_id] = pose.keypoints[kpt_id][0]
        pose_keypoints_ros_data[2*kpt_id+1] = pose.keypoints[kpt_id][1]
    for pose in current_poses:
          pose.draw(img)
    img = cv2.addWeighted(orig_img, 0.6, img, 0.4, 0)
    my_array_for_publishing.data = [pose_keypoints_ros_data[0],pose_keypoints_ros_data[1],
                                    pose_keypoints_ros_data[2],pose_keypoints_ros_data[3],
                                    pose_keypoints_ros_data[4],pose_keypoints_ros_data[5],
                                    pose_keypoints_ros_data[6],pose_keypoints_ros_data[7],
                                    pose_keypoints_ros_data[8],pose_keypoints_ros_data[9],
                                    pose_keypoints_ros_data[10],pose_keypoints_ros_data[11],
                                    pose_keypoints_ros_data[12],pose_keypoints_ros_data[13],
                                    pose_keypoints_ros_data[14],pose_keypoints_ros_data[15], ]
    # cv2.imshow('Lightweight Human Pose Estimation Python Demo', img)
    self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    self.keypts_pub.publish(my_array_for_publishing)
    # cv2.imwrite('/home/zheng/Bureau/image_1400_key.jpg',img)

    cv2.waitKey(2) 

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':

    main(sys.argv)
