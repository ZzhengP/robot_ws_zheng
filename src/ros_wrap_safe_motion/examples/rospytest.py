#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import String, Float32MultiArray    
import sys


class depthInference:

  def __init__(self):
    self.image_depth_sub_ = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,self.convert_depth_image, queue_size=1)
    self.keypts_sub = rospy.Subscriber('/keypoints',Float32MultiArray,self.key_point, queue_size = 1)
    self.bridge = CvBridge()
    self.key_point = np.zeros((8,2))

  def convert_depth_image(self,data):
      depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
      depth_array = np.array(depth_image, dtype=np.float32)
      center_idx = np.array(depth_array.shape) / 2
      print("depth_array ;", depth_array.shape)
      for i in range(0,8):
        print ('', depth_array[int(self.key_point[i,1]), int(self.key_point[i,0])])
  
  def key_point(self,data):
      my_array = Float32MultiArray()
      my_array = data
      
      for i in range(0,8):
        if (my_array.data[2*i] != -1):
          self.key_point[i][0] = my_array.data[2*i]
          self.key_point[i][1] = my_array.data[2*i+1]

      
def main(args):
  ic = depthInference()
  rospy.init_node('depthInference', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':

    main(sys.argv)


def convert_depth_image(ros_image):
    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        center_idx = np.array(depth_array.shape) / 2
        print ('center depth:', depth_array[center_idx[0], center_idx[1]])

    except CvBridgeError, e:
        print e
     #Convert the depth image to a Numpy array


def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	pixel2depth()
    
# #!/usr/bin/env python3
# from __future__ import print_function
# import rospy
# from std_msgs.msg import String
# from stacked_hourglass import HumanPosePredictor, hg2, hg8
# import torch
# import torchvision
# from torchvision import transforms
# import cv2
# import numpy as np
# from numpy import asarray
# from stacked_hourglass.datasets.mpii import MPII_JOINT_NAMES
# import time
# import roslib
# import sys
# from sensor_msgs.msg import CompressedImage
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# def toTensor(img):
#     img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
#     img = torch.from_numpy(img.transpose((2,0,1)))
#     return img.float().div(255).unsqueeze(0).cuda()
# class image_converter:

#   def __init__(self):
#     self.image_pub = rospy.Publisher("image_topic_2",Image)

#     self.bridge = CvBridge()
#     # self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
#     self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

#   def callback(self,data):
#     try:
#       cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#     except CvBridgeError as e:
#       print(e)
#             #### direct conversion to CV2 ####

#     ## Rescale Image size 
#     rescale_factor = 0.5
#     width = int(cv_image.shape[1] * rescale_factor)
#     height = int(cv_image.shape[0] * rescale_factor)
#     dim = (width,height)
#     resized_img = cv2.resize(cv_image, dim)


#     data = asarray(resized_img)
#     img_tensor = toTensor(data)
#     model = hg2(pretrained=True)
#     model = model.cuda()
#     predictor = HumanPosePredictor(model)
#     joints = predictor.estimate_joints(img_tensor, flip=True)
   
# # # # 0 right_ankle   # 4 left_knee     # 8 neck            # 12 right_shoulder
# # # # 1 right_knee    # 5 left_ankle    # 9 head_top        # 13 left_shoulder
# # # # 2 right_hip     # 6 pelvis        # 10 right_wrist    # 14 left_elbow
# # # # 3 left_hip      # 7 spine         # 11 right_elbow    # 15 left_wrist
#     left_arm = [10,11,12,13,14,15]
#     for i in left_arm:
#         cv2.circle(resized_img, (joints[0][i][0],joints[0][i][1]), 10, 0)
#     # cv2.imshow('cv_img', cv_image)
#     # cv2.waitKey(2)

 
#     self.image_pub.publish(self.bridge.cv2_to_imgmsg(resized_img, "bgr8"))

# def main(args):
#   ic = image_converter()
#   rospy.init_node('image_converter', anonymous=True)
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)
# # VERBOSE=False
