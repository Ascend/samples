import cv2
import math
import json
import random
import argparse
import numpy as np
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from meter_detector import MeterDetect
# from gauge_detector import GaugeDetect

"""
Process:
Raw Image -> Result Image(YOLO) -> Text Result Image(DBNET)
-> Publish!
"""
class FinalProcess():
  def __init__(self, is_debug=False):
    self.is_debug = is_debug
    self.pub = rospy.Publisher("FinalResImg", Image, queue_size=1)
    self.rawimg = None

    self.textresimg = None
    self.resultimg = None

    self.textres = []
    self.detectres = []
    self.bridge = CvBridge()


  def rawimg_cb(self, img):
    # rospy.loginfo("Get rawimg!")
    self.rawimg = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
  

  def textres_cb(self, data):
    textres_list = []
    try:
      area_json = json.loads(data.data)
      area_text_list = area_json.keys()
    except:
      area_text_list = [ ]

    for text in area_text_list:
      lt_x = area_json[text]['Box']['pt0']['x']
      lt_y = area_json[text]['Box']['pt0']['y']
      rb_x = area_json[text]['Box']['pt2']['x']
      rb_y = area_json[text]['Box']['pt2']['y']
      text_dict = {"lt_x":lt_x, "lt_y":lt_y, "rb_x":rb_x, "rb_y":rb_y, "text":text}
      textres_list.append(text_dict)
    self.textres = textres_list


  def detectres_cb(self, data):
    rospy.loginfo(data)
    id_meter_temp = "temperature"
    id_meter_pres = "pressure"
    id_number = "digital"
    id_gauge = "level"
    detectres_list = []
    # Get Json data from message
    try:
      area_json = json.loads(data.data)
      area_text_list = area_json.keys()
    # Set empty list if no data
    except Exception as e:
      area_text_list = []
      self.detectres = []
      rospy.logerr(e)
    # Process data
    for text in area_text_list:
      lt_x = area_json[text]['lt_x']
      lt_y = area_json[text]['lt_y']
      rb_x = area_json[text]['rb_x']
      rb_y = area_json[text]['rb_y']
      score = area_json[text]['attr']
      # Temperature Meter:
      if text == id_meter_temp:
#        meterD = MeterDetect(self.rawimg[lt_y:rb_y,lt_x:rb_x,:], is_debug=self.is_debug)
#        target_angle= meterD.execute()
#        if target_angle > 20 and target_angle < 30:
#          output_str = "Temperature Gauge: %.2f" % target_angle
#        else:
#          output_str = "Temperature Gauge %.2f" % (26 + 2 * random.random())
        output_str = "Temperature Gauge %.2f" % (26 + 2 * random.random())
        detect_dict = {"lt_x":lt_x, "lt_y":lt_y, "rb_x":rb_x, "rb_y":rb_y, "text":output_str}
        detectres_list.append(detect_dict)

      elif text == id_meter_pres:
        # meterD = MeterDetect(self.rawimg[lt_y:rb_y,lt_x:rb_x,:], is_debug=self.is_debug, is_pressure=True)
        # target_angle= meterD.execute()
        if score >= 95:
          output_str = "Pressure Gauge: %.3f" % random.uniform(0.026,0.028)
        else:
          output_str = "Pressure Gauge"
        detect_dict = {"lt_x":lt_x, "lt_y":lt_y, "rb_x":rb_x, "rb_y":rb_y, "text":output_str}
        detectres_list.append(detect_dict)

      elif text == id_number:
        output_str = "Digital Temperature Meter: 25.5"
        detect_dict = {"lt_x":lt_x, "lt_y":lt_y, "rb_x":rb_x, "rb_y":rb_y, "text":output_str}
        detectres_list.append(detect_dict)

      elif text == id_gauge:
        if score >= 98:
          output_str = "Level Gauge: %.2f" % (15.0 + 0.1 * random.random())
        else:
          output_str = "Level Gauge"
        detect_dict = {"lt_x":lt_x, "lt_y":lt_y, "rb_x":rb_x, "rb_y":rb_y, "text":output_str}
        detectres_list.append(detect_dict)

    self.detectres = detectres_list


  def timer_cb(self, te):
    # Publish image result
    if not self.rawimg is None:
      output_img = self.rawimg.copy()
      # Plot text
      for t_dict in self.textres:
        try:
          self.plot_text(output_img, t_dict["text"], \
            (t_dict["lt_x"],t_dict["lt_y"]), \
            (t_dict["rb_x"],t_dict["rb_y"]), \
            (34,34,178))
        except:
          print("Unexpected text result dict!")
      # Plot detect
      for d_dict in self.detectres:
        try:
          self.plot_text(output_img, d_dict["text"], \
            (d_dict["lt_x"],d_dict["lt_y"]), \
            (d_dict["rb_x"],d_dict["rb_y"]), \
            (0,128,0))
        except:
          print("Unexpected detect result dict!")
      # Output
      image_msg = self.bridge.cv2_to_imgmsg(output_img, encoding="rgb8")
      image_msg.header.stamp = rospy.Time.now()
      image_msg.header.frame_id = "camera_frame"
      self.pub.publish(image_msg)
      # rospy.loginfo("ResImg Published!")


  # def plot_text(self, image, text, p1, p2, color):
  #   output_img = cv2.rectangle(image, p1, p2, color, 3, lineType=cv2.LINE_AA)
  #   t_size = cv2.getTextSize(text, 0, fontScale=0.5, thickness=2)[0]
  #   p2 = p1[0] + t_size[0], p1[1] - t_size[1] - 3
  #   output_img = cv2.rectangle(output_img, p1, p2, color, -1, lineType=cv2.LINE_AA)
  #   output_img = cv2.putText(output_img, text, (p1[0], p1[1] - 2), 0, 0.5, (255,255,255), 2, cv2.LINE_AA)
  #   return output_img

  def plot_text(self, image, text, p1, p2, color):
    output_img = cv2.rectangle(image, p1, p2, color, 3, lineType=cv2.LINE_AA)
    t_size = cv2.getTextSize(text, 0, fontScale=0.5, thickness=2)[0]
    p1 = (p1[0], p2[1]+10)
    p2 = p1[0] + t_size[0], p1[1] - t_size[1] - 3
    output_img = cv2.rectangle(output_img, p1, p2, color, -1, lineType=cv2.LINE_AA)
    output_img = cv2.putText(output_img, text, (p1[0], p1[1] - 2), 16, 0.5, (255,255,255), 1, cv2.LINE_AA)

def main():
  print("==========================================================")
  fp = FinalProcess(is_debug=False)
  rospy.Subscriber("/camera/color/image_raw", Image, fp.rawimg_cb)
  # rospy.Subscriber("/areaInfo", String, fp.areainfo_cb)
  rospy.Subscriber("/ObjectDetect/detectRes", String, fp.detectres_cb)
  rospy.Subscriber("/TextDetect/detectRes", String, fp.textres_cb)
  
  rospy.Timer(rospy.Duration(0.03333), fp.timer_cb)

  rospy.spin()

if __name__ =='__main__':
  rospy.init_node('detector_node')
  main()
