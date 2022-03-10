import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

class MeterDetect():
  def __init__(self, target_img, is_pressure=False, is_debug=False):
    self.is_debug = is_debug
    self.is_pressure = is_pressure
    self.target = cv2.cvtColor(target_img, cv2.COLOR_BGR2RGB)
    h, w, c = self.target.shape
    self.cx, self.cy = h / 2, w / 2
    self.target_edges = None
    # Update cx, cy, edges
    self.__calc_center(is_binary=True, method="Canny")
    # self.__calc_center(is_binary=False, method="Canny")


  def execute(self):
    # Check if edges
    # if self.target_edges is None and self.is_debug:
    if self.target_edges is None:
      print("No edges!")
      return 0
    
    # Line segment detection
    length_threshold = 25
    distance_threshold = 1.41421356
    canny_th1 = 50.0
    canny_th2 = 100.0
    canny_aperture_size = 3
    # do_merge = False
    do_merge = True
    fld = cv2.ximgproc.createFastLineDetector(length_threshold,distance_threshold, \
      canny_th1,canny_th2,canny_aperture_size,do_merge)
    # fld = cv2.ximgproc.createFastLineDetector()
    lines = fld.detect(self.target_edges)

    # Check length of lines
    if lines is None:
      return 0

    # Calc length
    length_number_threshold = 20 if len(lines) > 20 else len(lines)
    good_lines = []
    lines_length = []
    for i, line in enumerate(lines):
      line = line[0]
      x1, y1 = line[0], line[1]
      x2, y2 = line[2], line[3]
      length = math.sqrt(pow(x1-x2,2) + pow(y1-y2,2))
      lines_length.append([i, length])
    lines_length.sort(reverse=True, key=lambda x:x[1])
    good_lines = []
    for i in range(length_number_threshold):
      id = lines_length[i][0]
      good_lines.append(lines[id])

    # Calc distance
    lines_dist = []
    for i, line in enumerate(good_lines):
      dist,_ = self.__center_dist_calc(line)
      lines_dist.append([i, dist])
    lines_dist.sort(reverse=False, key=lambda x:x[1])
    index = lines_dist[0][0]

    # Get target line segment
    target_line = good_lines[index]
    _, target_grad = self.__center_dist_calc(target_line)
    target_angle = math.atan(-target_grad) / math.pi * 180
    quadrant = self.__calc_pointer_quadrant([target_line])
    # trick
    if quadrant == 1: quadrant = 3
    if quadrant == 2: quadrant = 4
    
    if quadrant in [2,3]:
      target_angle += 180
    if target_angle < 0:
      target_angle += 360
    actural_angl= self.__angle_mapping(target_angle)

    if self.is_debug:
      self.__draw_result(actural_angl, target_grad)
      self.__draw_lines(good_lines)
    return actural_angl


  def __center_dist_calc(self, line):
    c_x, c_y = self.cx, self.cy
    x1, y1, x2, y2 = line[0]
    A = (y1-y2)/(x1-x2)
    B = -1
    C = y1 - A * x1
    dist = abs(A*c_x+B*c_y+C)/math.sqrt(A*A+B*B)
    return dist, A
  

  def __calc_pointer_quadrant(self, better_lines):
    quadrant_list = np.zeros(4)
    for line in better_lines:
      x1, y1, x2, y2 = line[0]
      x_m, y_m = (x1+x2)/2, (y1+y2)/2
      mid_quad = self.__get_quadrant([self.cx, self.cy], [x_m, y_m])
      quadrant_list[mid_quad-1] += 1
    return np.argmax(quadrant_list)


  def __get_quadrant(self, center, points):
    c_x, c_y = center
    x, y = points
    if x >= c_x and y <= c_y:
      return 1
    elif x > c_x and y > c_y:
      return 2
    elif x <= c_x and y >= c_y:
      return 3
    elif x < c_x and y < c_y:
      return 4


  def __angle_mapping(self, angle):
    if angle > 225 and angle < 315:
      # raise Exception("Illegal dashboard angle")
      print("Illegal dashboard angle")
      return 0

    if angle <= 225:
      b = 225/2.7
      a = -b/225
    else:
      a = -(100 - 225/2.7) / 45
      b = 225/2.7 - 360 * a
    
    if self.is_pressure:
      return (a * angle + b) / 400
    else:
      return a * angle + b


  def __draw_result(self, angle, grad):
    x = np.linspace(0, self.cx*2, 100)
    a, b = grad, self.cy - grad * self.cx
    temperature = '%.2f' % angle
    plt.axis('off')
    plt.imshow(self.target)
    plt.plot(x, a*x+b, c='g', linewidth=5)
    # plt.text(2,30,temperature, fontsize=20, c='g', fontfamily='monospace')
    plt.text(2,30,temperature, fontsize=20, color='g')
    plt.show()


  def __draw_lines(self, lines):
    fld = cv2.ximgproc.createFastLineDetector()
    line_on_image = fld.drawSegments(self.target, np.array(lines, dtype=np.float32))
    plt.imshow(line_on_image)
    plt.show()


  def __calc_center(self, is_binary=False, method="Canny"):
    h, w, c = self.target.shape
    img = cv2.cvtColor(self.target, cv2.COLOR_BGR2GRAY)
    img = cv2.equalizeHist(img)
    if is_binary:
      img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2) # Adaptive binary
      # img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,3)       # Adaptive binary
    img = cv2.medianBlur(img, 1)                        # Set Blur
    img = cv2.GaussianBlur(img, (3, 3), 0)              # Set Blur
    if method == "Canny":
      edges = cv2.Canny(img, 50, 100, apertureSize=3)
    else:
      grad_x = cv2.Sobel(img, -1, 1, 0, ksize=3)
      grad_y = cv2.Sobel(img, -1, 0, 1, ksize=3)
      edges  = cv2.addWeighted(grad_x, 0.5, grad_y, 0.5, 0)
    self.target_edges = edges
    # Calc circle
    circle = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1.5, h/4, 200, 100)

    if circle is None or circle.size==0 or not len(circle.shape)==3:
      print("Failed to find circle!")
      return

    circle = circle[0].tolist()
    circle.sort(reverse=True, key=lambda x:x[2])
    self.cx, self.cy = circle[0][0], circle[0][1]
    if self.is_debug:
      print("Circle Found: %d, %d" % (self.cx, self.cy))
    return