import tkinter as tk
import cv2
import numpy as np
import mediapipe as mp
from asset import detect_faces_mediapipe, GazeEstimator, gaze2point, mm2px, px2mm
import queue
import threading
import time
import math

calibration_start = False
point_q = queue.Queue(maxsize=15)
dxdy_q = queue.Queue()
point_dx,point_dy = 0,0


f_x,f_y = 0,0

tx, ty = 0, 0
ex, ey = 0, 0
gp_norma_angles = None
gc_norma_angles = None

frame = None
R = None
count = 0
face_center = None
normalized_gaze_vector_p = None
head_pose = None


point_15P = np.array([[250, 200], [604, 200], [958, 200], [1308, 200], [1655, 200],
                      [250, 533], [604, 533], [958, 533], [1308, 533], [1655, 533],
                      [250, 865], [604, 865], [958, 865], [1308, 865], [1655, 865]
                      ])
point_len = len(point_15P)
iter, point_index = 0, 0  # iter<5, point_index<23
iter_max = 1
isEnd = False

huawei_angle_list = []
huawei_point_list = []

def gazeEstimate(estor,face, frame):
    estor.estimate(face, frame)

    x, y = gaze2point(face.center * 1e3, face.gaze_vector)
    x, y = mm2px((x, y))
    x, y = int(x), int(y)

    return x,y

def gazeTo2d(gaze):
    yaw = np.arctan2(-gaze[0], -gaze[2])
    pitch = np.arcsin(-gaze[1])
    return np.array([pitch, yaw])

def angular(gaze, label):
    total = np.sum(gaze * label)
    return np.arccos(min(total / (np.linalg.norm(gaze) * np.linalg.norm(label)), 0.9999999)) * 180 / np.pi


def randomBtFunc(self_bt,endBt2):
    global head_pose, f_x,f_y, huawei_angle_list,huawei_point_list
    global R, face_center, normalized_gaze_vector_p, count, tx, ty, iter, point_index, point_15P, point_len, isEnd

    point_px = np.array([tx,ty])  # 真实的视线点（认为设置的点）
    # print("point_px",point_px)
    point_px2 = np.array([f_x,f_y])   # f_x,f_y加上偏差后的视线点
    # print("point_px_bias", point_px2)
    # error_mm = px2mm(np.array([error_x, error_y]))
    # print("error_distance_x_y ",error_x,error_y)
    # error_distance = math.sqrt(error_mm[0]**2+error_mm[1]**2)
    # print("error_diastance(mm) ",error_distance)
    #
    # huawei_point_list.append(error_distance)

    # 需要将像素变换成 m为单位的
    point_mm = px2mm(point_px)  # 注视点坐标由pixel变为mm
    point_mm = np.append(np.array(point_mm), 0) / 1000

    point_mm2 = px2mm(point_px2)  # 注视点坐标由pixel变为mm
    point_mm2 = np.append(np.array(point_mm2), 0) / 1000

    print("第" + str(count+1)+"个点")
    point_error = point_mm2-point_mm
    print("error_x_y ",point_error)
    error_distance = math.sqrt(point_error[0] ** 2 + point_error[1] ** 2)
    huawei_point_list.append(error_distance)
    print("error_diastance(cm) ", error_distance*100)

    gc = point_mm - face_center
    gc_normalized = R @ gc
    gc_normalized = gc_normalized / np.linalg.norm(gc_normalized)
    gc_norma_angles = gazeTo2d(gc_normalized)  # 从视线方向向量转为视线角度

    gc2 = point_mm2 - face_center
    gc2_normalized = R @ gc2
    gc2_normalized = gc2_normalized / np.linalg.norm(gc2_normalized)
    gc2_norma_angles = gazeTo2d(gc2_normalized)  # 从视线方向向量转为视线角度

    angular_error = angular(gc_normalized, gc2_normalized)
    print("angular_error", angular_error)

    huawei_angle_list.append(angular_error)
    count += 1  # 记录

    if point_index < point_len and iter < iter_max:
        point_index = count % point_len

    if count < point_len * iter_max:
        tx = point_15P[point_index][0]
        ty = point_15P[point_index][1]
        self_bt.place(x=tx, y=ty)
    else:
        self_bt.place_forget()
        endBt2.place(x=880, y=480)
    # tx = random.randint(0, 1905)  # 1920-15
    # ty = random.randint(0, 1065)  # 1080-15

##左上顶点的点击事件 点(250,200)
def bt_left_topFunc(self_bt, bt_right_top):
    global point_q,dxdy_q

    if point_q.full():
        x, y = np.mean(point_q.queue, 0).astype(np.int)
        dx = 250+7 - x
        dy = 200+7 - y
        dxdy_q.put([dx, dy])
        # print("dx %d, dy %d" %(dx, dy))
        self_bt.place_forget()
        bt_right_top.place(x=1670, y=200)

#右上顶点的点击事件 点(1670,200)
def bt_right_topFunc(self_bt, bt_left_bottom):
    global point_q, dxdy_q

    #计算该点的偏差dx,dy
    x, y = np.mean(point_q.queue, 0).astype(np.int)
    dx = 1670+7 - x
    dy = 200+7 - y
    dxdy_q.put([dx, dy])
    # print("dx %d, dy %d" %(dx, dy))

    self_bt.place_forget()
    bt_left_bottom.place(x=250, y=880)
    # bt_right_top.place(x=1670,y=200)


#左下底点 (250,880)
def bt_left_bottomFunc(self_bt, bt_right_bottom):
    global point_q, dxdy_q

    x, y = np.mean(point_q.queue, 0).astype(np.int)
    dx = 250+7 - x
    dy = 880+7 - y
    dxdy_q.put([dx, dy])
    # print("dx %d, dy %d" % (dx, dy))

    self_bt.place_forget()
    bt_right_bottom.place(x=1670, y=880)
    # bt_right_top.place(x=1670,y=200)

#右下底点 (1670,880)
def bt_right_bottomFunc(self_bt,endbt,resetbt):
    global point_q, dxdy_q

    # 计算该点的偏差dx,dy
    x, y = np.mean(point_q.queue, 0).astype(np.int)  # 取坐标的平均值作为估计点
    dx = 1670+7 - x
    dy = 880+7 - y
    dxdy_q.put([dx, dy])
    # print("dx %d, dy %d" %(dx, dy))
    # print(dxdy_q.queue)

    self_bt.place_forget()
    endbt.place(x=880, y=380)
    resetbt.place(x=880, y=580)

# 开始校准
def startBtFunc(self_bt, bt_left_top):
    global calibration_start
    calibration_start = True
    self_bt.place_forget()
    bt_left_top.place(x=250, y=200)  # 根据像素坐标设置点的位置


def endBt2Func(window):
    global huawei_point_list,huawei_angle_list
    mean_point_distance = np.mean(np.array(huawei_point_list))
    mean_point_angle = np.mean(np.array(huawei_angle_list))
    print("mean_distance(cm) ",mean_point_distance*100, " mean_angle(Degree)", mean_point_angle)

    window.destroy()

# 校准结束
def endBtFunc(self_bt,resetbt,randBt):
    global calibration_start, dxdy_q, point_dx, point_dy,tx,ty
    global lt_dx, lt_dy, rt_dx, rt_dy, lb_dx, lb_dy, rb_dx, rb_dy, middle_x, middle_y

    calibration_start = False
    # print(dxdy_q.queue)

    point_dx, point_dy = np.mean(dxdy_q.queue, 0).astype(np.int)  # 取坐标的平均值作为校准后的估计点偏置量

    self_bt.place_forget()
    resetbt.place_forget()
    randBt.place(x=250, y=200)
    # 初始的tx,ty值
    tx = point_15P[point_index][0]
    ty = point_15P[point_index][1]
    #point_dx, point_dy = np.mean(dxdy_q.queue, 0).astype(np.int)  # 取坐标的平均值作为校准后的估计点偏置量
    # print(point_dx,point_dy)
    # window.destroy()


def resetBtFunc(resetbt,endbt,bt_left_top):
    global dxdy_q, point_dx, point_dy

    while not dxdy_q.empty():
        dxdy_q.get()

    resetbt.place_forget()
    endbt.place_forget()
    bt_left_top.place(x=250, y=200)


def UICalibration():
    window = tk.Tk()
    # 设置并调整窗口的大小、位置
    window.geometry('1920x1080')
    window.attributes('-fullscreen', True)  # 设置全屏模式，去掉标题栏等边框

    redpoint = tk.PhotoImage(file="../Client/data/imgs/path870.png")

    endBt2 = tk.Button(window, text='测试结束', font=25, width=15, height=5,command=lambda: endBt2Func(window))
    endBt2["border"] = "1"

    random_bt = tk.Button(window, image=redpoint, activebackground='#7CCD7C', width=15, height=15,
                          command=lambda: randomBtFunc(random_bt,endBt2))
    random_bt["border"] = "0"


    endBt = tk.Button(window, text='校准结束', font=25, width=15, height=5,
                      command=lambda: endBtFunc(endBt,resetBt,random_bt))
    endBt["border"] = "1"

    resetBt = tk.Button(window, text='重新校准', font=25, width=15, height=5,
                        command=lambda: resetBtFunc(resetBt, endBt, bt_left_top))
    resetBt["border"] = "1"

    bt_right_bottom = tk.Button(window, image=redpoint, activebackground='#7CCD7C', width=15, height=15,
                                command=lambda: bt_right_bottomFunc(bt_right_bottom, endBt, resetBt))
    bt_right_bottom["border"] = "0"

    bt_left_bottom = tk.Button(window, image=redpoint, activebackground='#7CCD7C', width=15, height=15,
                               command=lambda: bt_left_bottomFunc(bt_left_bottom, bt_right_bottom))
    bt_left_bottom["border"] = "0"

    bt_right_top = tk.Button(window, image=redpoint, activebackground='#7CCD7C', width=15, height=15,
                             command=lambda: bt_right_topFunc(bt_right_top, bt_left_bottom))
    bt_right_top["border"] = "0"

    bt_left_top = tk.Button(window, image=redpoint, activebackground='#7CCD7C', width=15, height=15,
                            command=lambda: bt_left_topFunc(bt_left_top, bt_right_top))
    bt_left_top["border"] = "0"

    startBt = tk.Button(window, text='开始校准', font=25, width=15, height=5,
                        command=lambda: startBtFunc(startBt, bt_left_top))
    startBt["border"] = "1"
    startBt.place(x=880, y=480)

    # 点击按钮时执行的函数sss
    # button = tk.Button(window, text='点击前往', activebackground='#7CCD7C', activeforeground="#555555", width=20, height=20, command=click_button).pack()
    # 显示窗口
    window.mainloop()


def Estimate():
    global R, face_center, normalized_gaze_vector_p, head_pose

    global point_q, calibration_start, point_dx,point_dy,f_x,f_y

    cv2.namedWindow("boom", cv2.WND_PROP_FULLSCREEN)
    # cv2.moveWindow("boom", 1920,0)
    cv2.setWindowProperty("boom",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

    checkpoint_path = " "
    camera_params = "../data/sample_params.yaml"
    normalized_camera_params  = "../data/eth-xgaze.yaml"

    estimator = GazeEstimator(checkpoint_path, camera_params, normalized_camera_params, model_name="resnet18")

    detector = mp.solutions.face_mesh.FaceMesh(static_image_mode=False,max_num_faces=1,min_detection_confidence=0.5,min_tracking_confidence=0.5)

    boom = np.ones((1080,1920))

    # 调用笔记本摄像头
    cap = cv2.VideoCapture(0)

    xy_q = queue.Queue(maxsize=10)  #设置队列最大长度

    frameCount = 0

    t1 = time.time()

    while True:
        ret, frame = cap.read()
        if not ret :
            break

        faces = detect_faces_mediapipe(detector, frame)

        if faces:
            frameCount += 1  #记录帧数
            # print(time.time()-t1)
            for face in faces:

                x,y = gazeEstimate(estimator,face,frame)

                R = face.normalizing_rot.as_matrix()
                face_center = face.center
                normalized_gaze_vector_p = face.normalized_gaze_vector
                head_pose = face.normalized_head_rot2d

                if calibration_start:
                    if not point_q.full():
                        point_q.put([x, y])
                    else:
                        point_q.get()
                        point_q.put([x, y])
                #print(calibration_start,point_q.queue)

                if not xy_q.full():
                    xy_q.put([x,y])
                else:
                    xy_q.get()
                    xy_q.put([x,y])
                x,y = np.mean(xy_q.queue,0).astype(np.int)

                x = x + point_dx
                y = y + point_dy

                f_x = x
                f_y = y

                # print("After Calibration", x, y," dx=%d dy=%d" % (point_dx,point_dy))

                canvas = cv2.circle(boom.copy(),(x,y),10,(0,0,0),-1)
                frame = cv2.flip(frame,1)
                cv2.putText(canvas,f"{x},  {y}",
                    (50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,0),2)

                cv2.imshow("boom",canvas)
                cv2.imshow("face",cv2.flip(face.normalized_image,1))

        k = cv2.waitKey(1)
        if k == ord('q'):
            break
        # window.mainloop()

    cv2.destroyAllWindows()
    cap.release()
    t2 = time.time()
    print("时长", (t2 - t1), " frame", frameCount, "fps",frameCount/(t2-t1-0.7))

cap_thread = threading.Thread(target=Estimate,name="Estimate",args={})
cap_thread.start()
UICalibration()

