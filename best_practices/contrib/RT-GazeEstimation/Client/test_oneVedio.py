import tkinter as tk
import cv2
import numpy as np
import mediapipe as mp
from asset import detect_faces_mediapipe, GazeEstimator, gaze2point, mm2px
import queue
import threading
import time

calibration_start = False
point_q = queue.Queue(maxsize=15)
dxdy_q = queue.Queue()
point_dx,point_dy = 0,0

def gazeEstimate(estor,face, frame):
    estor.estimate(face, frame)

    x, y = gaze2point(face.center * 1e3, face.gaze_vector)
    x, y = mm2px((x, y))
    x, y = int(x), int(y)

    return x,y

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


def bt_right_topFunc(self_bt, bt_left_bottom):
    global point_q, dxdy_q

    x, y = np.mean(point_q.queue, 0).astype(np.int)
    dx = 1670+7 - x
    dy = 200+7 - y
    dxdy_q.put([dx, dy])
    # print("dx %d, dy %d" %(dx, dy))

    self_bt.place_forget()
    bt_left_bottom.place(x=250, y=880)
    # bt_right_top.place(x=1670,y=200)

def bt_left_bottomFunc(self_bt, bt_right_bottom):
    global point_q, dxdy_q

    # 计算该点的偏差dx,dy
    x, y = np.mean(point_q.queue, 0).astype(np.int)
    dx = 250+7 - x
    dy = 880+7 - y
    dxdy_q.put([dx, dy])
    # print("dx %d, dy %d" % (dx, dy))

    self_bt.place_forget()
    bt_right_bottom.place(x=1670, y=880)
    # bt_right_top.place(x=1670,y=200)

def bt_right_bottomFunc(self_bt,endbt,resetbt):
    global point_q, dxdy_q

    # 计算该点的偏差dx,dy
    x, y = np.mean(point_q.queue, 0).astype(np.int)
    dx = 1670+7 - x
    dy = 880+7 - y
    dxdy_q.put([dx, dy])

    self_bt.place_forget()
    endbt.place(x=880, y=380)
    resetbt.place(x=880, y=580)
def startBtFunc(self_bt, bt_left_top):
    global calibration_start
    calibration_start = True
    self_bt.place_forget()
    bt_left_top.place(x=250, y=200)

def endBtFunc(bt1,bt2,bt3,bt4,bt5,bt6,bt7,window):
    global calibration_start, dxdy_q, point_dx, point_dy
    calibration_start = False
    # print(dxdy_q.queue)
    point_dx, point_dy = np.mean(dxdy_q.queue, 0).astype(np.int)
    # print(point_dx,point_dy)

    bt1.destroy()
    bt2.destroy()
    bt3.destroy()
    bt4.destroy()
    bt5.destroy()
    bt6.destroy()
    bt7.destroy()
    window.destroy()


def resetBtFunc(resetbt,endbt,bt_left_top):
    global dxdy_q, point_dx, point_dy

    while not dxdy_q.empty():
        dxdy_q.get()

    resetbt.place_forget()
    endbt.place_forget()
    bt_left_top.place(x=250, y=200)

def UICalibration():
    window = tk.Tk()
    window.attributes('-fullscreen', True)  # 设置全屏模式，去掉标题栏等边框
    window.geometry('1920x1080')

    redpoint = tk.PhotoImage(file="./data/imgs/path870.png")

    endBt = tk.Button(window, text='校准结束', font=25, width=15, height=5,
                        command=lambda: endBtFunc(endBt,bt_left_top,bt_right_top,bt_left_bottom,bt_right_bottom,startBt,resetBt,window))
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
    # 右上红点
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

    # 显示窗口
    window.mainloop()


def Estimate():

    global point_q, calibration_start, point_dx,point_dy

    cv2.namedWindow("boom", cv2.WND_PROP_FULLSCREEN)
    # cv2.moveWindow("boom", 1920,0)
    cv2.setWindowProperty("boom",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

    checkpoint_path = " "  # 为空
    camera_params = "./data/sample_params.yaml"
    normalized_camera_params  = "./data/eth-xgaze.yaml"

    estimator = GazeEstimator(checkpoint_path, camera_params, normalized_camera_params, model_name="resnet18")

    detector = mp.solutions.face_mesh.FaceMesh(static_image_mode=False,max_num_faces=1,min_detection_confidence=0.5,min_tracking_confidence=0.5)

    boom = np.ones((1080,1920))

    vedio_path = "data/face.avi"
    # 调用笔记本摄像头
    cap = cv2.VideoCapture(vedio_path)

    xy_q = queue.Queue(maxsize=10)

    while True:
        ret, frame = cap.read()
        if not ret :
            break

        faces = detect_faces_mediapipe(detector, frame)

        if faces:
            # print(time.time()-t1)
            for face in faces:

                x,y = gazeEstimate(estimator,face,frame)

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

cap_thread = threading.Thread(target=Estimate,name="Estimate",args={})
cap_thread.start()
# UICalibration()

