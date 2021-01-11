# !/usr/bin/env python
# -*- coding:utf-8 -*-
from multiprocessing import Process, Queue, Manager, set_start_method
import sys


import acl


SUCCESS = 0
FAILED = 1
ACL_ERROR_NONE = 0



def SubProcessEntry():
    ret = acl.init()
    acl.rt.set_device(0)
    print("set device in subprocess.")

    context, ret = acl.rt.create_context(0)
    #self.context, ret = acl.rt.get_context()
    if (context is None) or (ret != ACL_ERROR_NONE):
        print("acl.rt.create_context failed in subporcess, ", ret)
        return FAILED
	
    # for i in range(0, 30):
    #     time.sleep(0.1)


def main():

    set_start_method('spawn')

    sub_process = Process(target=SubProcessEntry)
    sub_process.start() 

    sub_process1 = Process(target=SubProcessEntry)
    sub_process1.start()  

    ret = acl.init()
    if ret != ACL_ERROR_NONE:
        print("acl init failed")
        return

    ret = acl.rt.set_device(0)
    if ret != ACL_ERROR_NONE:
        print("acl set device 0 failed")
        return

    ctx1, ret = acl.rt.create_context(0)
    stream, ret = acl.rt.create_stream()
    print(stream)


    ctx2, ret = acl.rt.create_context(0)
    stream2, ret = acl.rt.create_stream()

    print(stream2)

    ret = acl.rt.set_context(ctx1)

     
    			
    # for i in range(0, 100):
    #     time.sleep(0.1)

    acl.rt.reset_device(0)
    acl.finalize()

if __name__ == '__main__':
    main()



# # !/usr/bin/env python
# # -*- coding:utf-8 -*-
# import os
# import time
# import configparser
# from multiprocessing import Process, Queue, Manager
# import queue
# import numpy as np
# import sys
# #sys.path.append("..")

# import acl

# DEVICE_ID = 0
# IMAGE_INPUT_WIDTH = 1280
# IMAGE_INPUT_HEIGHT = 720

# VENC_STATUS_ERROR = -1
# VENC_STATUS_INIT = 1
# VENC_STATUS_WORK = 1
# VENC_STATUS_EXIT = 2

# SUBSCRIBE_TIMEOUT = 1000

# SUCCESS = 0
# FAILED = 1
# ACL_ERROR_NONE = 0

# class VencMessage(object):
#     def __init__(self, name, data):
#         self.name = name
#         self.data = data

# class AclVenc(object):
#     def __init__(self, width, height):
#         self.msg_queue = Queue()
#         self.width = Manager().Value('i', width)
#         self.height =  Manager().Value('i', height)
#         self.status = Manager().Value('i', VENC_STATUS_INIT)
    
#     def init_resource(self):
#         venc_process = Process(target=VencProcessEntry,
#                                args=(self.msg_queue, self.width, self.height, self.status))
#         venc_process.start()        
      
#         for i in range(0, 1000):
#             if self.status.value != VENC_STATUS_INIT:
#                 break
#             time.sleep(0.01)
        
#         return SUCCESS if self.status.value == VENC_STATUS_WORK else FAILED

#     def process(self, image, is_finished=False):
#         if self.status.value != VENC_STATUS_WORK:
#             print("Venc status(%d) is invalid, "
#                   "process image failed"%(self.status.value))
#             return FAILED

#         if (is_finished == False) and (image is None):
#             print("Process frame venc failed for data is None")
#             return FAILED
        
#         msg = None
#         if is_finished:
#             msg = VencMessage("exit", None)
#         else:
#             msg = VencMessage("frame_data", None)

#         self.msg_queue.put(msg)


# class AclVencProcess(object):
#     def __init__(self, width, height):
#         self._width = width
#         self._height = height
#         self._size = width * height * 3 / 2
#         self.venc_channel_desc = 0
#         self.dvpp_pic_desc = 0
#         self.frame_config = None
#         self.cb_thread_id = None
#         self.context = None
#         self.callback_run_flag = True

#     def __del__(self):
#         if self.venc_channel_desc != 0:
#             acl.media.venc_destroy_channel(self.venc_channel_desc)
#             acl.media.venc_destroy_channel_desc(self.venc_channel_desc)
#         if self.dvpp_pic_desc != 0:
#             acl.media.dvpp_destroy_pic_desc(self.dvpp_pic_desc)
#         if self.frame_config:
#             acl.media.venc_destroy_frame_config(self.frame_config)
#         #acl.rt.destroy_context(self.context)

#     def init_resource(self):
#         print("init resource stage:")

#         ret = acl.rt.set_device(DEVICE_ID)
#         #check_ret("acl.rt.set_device", ret)

#         self.context, ret = acl.rt.create_context(DEVICE_ID)
#         #self.context, ret = acl.rt.get_context()
#         if (self.context is None) or (ret != ACL_ERROR_NONE):
#             print("acl.rt.create_context", ret)
#             return FAILED
#         print("Create venc context ok")

#         self.cb_thread_id, ret = acl.util.start_thread(
#             self.cb_thread_func, [self.context, SUBSCRIBE_TIMEOUT])
#         if ret != ACL_ERROR_NONE:
#             print("Start venc subscribe thread failed, error ", ret)
#             return FAILED
#         print("Create venc subscribe thread ok")
        
#         self.frame_config = acl.media.venc_create_frame_config()
#         if self.frame_config is None:
#             print("Create venc frame config failed")
#         print("Create venc frame config ok")
#         self.venc_set_frame_config(self.frame_config, 0, 0)
#         self.venc_get_frame_config(self.frame_config)

#         self.venc_channel_desc = acl.media.venc_create_channel_desc()
#         if self.venc_channel_desc is None:
#             print("acl.media.venc_create_channel_desc failed")
#             return FAILED
#         print("Create venc channel desc ok")
        
#         self.set_channel_desc()
#         print("Set venc channel desc ok")

#         ret = acl.media.venc_create_channel(self.venc_channel_desc)
#         if ret != ACL_ERROR_NONE:
#             print("Create venc channel failed, error ", ret)
#             return FAILED
#         print("Create venc channel ok")



#         self.dvpp_pic_desc = acl.media.dvpp_create_pic_desc()
#         ret = acl.media.dvpp_set_pic_desc_size(self.dvpp_pic_desc, image.size)
#         print("Set pic desc size ok")

#         return SUCCESS

#     def set_channel_desc(self):
#         # venc_channel_desc set function
#         acl.media.venc_set_channel_desc_thread_id(
#             self.venc_channel_desc, self.cb_thread_id)

#         acl.media.venc_set_channel_desc_callback(
#             self.venc_channel_desc, self.callback_func)
#         acl.media.venc_set_channel_desc_entype(
#             self.venc_channel_desc, VENC_ENTYPE_H264_MAIN)
#         acl.media.venc_set_channel_desc_pic_format(
#             self.venc_channel_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
#         key_frame_interval = 16
#         acl.media.venc_set_channel_desc_key_frame_interval(
#             self.venc_channel_desc, key_frame_interval)
#         acl.media.venc_set_channel_desc_pic_height(
#             self.venc_channel_desc, self._height)
#         acl.media.venc_set_channel_desc_pic_width(
#             self.venc_channel_desc, self._width)

#     def cb_thread_func(self, args_list):
#         context = args_list[0]
#         timeout = args_list[1]
#         print("[INFO] cb_thread_func args_list = ", context, timeout,
#               self.callback_run_flag)

#         context, ret = acl.rt.create_context(DEVICE_ID)
#         if ret != 0:
#             print("[INFO] cb_thread_func acl.rt.create_context ret=", ret)
#             return
#         while self.callback_run_flag is True:
#             ret = acl.rt.process_report(timeout)

#         ret = acl.rt.destroy_context(context)
#         print("[INFO] cb_thread_func acl.rt.destroy_context ret=", ret)

#     def callback_func(self, input_pic_desc, output_stream_desc, user_data):
#         if output_stream_desc == 0:
#             print("[INFO] [venc] output_stream_desc is null")
#             return
#         stream_data = acl.media.dvpp_get_stream_desc_data(output_stream_desc)
#         if stream_data is None:
#             print("[INFO] [venc] acl.media.dvpp_get_stream_desc_data is none")
#             return
#         ret = acl.media.dvpp_get_stream_desc_ret_code(output_stream_desc)
#         if ret == 0:
#             stream_data_size = acl.media.dvpp_get_stream_desc_size(
#                 output_stream_desc)
#             print("[INFO] [venc] stream_data size", stream_data_size)
#             # stream memcpy d2h
#             np_data = np.zeros(stream_data_size, dtype=np.byte)
#             np_data_ptr = acl.util.numpy_to_ptr(np_data)
#             ret = acl.rt.memcpy(np_data_ptr, stream_data_size,
#                                 stream_data, stream_data_size,
#                                 memcpy_kind.get("ACL_MEMCPY_DEVICE_TO_HOST"))
#             if ret != 0:
#                 print("[INFO] [venc] acl.rt.memcpy ret=", ret)
#                 return
#             with open('./data/output.h265', 'ab') as f:
#                 f.write(np_data)

#     def venc_set_frame_config(self, frame_confg, eos, iframe):
#         acl.media.venc_set_frame_config_eos(frame_confg, eos)
#         acl.media.venc_set_frame_config_force_i_frame(frame_confg, iframe)

#     def venc_get_frame_config(self, frame_confg):
#         get_eos = acl.media.venc_get_frame_config_eos(frame_confg)
#         if get_eos != ACL_ERROR_NONE:
#             print("Venc get frame config failed")
#             return FAILED

#         get_force_frame = acl.media.venc_get_frame_config_force_i_frame(
#             frame_confg)
#         if get_force_frame != ACL_ERROR_NONE:
#             print("Venc get frame config force I frame failed")
#             return FAILED
#         return SUCCESS

#     def venc_process(self, image, is_finished=False):
#         if is_finished == False:
#             acl.media.dvpp_set_pic_desc_data(self.dvpp_pic_desc, image.data())

#             ret = acl.media.venc_send_frame(
#                 self.venc_channel_desc,
#                 self.dvpp_pic_desc, 0, self.frame_config, None)
#             if ret != ACL_ERROR_NONE:
#                 print("Venc send frame failed, error ", ret)
#         else:
#             self.venc_set_frame_config(self.frame_config, 1, 0)
#             # send eos frame
#             print("[INFO] venc send frame eos")
#             ret = acl.media.venc_send_frame(
#                 self.venc_channel_desc, 0, 0, self.frame_config, None)
#             if ret != ACL_ERROR_NONE:
#                 print("Venc send eos frame failed, error ", ret)
#             self.thread_join()

#     def thread_join(self):
#         self.callback_run_flag = False
#         ret = acl.util.stop_thread(self.cb_thread_id)
#         print("[INFO] stop_thread", ret)

#     def venc_stream_desc_set(self):
#         stream_format = 0
#         timestamp = 123456
#         ret_code = 1
#         eos = 1

#         # stream desc
#         dvpp_stream_desc = acl.media.dvpp_create_stream_desc()
#         if dvpp_stream_desc is None:
#             print("Create venc stream desc failed")
#             return FAILED

#         # stream_desc set function
#         acl.media.dvpp_set_stream_desc_format(dvpp_stream_desc, stream_format)
#         acl.media.dvpp_set_stream_desc_timestamp(dvpp_stream_desc, timestamp)
#         acl.media.dvpp_set_stream_desc_ret_code(dvpp_stream_desc, ret_code)
#         acl.media.dvpp_set_stream_desc_eos(dvpp_stream_desc, eos)

#         ret = acl.media.dvpp_destroy_stream_desc(dvpp_stream_desc)
#         if ret != ACL_ERROR_NONE:
#             print("Destroy stream desc failed")
#             return FAILED

#         return SUCCESS


# def VencProcessEntry(msg_queue, width, height, status):
#     venc_handle = AclVencProcess(width.value, height.value)
#     ret = venc_handle.init_resource()   
#     if ret != SUCCESS:
#         print("Venc process failed") 
#         status.value = VENC_STATUS_ERROR
#         return

#     status.value = VENC_STATUS_WORK
#     ret = SUCCESS
#     while ret == SUCCESS:
#         msg = msg_queue.get()
#         if msg is None:
#             continue

#         if msg.name == "frame_data":
#             ret = venc_handle.process(msg.data)
#         elif msg.name == "exit":
#             ret = venc_handle.process(None, True)
#         else:
#             print("Unknow msg, stop encode")
#             ret = FAILED
#     if ret == SUCCESS:
#         status.value = VENC_STATUS_EXIT
#     else:
#         status.value = VENC_STATUS_ERROR

#     venc_handle.venc_stream_desc_set()
#     print("Venc process exit")


# def main():
#     acl.init()
#     acl.rt.set_device(0)

#     dvpp_venc = AclVenc(1280, 720)
#     ret = dvpp_venc.init_resource()
#     if ret != SUCCESS:
#         print("Init venc failed")
#         return

# if __name__ == '__main__':
#     main()