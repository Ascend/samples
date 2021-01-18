import sys
import os

import os
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from lib.atlasutil_so import libatlas


_ACL_DEBUG = 0
_ACL_INFO = 1
_ACL_WARNING = 2
_ACL_ERROR = 3


def log_error(*log_info):
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)
    
    print(log_str)

    caller_frame = sys._getframe().f_back 
    #调用者文件名
    filename = caller_frame.f_code.co_filename                          
    #调用者所在行号
    line_no   = caller_frame.f_lineno
    #调用者函数名                   
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_ERROR, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))

def log_warning(*log_info):
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)
    caller_frame = sys._getframe().f_back 
    #调用者文件名
    filename = caller_frame.f_code.co_filename                          
    #调用者所在行号
    line_no   = caller_frame.f_lineno
    #调用者函数名                   
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_WARNING, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))    

def log_info(*log_info):
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)
    print(log_str)
    caller_frame = sys._getframe().f_back 
    #调用者文件名
    filename = caller_frame.f_code.co_filename                          
    #调用者所在行号
    line_no   = caller_frame.f_lineno
    #调用者函数名                   
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_INFO, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))   

def log_debug(*log_info):
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)
    caller_frame = sys._getframe().f_back 
    #调用者文件名
    filename = caller_frame.f_code.co_filename                          
    #调用者所在行号
    line_no   = caller_frame.f_lineno
    #调用者函数名                   
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_DEBUG, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))