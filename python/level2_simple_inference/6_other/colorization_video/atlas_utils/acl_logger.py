"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
@Date: 2020-12-09 23:28:01
@LastEditTime: 2020-12-17 18:52:02
@FilePath: /colorization_video_python/atlas_utils/acl_logger.py
"""

import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from lib.atlasutil_so import libatlas


_ACL_DEBUG = 0
_ACL_INFO = 1
_ACL_WARNING = 2
_ACL_ERROR = 3


def log_error(log_info):
    """print error message
    
    """
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)
    caller_frame = sys._getframe().f_back 
    #Caller file name
    filename = caller_frame.f_code.co_filename                          
    #Caller line number
    line_no   = caller_frame.f_lineno
    #Caller function name
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_ERROR, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))


def log_warning(log_info):
    """
    print warning message
    """
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)
    caller_frame = sys._getframe().f_back
    #Caller file name
    filename = caller_frame.f_code.co_filename
    #Caller line number
    line_no   = caller_frame.f_lineno
    #Caller function name
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_WARNING, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))    

                    
def log_info(*log_info):
    """
    print info message
    """
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)

    caller_frame = sys._getframe().f_back
    #Caller file name
    filename = caller_frame.f_code.co_filename
    #Caller line number
    line_no   = caller_frame.f_lineno
    #Caller function name
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_INFO, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))   


def log_debug(log_info):
    """
    print log_debug  
    """
    log_str = [str(i) for i in log_info]
    log_str = "".join(log_str)
    caller_frame = sys._getframe().f_back
    #Caller file name
    filename = caller_frame.f_code.co_filename
    #Caller line number
    line_no   = caller_frame.f_lineno
    #Caller function name
    func_name = caller_frame.f_code.co_name 

    libatlas.AclLog(_ACL_DEBUG, func_name.encode("utf-8"), 
                    filename.encode("utf-8"), line_no, 
                    log_str.encode("utf-8"))
