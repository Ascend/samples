import sys
import os

import acl


_ACL_DEBUG = 0
_ACL_INFO = 1
_ACL_WARNING = 2
_ACL_ERROR = 3


def log_error(*log_msg):
    """Recode error level log to file
    Args:
        *log_msg: format string and args list
    """
    log_str = [str(i) for i in log_msg]
    log_str = "".join(log_str)

    print(log_str)

    caller_frame = sys._getframe().f_back
    # caller file
    filename = caller_frame.f_code.co_filename
    # caller line no
    line_no = caller_frame.f_lineno
    # caller function
    func_name = caller_frame.f_code.co_name

    message = "[" + filename + ":" + str(line_no) + \
              " " + func_name + "]" + log_str
    acl.app_log(_ACL_ERROR, message)


def log_warning(*log_msg):
    """Recode warning level log to file
    Args:
        *log_msg: format string and args list
    """
    log_str = [str(i) for i in log_msg]
    log_str = "".join(log_str)
    caller_frame = sys._getframe().f_back
    # caller file
    filename = caller_frame.f_code.co_filename
    # caller line no
    line_no = caller_frame.f_lineno
    # caller function
    func_name = caller_frame.f_code.co_name

    message = "[" + filename + ":" + str(line_no) + \
              " " + func_name + "]" + log_str
    acl.app_log(_ACL_WARNING, message)


def log_info(*log_msg):
    """Recode info level log to file
    Args:
        *log_msg: format string and args list
    """
    log_str = [str(i) for i in log_msg]
    log_str = "".join(log_str)
    print(log_str)
    caller_frame = sys._getframe().f_back
    # caller file
    filename = caller_frame.f_code.co_filename
    # caller line no
    line_no = caller_frame.f_lineno
    # caller function
    func_name = caller_frame.f_code.co_name

    message = "[" + filename + ":" + str(line_no) + \
              " " + func_name + "]" + log_str
    acl.app_log(_ACL_INFO, message)


def log_debug(*log_msg):
    """Recode debug level log to file
    Args:
        *log_msg: format string and args list
    """
    log_str = [str(i) for i in log_msg]
    log_str = "".join(log_str)
    caller_frame = sys._getframe().f_back
    # caller file
    filename = caller_frame.f_code.co_filename
    # caller line no
    line_no = caller_frame.f_lineno
    # caller function
    func_name = caller_frame.f_code.co_name

    message = "[" + filename + ":" + str(line_no) + \
              " " + func_name + "]" + log_str

    acl.app_log(_ACL_DEBUG, message)
