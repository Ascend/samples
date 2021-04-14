"""
@Date: 2020-12-09 23:28:01
@LastEditTime: 2020-12-18 01:26:11
@FilePath: /colorization_video_python/atlas_utils/presenteragent/presenter_datatype.py
"""

STATUS_DISCONNECT = 0
STATUS_CONNECTED = 1
STATUS_OPEN_CH_REQUEST = 2
STATUS_OPENED = 3
STATUS_EXITING = 4
STATUS_EXITTED = 5

CONTENT_TYPE_IMAGE = 0
CONTENT_TYPE_VIDEO = 1

STATUS_OK = 0
STATUS_ERROR = 1


class Point(object):
    """
    initialization
    """
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class Box(object):
    """
    function:box
    """
    def __init__(self, lt, rb):
        self.lt = Point(lt)
        self.rb = Point(rb)

    def box_valid(self):
        return ((self.lt.x >= 0)
                and (self.lt.y >= 0)
                and (self.rb.x >= self.lt.x)
                and (self.rb.y >= self.lt.y))


class ObjectDetectionResult(object):
    """
    initialization
    """
    def __init__(self, ltx=0, lty=0, rbx=0, rby=0, text=None):
        self.object_class = 0
        self.confidence = 0
        self.box = Box((ltx, lty), (rbx, rby))
        self.result_text = text

