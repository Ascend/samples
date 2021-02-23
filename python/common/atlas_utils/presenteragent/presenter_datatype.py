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
    point coordinate 
    """
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class Box(object):
    """
    object rectangle area 	
    """
    def __init__(self, lt, rb):
        self.lt = Point(lt)
        self.rb = Point(rb)

    def box_valid(self):
        """
        verify box coordinate is valid
        """
        return ((self.lt.x >= 0)
                and (self.lt.y >= 0)
                and (self.rb.x >= self.lt.x)
                and (self.rb.y >= self.lt.y))


class ObjectDetectionResult(object):
    """
    object detection information, include object position, confidence and label	
    """
    def __init__(self, ltx=0, lty=0, rbx=0, rby=0, text=None):
        self.object_class = 0
        self.confidence = 0
        self.box = Box((ltx, lty), (rbx, rby))
        self.result_text = text

    def check_box_vaild(self, width, height):
        """
        verify object position is valid		
        """
        return (self.box.box_valid() and 
               (self.box.rb.x <= width) and 
               (self.box.rb.y <= height))


class FinishMsg(object):
    """
    the message to notify presenter agent exit
    """
    def __init__(self, data):
        self.data = data
    