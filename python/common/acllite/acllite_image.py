import os
import numpy as np
from PIL import Image

import acl
import acllite_utils as utils
import acllite_logger as acl_log
import constants as const

class AclLiteImage(object):
    """Image data and operation class
    Wrap image data and operation method, support jpeg, png, yuv file and
    memory data

    Attributes:
        _run_mode: device run mode
        _data: image binary data or numpy array
        _memory_type: the data in which memory, include dvpp,
                      device and np array
        width: image width
        height: image height
        align_width: align image width
        align_height: align image height
        _encode_format: image format
        _load_ok: load image success or not

    """
    _run_mode, _ = acl.rt.get_run_mode()

    def __init__(self, image, width=0, height=0, align_width=0, align_height=0,
                 size=0, memory_type=const.MEMORY_NORMAL):
        """Create AclLiteImage instance
        Args:
            image: image data, binary, numpy array or file path
            width: image width. if image is jpeg or png file,
                   this arg is not nesscessary
            height: image height. if image is jpeg or png file, this arg is
                    not nesscessary
            size: image data size. if image is file path, this arg is not
                  nesscessary
            memory_type: memory type of image data. if image is file path, this
                         arg is not nesscessary
        """
        self._data = None
        self._bytes_data = None
        self._memory_type = memory_type
        self.width = 0
        self.height = 0
        self.align_width = 0
        self.align_height = 0
        self.size = 0
        self._encode_format = const.ENCODE_FORMAT_UNKNOW
        self._load_ok = True

        if isinstance(image, str):
            self._instance_by_image_file(image, width, height)
        elif isinstance(image, int):
            self._instance_by_buffer(image, width, height, align_width, align_height, size)
        elif isinstance(image, np.ndarray):
            self._instance_by_nparray(image, width, height, align_width, align_height)
        else:
            acl_log.log_error("Create instance failed for "
                              "unknow image data type")

    def __del__(self):
        self.destroy()

    def is_loaded(self):
        """Image file load result
        When create image instance by file, call this method to check
        file load success or not

        Returns:
            True: load success
            False: load failed
        """
        return self._load_ok

    def byte_data_to_np_array(self):
        """Trans image data to np array"""
        if self._type == const.IMAGE_DATA_NUMPY:
            return self._data.copy()

        return utils.copy_data_as_numpy(self._data, self.size,
                                        self._memory_type, AclLiteImage._run_mode)

    def data(self):
        """Get image binary data"""
        if self._type == const.IMAGE_DATA_NUMPY:
            if "bytes_to_ptr" in dir(acl.util):
                self._bytes_data = self._data.tobytes()
                factor_ptr = acl.util.bytes_to_ptr(self._bytes_data)
            else:
                factor_ptr = acl.util.numpy_to_ptr(self._data)
            return factor_ptr
        else:
            return self._data

    def copy_to_dvpp(self):
        """Copy image data to dvpp"""
        device_ptr = utils.copy_data_to_dvpp(self.data(), self.size,
                                             self._run_mode)
        if device_ptr is None:
            acl_log.log_error("Copy image to dvpp failed")
            return None
        return AclLiteImage(device_ptr, self.width, self.height, 0, 0,
                        self.size, const.MEMORY_DVPP)

    def copy_to_host(self):
        """"Copy data to host"""
        if self._type == const.IMAGE_DATA_NUMPY:
            data_np = self._data.copy()
            return AclLiteImage(data_np, self.width, self.height, 0, 0)

        data = None
        mem_type = const.MEMORY_HOST
        if AclLiteImage._run_mode == const.ACL_HOST:
            if self.is_local():
                data = utils.copy_data_host_to_host(self._data, self.size)
            else:
                data = utils.copy_data_device_to_host(self._data, self.size)
        else:
            data = utils.copy_data_device_to_device(self._data, self.size)
            mem_type = const.MEMORY_DEVICE
        if data is None:
            acl_log.log_error("Copy image to host failed")
            return None

        return AclLiteImage(data, self.width, self.height, 0, 0, self.size, mem_type)

    def is_local(self):
        """Image data is in host server memory and access directly or not"""
        # in atlas200dk, all kind memory can access directly
        if AclLiteImage._run_mode == const.ACL_DEVICE:
            return True
        # in atlas300, only acl host memory or numpy array can access directly
        elif ((AclLiteImage._run_mode == const.ACL_HOST) and
              ((self._memory_type == const.MEMORY_HOST) or
               (self._memory_type == const.MEMORY_NORMAL))):
            return True
        else:
            return False

    def save(self, filename):
        """Save image as file"""
        image_np = self.byte_data_to_np_array()
        image_np.tofile(filename)

    def destroy(self):
        """Release image memory"""
        if (self._data is None) or (self.size == 0):
            acl_log.log_error("Release image abnormaly, data is None")
            return

        if self._memory_type == const.MEMORY_DEVICE:
            acl.rt.free(self._data)
        elif self._memory_type == const.MEMORY_HOST:
            acl.rt.free_host(self._data)
        elif self._memory_type == const.MEMORY_DVPP:
            acl.media.dvpp_free(self._data)
        # numpy no need release
        self._data = None
        self.size = 0

    def _instance_by_image_file(self, image_path, width, height):
        # Get image format by filename suffix
        self._encode_format = self._get_image_format_by_suffix(image_path)
        if self._encode_format == const.ENCODE_FORMAT_UNKNOW:
            acl_log.log_error("Load image %s failed" % (image_path))
            self._load_ok = False
            return

        # Read image data from file to memory
        self._data = np.fromfile(image_path, dtype=np.byte)
        self._type = const.IMAGE_DATA_NUMPY
        self.size = self._data.itemsize * self._data.size
        self._memory_type = const.MEMORY_NORMAL

        # Get image parameters of jpeg or png file by pillow
        if ((self._encode_format == const.ENCODE_FORMAT_JPEG) or
                (self._encode_format == const.ENCODE_FORMAT_PNG)):
            image = Image.open(image_path)
            self.width, self.height = image.size
        else:
            # pillow can not decode yuv, so need input widht and height args
            self.width = width
            self.height = height

    def _get_image_format_by_suffix(self, filename):
        suffix = os.path.splitext(filename)[-1].strip().lower()
        if (suffix == ".jpg") or (suffix == ".jpeg"):
            image_format = const.ENCODE_FORMAT_JPEG
        elif suffix == ".png":
            image_format = const.ENCODE_FORMAT_PNG
        elif suffix == ".yuv":
            image_format = const.ENCODE_FORMAT_YUV420_SP
        else:
            acl_log.log_error("Unsupport image format: ", suffix)
            image_format = const.ENCODE_FORMAT_UNKNOW

        return image_format

    def _instance_by_buffer(self, image_buffer, width, height, align_width, align_height, size):
        self.width = width
        self.height = height
        self.align_height = align_height
        self.align_width = align_width
        self.size = size
        self._data = image_buffer
        self._type = const.IMAGE_DATA_BUFFER

    def _instance_by_nparray(self, data, width, height, align_width, align_height):
        self.width = width
        self.height = height
        self.align_height = align_height
        self.align_width = align_width
        self.size = data.itemsize * data.size
        self._data = data
        self._type = const.IMAGE_DATA_NUMPY
        self._memory_type = const.MEMORY_NORMAL
