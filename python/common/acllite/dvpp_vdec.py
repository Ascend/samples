import numpy as np
import acl
import queue

import constants as const
import acllite_utils as utils
import acllite_logger as acl_log
from acllite_image import AclLiteImage

READ_TIMEOUT = 5
WAIT_INTERVAL = 0.1

class DvppVdec(object):
    """Decode h264/h265 stream by dvpp vdec
    Decode one frame of h264/h265 stream.The stream must be h264 main, baseline
    or high level, annex-b format, or h265 main level.Output image is yuv420sp
    Attributes:
        _channel_id: dvpp vdec channel parameter, must global unique
        _width: input frame width
        _height:input frame height
        _run_flag:deocde is running or not currently, callback thread daemon condition
        _callbak_tid: decode callback thread id
        _channel_desc: vdec channel desc handle
        _ctx: current thread acl context
        _entype: video stream encode type, dvpp vdec support:
                const.ENTYPE_H265_MAIN = 0 H265 main level
                const.ENTYPE_H264_BASE = 1 H264 baseline level
                const.ENTYPE_H264_MAIN = 2 H264 main level
                const.ENTYPE_H264_HIGH = 3 H264 high level
        _format: output frame image format, use yuv420sp
        _decod_complete_cnt: output decoded complete frames counter
        _decode_cnt: input frames counter
        _output_pic_size: output image data size
        _frame_queue: output decoded frame image queue
    """

    def __init__(self, channel_id, width, height, entype, ctx,
                 output_format=const.PIXEL_FORMAT_YUV_SEMIPLANAR_420):
        """Create dvpp vdec instance
        Args:
            channel_id: decode channel id, must be global unique
            width: frame width
            height: frame height
            entype: video stream encode type
            ctx: current thread acl context
            output_format: output image format, support yuv420 nv12 and nv21
        """
        self._channel_id = channel_id
        self._width = width
        self._height = height
        self._run_flag = True
        self._callbak_tid = None
        self._channel_desc = None
        self._ctx = ctx
        self._entype = entype
        self._format = output_format
        self._decode_complete_cnt = 0
        self._decode_cnt = 0
        self._output_pic_size = (self._width * self._height * 3) // 2
        self._frame_queue = queue.Queue(64)
        self._frame_config = None
        self._destory_channel_flag = False

    def _callback_thread_entry(self, args_list):
        ret = acl.rt.set_context(self._ctx)
        while self._run_flag is True:
            ret = acl.rt.process_report(300)

    def _callback(self, input_stream_desc, output_pic_desc, user_data):
        self._decode_complete_cnt += 1
        #print("callback ", self._decode_complete_cnt)
        input_stream_data = acl.media.dvpp_get_stream_desc_data(
            input_stream_desc)
        input_stream_data_size = acl.media.dvpp_get_stream_desc_size(
            input_stream_desc)
        ret = acl.media.dvpp_destroy_stream_desc(input_stream_desc)

        self._get_pic_desc_data(output_pic_desc, user_data)

    def _get_pic_desc_data(self, pic_desc, user_data):
        pic_data = acl.media.dvpp_get_pic_desc_data(pic_desc)
        pic_data_size = acl.media.dvpp_get_pic_desc_size(pic_desc)
        ret_code = acl.media.dvpp_get_pic_desc_ret_code(pic_desc)
        if ret_code:
            channel_id, frame_id = user_data
            acl_log.log_error("Decode channel %d frame %d failed, error %d"
                              % (channel_id, frame_id, ret_code))
            acl.media.dvpp_free(pic_data)
        else:
            image = AclLiteImage(pic_data, self._width, self._height, 0, 0,
                             pic_data_size, const.MEMORY_DVPP)
            self._frame_queue.put(image)
        acl.media.dvpp_destroy_pic_desc(pic_desc)

    def init(self):
        """Init dvpp vdec
        Returns:
            const.SUCCESS: init success
            const.FAILED: init failed
        """
        # print("0")
        self._channel_desc = acl.media.vdec_create_channel_desc()
        self._callbak_tid, ret = acl.util.start_thread(
            self._callback_thread_entry, [])
        acl.media.vdec_set_channel_desc_channel_id(self._channel_desc,
                                                   self._channel_id)
        acl.media.vdec_set_channel_desc_thread_id(self._channel_desc,
                                                  self._callbak_tid)

        # print("1")
        acl.media.vdec_set_channel_desc_callback(self._channel_desc,
                                                 self._callback)

        acl.media.vdec_set_channel_desc_entype(self._channel_desc,
                                               self._entype)
        acl.media.vdec_set_channel_desc_out_pic_format(self._channel_desc,
                                                       self._format)
        # print("3")
        out_mode = acl.media.vdec_get_channel_desc_out_mode(self._channel_desc)
        if out_mode != 0:
            acl_log.log_error("Dvpp vdec out mode(%d) is invalid" % (out_mode))
            return const.FAILED

        acl.media.vdec_set_channel_desc_out_mode(self._channel_desc,
                                                 out_mode)
        acl.media.vdec_create_channel(self._channel_desc)

        self._frame_config = acl.media.vdec_create_frame_config()
        if self._frame_config is None:
            acl_log.log_error("Create dvpp frame config failed")
            return const.FAILED

        return const.SUCCESS

    def _thread_join(self):
        if self._callbak_tid is not None:
            self._run_flag = False
            ret = acl.util.stop_thread(self._callbak_tid)
            self._callbak_tid = None

    def process(self, input_data, input_size, user_data):
        """Decode frame
        Args:
            input_data: input frame data
            input_size: input frame data size

        Returns:
            const.SUCCESS: process success
            const.FAILED: process failed
        """
        input_stream_desc = self._create_input_pic_stream_desc(input_data,
                                                               input_size)
        if input_stream_desc is None:
            acl_log.log_error("Dvpp vdec decode frame failed for "
                              "create input stream desc error")
            return const.FAILED

        output_pic_desc = self._create_output_pic_desc()
        if output_pic_desc is None:
            acl_log.log_error("Dvpp vdec decode frame failed for create "
                              "output pic desc failed")
            return const.FAILED

        ret = acl.media.vdec_send_frame(self._channel_desc, input_stream_desc,
                                        output_pic_desc, self._frame_config,
                                        user_data)
        if ret:
            acl_log.log_error("Dvpp vdec send frame failed, error ", ret)
            return const.FAILED

        self._decode_cnt += 1
        #print("send frame ", self._decode_cnt)

        return const.SUCCESS

    def _create_input_pic_stream_desc(self, input_data, input_size):
        stream_desc = acl.media.dvpp_create_stream_desc()
        if stream_desc is None:
            acl_log.log_error("Create dvpp vdec input pic stream desc failed")
            return None

        acl.media.dvpp_set_stream_desc_size(stream_desc, input_size)
        acl.media.dvpp_set_stream_desc_data(stream_desc, input_data)

        return stream_desc

    def _create_output_pic_desc(self):
        output_buffer, ret = acl.media.dvpp_malloc(self._output_pic_size)
        if (output_buffer is None) or ret:
            acl_log.log_error(
                "Dvpp vdec malloc output memory failed, "
                "size %d, error %d" %
                (self._output_pic_size, ret))
            return None

        pic_desc = acl.media.dvpp_create_pic_desc()
        if pic_desc is None:
            acl_log.log_error("Create dvpp vdec output pic desc failed")
            return None

        acl.media.dvpp_set_pic_desc_data(pic_desc, output_buffer)
        acl.media.dvpp_set_pic_desc_size(pic_desc, self._output_pic_size)
        acl.media.dvpp_set_pic_desc_format(pic_desc, self._format)

        return pic_desc

    def destroy(self):
        """Release dvpp vdec resource"""
        #print("vdec destroy****************")
        if self._channel_desc is not None:
            ret = acl.media.vdec_destroy_channel(self._channel_desc)
            self._channel_desc = None

        self._thread_join()

        if self._frame_config is not None:
            acl.media.vdec_destroy_frame_config(self._frame_config)
            self._frame_config = None
        self._destory_channel_flag = True

    def is_finished(self):
        """Video decode finished"""
        return ((self._decode_cnt > 0) and
                (self._decode_complete_cnt >= self._decode_cnt))

    def read(self, no_wait=False):
        """Read decoded frame
        no_wait: Get image without wait. If set this arg True, and
            return image is None, should call is_finished() method
            to confirm decode finish or failed

        Returns:
            1. const.SUCCESS, not None: get image success
            2. const.SUCCESS, None: all frames decoded and be token off
            3. const.FAILED, None: Has frame not decoded, but no image decoded,
                                it means decode video failed
        """
        image = None
        ret = const.SUCCESS
        # received eos frame and all received frame decode complete
        if no_wait or self.is_finished():
            try:
                image = self._frame_queue.get_nowait()
            except queue.Empty:
                acl_log.log_info("No decode frame in queue anymore")
        else:
            try:
                image = self._frame_queue.get(timeout=READ_TIMEOUT)
            except queue.Empty:
                ret = const.FAILED
                acl_log.log_error("Read channel id %d frame timeout, "
                                  "receive frame %d, decoded %d"
                                  % (self._channel_id, self._decode_cnt,
                                     self._decode_complete_cnt))
        return ret, image

