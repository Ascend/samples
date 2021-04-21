import av
import numpy as np
import acl
import time

import atlas_utils.constants as const
import atlas_utils.utils as utils
import atlas_utils.resource_list as resource_list
import atlas_utils.acl_logger as acl_log
import atlas_utils.dvpp_vdec as dvpp_vdec
from atlas_utils.acl_image import AclImage
import atlas_utils.chanel_id_generator as channel_id_gen

WAIT_INTERVAL = 0.01
WAIT_READY_MAX = 10
WAIT_FIRST_DECODED_FRAME = 0.02

DECODE_STATUS_INIT = 0
DECODE_STATUS_READY = 1
DECODE_STATUS_RUNNING = 2
DECODE_STATUS_PYAV_FINISH = 3
DECODE_STATUS_ERROR = 4
DECODE_STATUS_STOP = 5
DECODE_STATUS_EXIT = 6


class AclVideo(object):
    """Decode video by pyav and pyacl dvpp vdec
    This class only support decode annex-b h264 file or rtsp ip camera.
    You can use command:
    ffmpeg -i aaa.mp4 -codec copy -bsf: h264_mp4toannexb -f h264 aaa.h264
    to transform mp4 file to h264 stream file.
    If decode rtsp of ip camera or stream pull stream software, make sure
    the stream format is annex-b

    Attributes:
        _stream_name: video stream name
        _input_buffer: dvpp vdec decode input data buffer
        _ctx: decode thread acl context, use the same contxt with app
        _entype: video stream encode type, dvpp vdec support:
                const.ENTYPE_H265_MAIN = 0 H265 main level
                const.ENTYPE_H264_BASE = 1 H264 baseline level
                const.ENTYPE_H264_MAIN = 2 H264 main level
                const.ENTYPE_H264_HIGH = 3 H264 high level
                this attributes will read from video stream extradata
        _channel_id: dvpp vdec decode channel id parameter, global unique
        _vdec: pyacl dvpp vdec instance
        _is_opened: the video stream wether open or not
        _status: video decoder current status
        _run_mode: the device mode
    """

    def __init__(self, strame_name):
        self._stream_name = strame_name
        self._input_buffer = None
        self._vdec = None
        self._is_opened = False
        self._width = 0
        self._height = 0
        self._decode_thread_id = None
        self._ctx, ret = acl.rt.get_context()
        if ret:
            acl_log.log_error("Get acl context failed when "
                              "instance AclVideo, error ", ret)
        else:
            self._entype = const.ENTYPE_H264_MAIN
            self._channel_id = channel_id_gen.gen_unique_channel_id()
            self._status = DECODE_STATUS_INIT
            self._run_mode, ret = acl.rt.get_run_mode()
            if ret:
                acl_log.log_error("Get acl run mode failed when "
                                  "instance AclVideo, error ", ret)
            else:
                self._open()

    def __del__(self):
        self.destroy()

    def _open(self):
        # Get frame width, height, encode type by pyav
        if self._get_param():
            acl_log.log_error("Decode %s failed for get stream "
                              "parameters error" % (self._stream_name))
            return

        # Create decode thread and prepare to decode
        self._decode_thread_id, ret = acl.util.start_thread(
            self._decode_thread_entry, [])
        if ret:
            acl_log.log_error("Create %s decode thread failed, error %d"
                              % (self._stream_name, ret))
            return

        # Wait decode thread decode ready
        for i in range(0, WAIT_READY_MAX):
            if self._status == DECODE_STATUS_INIT:
                time.sleep(WAIT_INTERVAL)

        if self._status == DECODE_STATUS_READY:
            self._is_opened = True
            acl_log.log_info("Ready to decode %s..." % (self._stream_name))
        else:
            acl_log.log_error("Open %s failed for wait ready timeout"
                              % (self._stream_name))
        return

    def _get_param(self):
        container = av.open(self._stream_name)
        stream = [s for s in container.streams if s.type == 'video']
        if len(stream) == 0:
            # The stream is not video
            acl_log.log_error("%s has no video stream" % (self._stream_name))
            return const.FAILED

        ret, profile = self._get_profile(stream)
        if ret:
            acl_log.log_error("%s is not annex-b format, decode failed"
                              % (self._stream_name))
            return const.FAILED

        video_context = container.streams.video[0].codec_context
        codec_id_name = video_context.name
        ret, self._entype = self._get_entype(codec_id_name, profile)
        if ret:
            return const.FAILED

        self._width = video_context.width
        self._height = video_context.height

        acl_log.log_info(
            "Get %s infomation: width %d, height %d, profile %d, "
            "codec %s, entype %d" %
            (self._stream_name,
             self._width,
             self._height,
             profile,
             codec_id_name,
             self._entype))

        container.close()

        return const.SUCCESS

    def _get_profile(self, stream):
        # Annex-b format h264 extradata is start with 0x000001 or 0x00000001
        extradata = np.frombuffer(stream[0].codec_context.extradata, np.ubyte)
        if (extradata[0:3] == [0, 0, 1]).all():
            profile_id = extradata[4]
        elif (extradata[0:4] == [0, 0, 0, 1]).all():
            profile_id = extradata[5]
        else:
            acl_log.log_error("The stream %s is not annex-b h264, "
                              "can not decode it" % (self._stream_name))
            return const.FAILED, None

        return const.SUCCESS, profile_id

    def _get_entype(self, codec_id_name, profile):
        # Dvpp vdec support h264 baseline, main and high level
        profile_entype_tbl = {
            'h264': {const.FF_PROFILE_H264_BASELINE: const.ENTYPE_H264_BASE,
                     const.FF_PROFILE_H264_MAIN: const.ENTYPE_H265_MAIN,
                     const.FF_PROFILE_H264_HIGH: const.ENTYPE_H264_HIGH},
            'h265': {const.FF_PROFILE_HEVC_MAIN: const.ENTYPE_H265_MAIN}}
        entype = None
        ret = const.SUCCESS

        if codec_id_name in profile_entype_tbl.keys():
            entype_tbl = profile_entype_tbl[codec_id_name]
            if profile in entype_tbl.keys():
                entype = entype_tbl[profile]
            elif codec_id_name == 'h264':
                # if not support profile, try to decode as main
                entype = const.ENTYPE_H264_MAIN
                acl_log.log_error("Unsurpport h264 profile ", profile,
                                  ", decode as main level")
            else:
                entype = const.ENTYPE_H265_MAIN
                acl_log.log_error("Unsurpport h265 profile ", profile,
                                  ", decode as main level")
        else:
            # Not h264 or h265
            ret = const.FAILED
            acl_log.log_error("Unsupport codec type ", codec_id_name)

        return ret, entype

    def _pyav_vdec(self):
        frame = 0
        video = av.open(self._stream_name)
        stream = [s for s in video.streams if s.type == 'video']
        acl_log.log_info("Start decode %s frames" % (self._stream_name))
        for packet in video.demux([stream[0]]):
            # Get frame data from packet and copy to dvpp
            frame_data, data_size = self._prepare_frame_data(packet)
            if data_size == 0:
                # Last packet size is 0, no frame to decode anymore
                break

            if self._vdec.process(frame_data, data_size,
                                  [self._channel_id, frame]):
                acl_log.log_error("Dvpp vdec deocde frame %d failed, "
                                  "stop decode" % (frame))
                self._status = DECODE_STATUS_ERROR
                break
            frame += 1

            # The status chang to stop when app stop decode
            if self._status != DECODE_STATUS_RUNNING:
                acl_log.log_info("Decode status change to %d, stop decode"
                                 % (self._status))
                break

    def _prepare_frame_data(self, packet):
        in_frame_np = np.frombuffer(packet.to_bytes(), np.byte)
        size = in_frame_np.size
        if size == 0:
            # Last frame data is empty
            acl_log.log_info("Pyav decode finish")
            self._status = DECODE_STATUS_PYAV_FINISH
            return None, 0

        in_frame_ptr = acl.util.numpy_to_ptr(in_frame_np)
        policy = const.ACL_MEMCPY_DEVICE_TO_DEVICE
        if self._run_mode == const.ACL_HOST:
            policy = const.ACL_MEMCPY_HOST_TO_DEVICE
        ret = acl.rt.memcpy(self._input_buffer, size, in_frame_ptr, size,
                            policy)
        if ret:
            acl_log.log_error("Copy data to dvpp failed, policy %d, error %d"
                              % (policy, ret))
            self._status = DECODE_STATUS_ERROR
            return None, 0

        return self._input_buffer, size

    def _decode_thread_entry(self, arg_list):
        # Set acl context for decode thread
        if self._decode_thread_init():
            acl_log.log_error("Decode thread init failed")
            return const.FAILED

        self._status = DECODE_STATUS_READY
        while (self._status == DECODE_STATUS_READY):
            time.sleep(WAIT_INTERVAL)

        self._pyav_vdec()
        self._decode_thread_join()

        return const.SUCCESS

    def _decode_thread_init(self):
        # Set acl context for decode thread
        ret = acl.rt.set_context(self._ctx)
        if ret:
            acl_log.log_error("%s decode thread init dvpp vdec failed")
            return const.FAILED

        # Instance dvpp vdec and init it
        self._vdec = dvpp_vdec.DvppVdec(self._channel_id, self._width,
                                        self._height, self._entype, self._ctx)
        if self._vdec.init():
            acl_log.log_error("%s decode thread init dvpp vdec failed"
                              % (self._stream_name))
            return const.FAILED

        # Malloc dvpp vdec decode input dvpp memory
        self._input_buffer, ret = acl.media.dvpp_malloc(
            utils.rgbu8_size(self._width, self._height))
        if ret:
            acl_log.log_error("%s decode thread malloc input memory failed, "
                              "error %d. frame width %d, height %d, size %d"
                              % (self._stream_name, ret,
                                 self._width, self._height,
                                 utils.rgbu8_size(self._width, self._height)))
            return const.FAILED

        return const.SUCCESS

    def _decode_thread_join(self):
        self.destroy()
        # Wait all decoded frame token off by read()
        while self._status < DECODE_STATUS_STOP:
            time.sleep(WAIT_INTERVAL)
        self._status = DECODE_STATUS_EXIT

    def is_finished(self):
        """Decode finished
        Pyav and dvpp vdec decoded all frame, and all deocde frames were
        token off. When read() return success but image is none, use this to
        confirm decode finished
        """
        return self._status == DECODE_STATUS_EXIT

    def read(self, no_wait=False):
        """Read decoded frame
        Args:
            no_wait: Get image without wait. If set this arg True, and
                     return image is None, should call is_finished() method
                     to confirm decode finish or failed

        Returns:
            1. const.SUCCESS, not None: get image success
            2. const.SUCCESS, None: all frames decoded and be token off
            3. const.FAILED, None: Has frame not decoded, but no image decoded,
                                it means decode video failed
        """
        # Pyav and dvpp vdec decoded all frame,
        # and all deocde frames were token off
        if self._status == DECODE_STATUS_EXIT:
            return const.SUCCESS, None

        # When call read first time, the decode thread only ready to decode,
        # but not decoding already. Set status to DECODE_STATUS_RUNNING will
        # cause pyav and dvpp vdec start decode actually
        if self._status == DECODE_STATUS_READY:
            self._status = DECODE_STATUS_RUNNING
            # The decode just begin, need wait the first frame to be decoded
            time.sleep(WAIT_FIRST_DECODED_FRAME)

        ret, image = self._vdec.read(no_wait)

        # Decode finish or stopped, and all decode frames were token off
        if (image is None) and (self._status > DECODE_STATUS_RUNNING):
            self._status = DECODE_STATUS_EXIT

        return ret, image

    def destroy(self):
        """Release all decode resource"""
        if self._vdec is not None:
            self._vdec.destroy()
        if self._input_buffer is not None:
            acl.media.dvpp_free(self._input_buffer)
            self._input_buffer = None

