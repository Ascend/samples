"""
EDVR super resolution
"""
import sys
import os
import acl
import time
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from dataloader import build_test_dataloader
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource

import numpy as np
import imageio
import ffmpeg
import cv2
import shutil
import make_reds_dataset

FPS_MUL = 2
MODEL_PATH = "../model/EDVR_180_320.om"
MODEL_WIDTH = 320
MODEL_HEIGHT = 180

class VideoSuperResolution(object):
    """
    video super resolution
    """
    def __init__(self, model_path, model_width, model_height,
                 scale,
                 num_frames,
                 set_file,
                 batch_size,
                 input_dir,
                 input_name,
                 output_dir):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._model = AclLiteModel(model_path)

        self.scale = scale
        self.num_frames = num_frames
        self.set_file = set_file
        self.batch_size = batch_size
        self.input_dir = input_dir
        self.input_name = input_name
        self.output_dir = output_dir

    def __del__(self):
        print("[Sample] class Samle release source success")

    def _get_fps(self):
        probe = ffmpeg.probe(str(os.path.join(self.input_dir, self.input_name)))
        stream_data = next(
            (stream for stream in probe['streams'] if stream['codec_type'] == 'video'),
            None,
        )
        frame_rate = eval(stream_data['avg_frame_rate'])
        return frame_rate

    def extract_raw_frames(self):
        """
        extract frames from video
        """
        source_path = os.path.join(self.input_dir, self.input_name)

        target_path = os.path.join(self.input_dir, "images")
        if os.path.exists(target_path):
            shutil.rmtree(target_path)

        os.makedirs(target_path)
        print(source_path)
        vidcap = cv2.VideoCapture(source_path)
        success, image = vidcap.read()

        total_frames = vidcap.get(cv2.CAP_PROP_FRAME_COUNT)

        meta, meta4 = make_reds_dataset.get_videos_meta(self._model_width, self._model_height, total_frames)
        make_reds_dataset.split_sets(self.input_dir, meta, meta4)
        count = 0
        while success:
            image = cv2.resize(image, (self._model_width, self._model_height), interpolation = cv2.INTER_AREA)
            cv2.imwrite(os.path.join(target_path, "%.8d.png") % count, image)
            success, image = vidcap.read()
            count += 1

        return target_path

    def build_video(self, output_img_path, output_video_name):
        """
        build video from frames
        """
        output_video_name = os.path.join(self.output_dir, output_video_name)
        if os.path.exists(output_video_name):
            os.remove(output_video_name)

        fps = (self._get_fps()) * FPS_MUL
        print(fps)

        template = os.path.join(output_img_path, '%8d.png')        
        ffmpeg.input(str(template), format='image2', framerate=38).output(output_video_name,
                crf = 17, vcodec = 'libx264', pix_fmt = 'yuv420p').run(capture_stdout=True)

    def inference(self):
        """
        super resolution inference
        """
        dataloader = build_test_dataloader(
            batch_size = self.batch_size,
            scale = self.scale,
            set_file = self.set_file,
            num_frames = self.num_frames,
            data_config = self.input_dir
        )

        output_img_dir = os.path.join(self.output_dir, "images")
        if os.path.exists(output_img_dir):
            shutil.rmtree(output_img_dir)
        os.makedirs(output_img_dir)

        ave_time = 0
        max_frame = len(dataloader)
        print("max_frame ", max_frame)
        for i in range(max_frame):
            lr_names, lr = dataloader.get_next()

            sr = self._model.execute([lr])
            sr = np.asarray(sr).squeeze().astype(np.uint8)

            im_name = lr_names[0].split(os.path.sep)
            output_img_path = os.path.join(output_img_dir, *im_name[-1:])

            print('Save high resolution image : ', output_img_path)
            imageio.imwrite(output_img_path, sr)

        print('Save high resolution image complete!')
        return output_img_dir


def main():
    """
    video super resolution inference
    """

    acl_resource = AclLiteResource()
    acl_resource.init()
    input_video_name = 'low_resolution.mp4'
    input_dir = '../data'
    scale = 4
    num_frames = 5
    set_file = 'test.json'
    batch_size = 1
    output_dir = '../out'
    output_video_name = "high_resolution.mp4"

    classify = VideoSuperResolution(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT,
                 scale,
                 num_frames,
                 set_file,
                 batch_size,
                 input_dir,
                 input_video_name,
                 output_dir)
    classify.extract_raw_frames()

    output_img_path = classify.inference()

    classify.build_video(output_img_path, output_video_name)

if __name__ == '__main__':
    main()
