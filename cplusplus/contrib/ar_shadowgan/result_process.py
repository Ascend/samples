"""图片后处理"""
import os
import numpy as np
import os.path as osp
from PIL import Image

result_process_path = os.path.dirname(os.path.abspath(__file__))

result_path = os.path.join(result_process_path, 'out/output/output1_0.bin')

save_path = os.path.join(result_process_path, 'out/output/result_0.jpg')

result_img = np.fromfile(result_path, dtype='float32')
result_img = result_img.reshape([1, 256, 256, 3])
result_img = ((result_img + 1.0) * 127.5).astype(np.uint8)
result_img = result_img[0, :, :, :]
result_img = result_img[:, :, ::-1]
output_img = Image.fromarray(result_img, mode='RGB')

output_img.save(save_path)

if not osp.exists(save_path):
	print("[ERROR] No file %s." % save_path)
	exit(1)
image = Image.open(save_path).convert('RGB')

if image is None:
    print("[ERROR] %s is not an image." % save_path)
    exit(1)
image.show()

