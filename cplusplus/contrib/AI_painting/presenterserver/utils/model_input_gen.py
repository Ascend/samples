import numpy as np
import painting_message_pb2

vocab = {"sky": 1, "sand": 2, "sea": 3, "mountain": 4, "rock": 5, "earth": 6, "tree": 7, "water": 8,
         "land": 9, "grass": 10, "path": 11, "dirt": 12, "river": 13, "hill": 14, "filed": 15, "lake": 16}
MAX_OBJ_PER_IMG = 9

def gen_coarse_layout(objs, boxes, attributes, obj_valid_inds, layout_size=256, num_classes=17):
    height, width = layout_size, layout_size
    layout = np.zeros((1, height, width, num_classes), dtype=np.float)
    for objs_item, box, attributes_item, obj_valid_inds_item in zip(objs, boxes, attributes, obj_valid_inds):
        if obj_valid_inds_item == 0:
            break
        x_c, y_c = width * box[0], height * box[1]
        obj_size = attributes_item
        w, h = width * float(obj_size) / 10, height * float(obj_size) / 10
        x0, y0, x1, y1 = int(x_c - w / 2), int(y_c - h / 2), int(x_c + w / 2), int(y_c + h / 2)
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, width), min(y1, height)
        layout[:, y0:y1 - 1, x0:x1 - 1, int(objs_item)] = 1

    layout[:, :, :, 0] = 0
    return layout


# inputs descriptions:
# objects - [objs_name_1, objs_name_2, ...], len(objects)<=9
# boxes - [[x_center, y_center], ....], x_center/y_center in [0,1], len(boxes)<=9
# size_att - [objs_size_1, objs_size_2, ...], objs_size_1 in {0,1,2,3,4,5,6,7,8,9}, len(size_att)<=9

objects = ['sky', 'sand', 'sea', 'grass']
boxes = [[0.5, 0.1], [0.5, 0.9], [0.7, 0.8], [0.1, 0.5]]
size_att = [8, 4, 4, 4]

num_objs = len(objects)
objects = np.array([vocab[name] for name in objects])
to_pad = MAX_OBJ_PER_IMG - num_objs
objects = np.pad(objects, (0, to_pad), mode='constant').astype(np.int64)
obj_valid_inds = np.array([1] * num_objs + [0] * to_pad)
layout = np.array(gen_coarse_layout(objects, boxes, size_att, obj_valid_inds, layout_size=256, num_classes=17),
                  dtype=np.float32)

layout.tofile('layout.bin')
objects.tofile('objects.bin')

objects_bytes = objects.tobytes()
with open('objects.bin', 'rb') as f1:
    objects_bin_file_data = f1.read()
print('objects_bytes == objects_bin_file_data:', objects_bytes == objects_bin_file_data)

layout_bytes = layout.tobytes()
with open('layout.bin', 'rb') as f2:
    layout_bin_file_data = f2.read()
print('layout_bytes == layout_bin_file_data:', layout_bytes == layout_bin_file_data)


request = painting_message_pb2.DataPackage()
request.objectData.name = 'object_data'
request.objectData.data = objects_bytes
request.layoutData.name = 'layout_data'
request.objectData.data = layout_bytes

# response = painting_message_pb2.CommonResponse()
msg_name = painting_message_pb2._DATAPACKAGE.full_name

print('msg_name:', msg_name)
