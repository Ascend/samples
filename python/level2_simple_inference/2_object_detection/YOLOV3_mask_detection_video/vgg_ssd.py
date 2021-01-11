import numpy as np

from acl_dvpp import Dvpp
from atlas_utils.presenteragent import presenter_datatype

LABEL = 1
SCORE = 2
TOP_LEFT_X = 3
TOP_LEFT_Y = 4
BOTTOM_RIGHT_X = 5
BOTTOM_RIGHT_Y = 6
MODEL_WIDTH = 640
MODEL_HEIGHT = 352

labels = ["face","person", "mask"]

INPUT_DIR = './data/'
OUTPUT_DIR = './out/'
MODEL_PATH = "./model/mask_detection.om"
MODEL_WIDTH = 640
MODEL_HEIGHT = 352
class_num = 3
stride_list = [8, 16, 32]
anchors_1 = np.array([[10, 13], [16, 30], [33, 23]]) / stride_list[0]
anchors_2 = np.array([[30, 61], [62, 45], [59, 119]]) / stride_list[1]
anchors_3 = np.array([[116, 90], [156, 198], [163, 326]]) / stride_list[2]
anchor_list = [anchors_1, anchors_2, anchors_3]

conf_threshold = 0.2
iou_threshold = 0.3

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 0, 255), (255, 255, 0)]


class VggSsd(object):
    def __init__(self, acl_resource, model_width, model_height):
        self._acl_resource = acl_resource
        self._model_width = model_width
        self._model_height = model_height
        #使用dvpp处理图像,当使用opencv或者PIL时则不需要创建dvpp实例
        self._dvpp = Dvpp(acl_resource)

    def __del__(self):
        print("Release yolov3 resource finished")


    def pre_process(self, image):
        #使用dvpp将图像缩放到模型要求大小
        resized_image = self._dvpp.resize(image, self._model_width,
                                          self._model_height)
        if resized_image == None:
            print("Resize image failed")
            return None
        #输出缩放后的图像和图像信息作为推理输入数据
        return [resized_image,]

        # img_h = image.size[1]
        # img_w = image.size[0]
        # net_h = MODEL_HEIGHT
        # net_w = MODEL_WIDTH

        # scale = min(float(net_w) / float(img_w), float(net_h) / float(img_h))
        # new_w = int(img_w * scale)
        # new_h = int(img_h * scale)

        # shift_x = (net_w - new_w) // 2
        # shift_y = (net_h - new_h) // 2
        # shift_x_ratio = (net_w - new_w) / 2.0 / net_w
        # shift_y_ratio = (net_h - new_h) / 2.0 / net_h

        # image_ = image.resize( (new_w, new_h))
        # new_image = np.zeros((net_h, net_w, 3), np.uint8)
        # new_image[shift_y: new_h + shift_y, shift_x: new_w + shift_x, :] = np.array(image_)
        # new_image = new_image.astype(np.float32)
        # new_image = new_image / 255

        # return new_image

    def post_process(self, infer_output, origin_img):
        #解析推理输出数据
        detection_result_list = self._analyze_inference_output(infer_output, 
                                                               origin_img)
        #将yuv图像转换为jpeg图像
        jpeg_image = self._dvpp.jpege(origin_img)



        return jpeg_image, detection_result_list

    def overlap(self, x1, x2, x3, x4):
        left = max(x1, x3)
        right = min(x2, x4)
        return right - left
    
    def cal_iou(self, box, truth):
        w = self.overlap(box[0], box[2], truth[0], truth[2])
        h = self.overlap(box[1], box[3], truth[1], truth[3])
        if w <= 0 or h <= 0:
            return 0
        inter_area = w * h
        union_area = (box[2] - box[0]) * (box[3] - box[1]) + (truth[2] - truth[0]) * (truth[3] - truth[1]) - inter_area
        return inter_area * 1.0 / union_area
    
    def apply_nms(self, all_boxes, thres):
        res = []
    
        for cls in range(class_num):
            cls_bboxes = all_boxes[cls]
            sorted_boxes = sorted(cls_bboxes, key=lambda d: d[5])[::-1]
    
            p = dict()
            for i in range(len(sorted_boxes)):
                if i in p:
                    continue
    
                truth = sorted_boxes[i]
                for j in range(i + 1, len(sorted_boxes)):
                    if j in p:
                        continue
                    box = sorted_boxes[j]
                    iou = self.cal_iou(box, truth)
                    if iou >= thres:
                        p[j] = 1
    
            for i in range(len(sorted_boxes)):
                if i not in p:
                    res.append(sorted_boxes[i])
        return res
    
    def decode_bbox(self, conv_output, anchors, img_w, img_h, x_scale, y_scale, shift_x_ratio, shift_y_ratio):
        def _sigmoid(x):
            s = 1 / (1 + np.exp(-x))
            return s
    
        h, w, _ = conv_output.shape
        
        pred = conv_output.reshape((h * w, 3, 5 + class_num))
    
        pred[..., 4:] = _sigmoid(pred[..., 4:])
        pred[..., 0] = (_sigmoid(pred[..., 0]) + np.tile(range(w), (3, h)).transpose((1, 0))) / w
        pred[..., 1] = (_sigmoid(pred[..., 1]) + np.tile(np.repeat(range(h), w), (3, 1)).transpose((1, 0))) / h
        pred[..., 2] = np.exp(pred[..., 2]) * anchors[:, 0:1].transpose((1, 0)) / w
        pred[..., 3] = np.exp(pred[..., 3]) * anchors[:, 1:2].transpose((1, 0)) / h
    
        bbox = np.zeros((h * w, 3, 4))
        bbox[..., 0] = np.maximum((pred[..., 0] - pred[..., 2] / 2.0 - shift_x_ratio) * x_scale * img_w, 0)  # x_min
        bbox[..., 1] = np.maximum((pred[..., 1] - pred[..., 3] / 2.0 - shift_y_ratio) * y_scale * img_h, 0)  # y_min
        bbox[..., 2] = np.minimum((pred[..., 0] + pred[..., 2] / 2.0 - shift_x_ratio) * x_scale * img_w, img_w)  # x_max
        bbox[..., 3] = np.minimum((pred[..., 1] + pred[..., 3] / 2.0 - shift_y_ratio) * y_scale * img_h, img_h)  # y_max
    
        pred[..., :4] = bbox
        pred = pred.reshape((-1, 5 + class_num))
        pred[:, 4] = pred[:, 4] * pred[:, 5:].max(1)
        pred = pred[pred[:, 4] >= conf_threshold]
        pred[:, 5] = np.argmax(pred[:, 5:], axis=-1)
    
        all_boxes = [[] for ix in range(class_num)]
        for ix in range(pred.shape[0]):
            box = [int(pred[ix, iy]) for iy in range(4)]
            box.append(int(pred[ix, 5]))
            box.append(pred[ix, 4])
            all_boxes[box[4] - 1].append(box)
        
        return all_boxes
    
    def convert_labels(self, label_list):
        if isinstance(label_list, np.ndarray):
            label_list = label_list.tolist()
            label_names = [labels[int(index)] for index in label_list]
        return label_names
    

    def _analyze_inference_output(self, infer_output, origin_img):

        result_return = dict()
        #img_h = origin_img.size[1]
        #img_w = origin_img.size[0]
        img_h = origin_img.height
        img_w = origin_img.width
        scale = min(float(MODEL_WIDTH) / float(img_w), float(MODEL_HEIGHT) / float(img_h))
        new_w = int(img_w * scale)
        new_h = int(img_h * scale)
        shift_x_ratio = (MODEL_WIDTH - new_w) / 2.0 / MODEL_WIDTH
        shift_y_ratio = (MODEL_HEIGHT- new_h) / 2.0 / MODEL_HEIGHT
        class_num = len(labels)
        num_channel = 3 * (class_num + 5)
        x_scale = MODEL_WIDTH / float(new_w)
        y_scale = MODEL_HEIGHT / float(new_h)
        all_boxes = [[] for ix in range(class_num)]
        for ix in range(3):
            pred = infer_output[2 - ix].reshape((MODEL_HEIGHT // stride_list[ix], MODEL_WIDTH // stride_list[ix], num_channel))
            anchors = anchor_list[ix]
            boxes = self.decode_bbox(pred, anchors, img_w, img_h, x_scale, y_scale, shift_x_ratio, shift_y_ratio)
            all_boxes = [all_boxes[iy] + boxes[iy] for iy in range(class_num)]

        res = self.apply_nms(all_boxes, iou_threshold)
        if not res:
            result_return['detection_classes'] = []
            result_return['detection_boxes'] = []
            result_return['detection_scores'] = []
            # return result_return
        else:
            new_res = np.array(res)
            picked_boxes = new_res[:, 0:4]
            picked_boxes = picked_boxes[:, [1, 0, 3, 2]]
            picked_classes = self.convert_labels(new_res[:, 4])
            picked_score = new_res[:, 5]
            result_return['detection_classes'] = picked_classes
            result_return['detection_boxes'] = picked_boxes.tolist()
            result_return['detection_scores'] = picked_score.tolist()
            # return result_return

        detection_result_list = []
        for i in range(len(result_return['detection_classes'])):
            box = result_return['detection_boxes'][i]
            class_name = result_return['detection_classes'][i]
            confidence = result_return['detection_scores'][i]
            detection_item = presenter_datatype.ObjectDetectionResult()            
            detection_item.confidence = confidence
            detection_item.box.lt.x = int(box[1])
            detection_item.box.lt.y = int(box[0])
            detection_item.box.rb.x = int(box[3])
            detection_item.box.rb.y = int(box[2])
            detection_item.result_text = str(class_name)
            detection_result_list.append(detection_item)
        return detection_result_list






    #     #vgg ssd有两个输出,第一个输出infer_output[0]为检测到的物体个数,shape为(1,8)
    #     box_num = int(infer_output[0][0, 0])
    #    #第二个输出infer_output[1]为检测到的物体信息,shape为(1, 200, 8)
    #     box_info = infer_output[1][0]  
    #     detection_result_list = []
    #     for i in range(box_num):
    #         #检测到的物体置信度
    #         score = box_info[i, SCORE]
    #         if score < 0.9:
    #             break 
            
    #         detection_item = presenter_datatype.ObjectDetectionResult()            
    #         detection_item.confidence = score
    #         #人脸位置框坐标, 是归一化的坐标，需要乘以图片宽高转换为图片上的坐标
    #         detection_item.box.lt.x = int(box_info[i, TOP_LEFT_X] * origin_img.width)
    #         detection_item.box.lt.y = int(box_info[i, TOP_LEFT_Y] * origin_img.height)
    #         detection_item.box.rb.x = int(box_info[i, BOTTOM_RIGHT_X] * origin_img.width)
    #         detection_item.box.rb.y = int(box_info[i, BOTTOM_RIGHT_Y] * origin_img.height)
    #         #将置信度组织为字符串
    #         detection_item.result_text = str(round(detection_item.confidence * 100, 2)) + "%"
    #         detection_result_list.append(detection_item)
            
    #     return detection_result_list
