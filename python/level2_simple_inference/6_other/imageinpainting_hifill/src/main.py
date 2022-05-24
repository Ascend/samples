import sys
import os
import numpy as np
import acl
import cv2
import glob


path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from acllite_resource import AclLiteResource
from acllite_model import AclLiteModel
from acllite_utils import display_time, check_ret 
from constants import ACL_FLOAT, ACL_FORMAT_NCHW, ACL_MEM_MALLOC_NORMAL_ONLY, \
    ACL_MEM_MALLOC_HUGE_FIRST, ACL_MEMCPY_HOST_TO_DEVICE, ACL_MEMCPY_DEVICE_TO_HOST 

currentPath = os.path.join(path, "..")
OUTPUT_DIR = os.path.join(currentPath, 'out/')
MODEL_PATH = os.path.join(currentPath, "model/hifill.om")
OP_TYPE = "BatchMatMul"
MODEL_MATMUL_PATH =os.path.join(currentPath, "model")
print("MODEL_MATMUL_PATH", MODEL_MATMUL_PATH)
INPUT_SIZE = 512  
ATTENTION_SIZE = 32 
MULTIPLE = 6
NPTYPE_FLOAT32 = np.float32

def sort(str_lst):
    return [s for s in sorted(str_lst)]

def reconstruct_residual_from_patches(residual, multiple):
    """
    reconstruct residual from patches
    """
    residual = np.reshape(residual, [ATTENTION_SIZE, ATTENTION_SIZE, multiple, multiple, 3])
    residual = np.transpose(residual, [0, 2, 1, 3, 4])
    return np.reshape(residual, [ATTENTION_SIZE * multiple, ATTENTION_SIZE * multiple, 3])

def extract_image_patches(img, multiple):
    """
    extract image patch
    """
    h, w, c = img.shape
    img = np.reshape(img, [h // multiple, multiple, w // multiple, multiple, c])
    img = np.transpose(img, [0, 2, 1, 3, 4])
    return img

@display_time
def pre_process(raw_img, raw_mask): 
    """
    process raw image & mask
    """
    raw_mask = raw_mask.astype(NPTYPE_FLOAT32) / 255.
    raw_img = raw_img.astype(NPTYPE_FLOAT32)

    # resize raw image & mask to desinated size
    large_img = cv2.resize(raw_img,  (MULTIPLE * INPUT_SIZE, MULTIPLE * INPUT_SIZE), interpolation = cv2. INTER_LINEAR)
    large_mask = cv2.resize(raw_mask, (MULTIPLE * INPUT_SIZE, MULTIPLE * INPUT_SIZE), interpolation = cv2.INTER_NEAREST)

    small_img = cv2.resize(raw_img, (INPUT_SIZE, INPUT_SIZE), interpolation = cv2.INTER_NEAREST)
    small_mask = cv2.resize(raw_mask, (INPUT_SIZE, INPUT_SIZE), interpolation = cv2.INTER_NEAREST)
    
    # set hole region to 1. and backgroun to 0.
    small_mask = 1. - small_mask

    mask_512_hwc = small_mask[:,:,0:1]
    mask_512_chw = mask_512_hwc.transpose(2,0,1).copy()
    return (large_img, large_mask, small_img, mask_512_chw)

"""
get img list
"""
def get_imgs_masks_file_list(images, masks):
    paths_img = glob.glob(images + '/*.*[gG]')
    paths_mask = glob.glob(masks + '/*.*[gG]')
    paths_img = sort(paths_img)
    paths_mask = sort(paths_mask)
    return paths_img, paths_mask

"""
create input databuffer
"""
def create_input(np_data, size):
    if "bytes_to_ptr" in dir(acl.util):
        data_out = np_data
        bytes_data = np_data.tobytes()
        ptr = acl.util.bytes_to_ptr(bytes_data)
    else:
        ptr, data_out = acl.util.numpy_contiguous_to_ptr(np_data)
    dev_ptr, ret = acl.rt.malloc(size, ACL_MEM_MALLOC_HUGE_FIRST)
    ret = acl.rt.memcpy(dev_ptr,
                size,
                ptr,
                size,
                ACL_MEMCPY_HOST_TO_DEVICE)
    check_ret("acl.rt.memcpy", ret)

    return acl.create_data_buffer(dev_ptr, size)


def get_forward_result(dev_ptr, size):
    host_buffer, ret = acl.rt.malloc_host(size)
    check_ret("acl.rt.malloc_host", ret)
    ret = acl.rt.memcpy(host_buffer,
                        size,
                        dev_ptr,
                        size,
                        ACL_MEMCPY_DEVICE_TO_HOST)
    check_ret("acl.rt.memcpy", ret)

    if "ptr_to_bytes" in dir(acl.util):
        bytes_data = acl.util.ptr_to_bytes(host_buffer, size)
        data = np.frombuffer(bytes_data, dtype=np.float32).reshape(1024,27648)
    else:
        data = acl.util.ptr_to_numpy(host_buffer, (1024,27648), 11)
    return data
  

def forward_op_batch_matmul(data, stream):
    ret = acl.op.set_model_dir(MODEL_MATMUL_PATH)
    check_ret("acl.op.set_model_dir", ret)
    
    op_attr = acl.op.create_attr()
    ret = acl.op.set_attr_bool(op_attr, "adj_x1", False)
    check_ret("acl.op.set_attr_bool", ret)
    ret = acl.op.set_attr_bool(op_attr, "adj_x2", False)
    check_ret("acl.op.set_attr_bool", ret)

    input_desc_batch_matmul_x1 = \
        acl.create_tensor_desc(ACL_FLOAT,
                                [1, 1, 1024, 1024],
                                ACL_FORMAT_NCHW)
    input_desc_batch_matmul_x2 = \
        acl.create_tensor_desc(ACL_FLOAT,
                                [1, 1, 1024, 27648],
                                ACL_FORMAT_NCHW)
    output_desc_batch_matmul_y = \
        acl.create_tensor_desc(ACL_FLOAT,
                                [1, 1, 1024, 27648],
                                ACL_FORMAT_NCHW)
    tensor_size_batch_matmul_x1 = \
        acl.get_tensor_desc_size(input_desc_batch_matmul_x1)
    tensor_size_batch_matmul_x2 = \
        acl.get_tensor_desc_size(input_desc_batch_matmul_x2)
    tensor_size_batch_matmul_y = \
        acl.get_tensor_desc_size(output_desc_batch_matmul_y)
        
    input_buffer_x1 = create_input(data[0], tensor_size_batch_matmul_x1)
    input_buffer_x2 = create_input(data[1], tensor_size_batch_matmul_x2)
    
    dev_buffer_batch_matmul, ret = \
        acl.rt.malloc(tensor_size_batch_matmul_y,
                      ACL_MEM_MALLOC_NORMAL_ONLY)
    check_ret("acl.rt.malloc", ret)
    

    output_buffer_batch_matmul_y = \
        acl.create_data_buffer(dev_buffer_batch_matmul,
                                tensor_size_batch_matmul_y)    
   
    ret = acl.op.execute_v2(
        OP_TYPE,
        [input_desc_batch_matmul_x1, input_desc_batch_matmul_x2],
        [input_buffer_x1, input_buffer_x2],
        [output_desc_batch_matmul_y],
        [output_buffer_batch_matmul_y],
        op_attr,
        stream)
    check_ret("acl.op.execute_v2", ret)
    ret = acl.rt.synchronize_stream(stream)
    check_ret("acl.rt.synchronize_stream", ret)
    print("[SingleOp] batch_matmul run success")
    return get_forward_result(dev_buffer_batch_matmul, tensor_size_batch_matmul_y)
   
 
@display_time
def matmul_om_large(attention, residual, stream):
    """
    matul om large
    """
    attention_reshape = attention.reshape(1024, 1024)
    residual_reshape = residual.reshape(1024, 96 * 96 * 3)
    matmul_ret = forward_op_batch_matmul([attention_reshape, residual_reshape], stream)
    return matmul_ret.reshape(ATTENTION_SIZE, ATTENTION_SIZE, 3072 * 9)


def residual_aggregate(residual, attention, stream):
    """
    MULTIPLE * INPUT_SIZE//ATTENTION_SIZE = 6*512/32 = 96
    """
    residual = extract_image_patches(residual, MULTIPLE * INPUT_SIZE // ATTENTION_SIZE)
    residual = np.reshape(residual, [1, residual.shape[0] * residual.shape[1], -1])
    residual = matmul_om_large(attention, residual, stream)
    residual = reconstruct_residual_from_patches(residual, MULTIPLE * INPUT_SIZE // ATTENTION_SIZE)
    return residual

    
@display_time
def post_process(raw_img, large_img, large_mask, inpainted_512, img_512, mask_512, attention, stream):
    """
    compute the raw residual map
    s = time.time()
    """
    h, w, c = raw_img.shape
    low_base = cv2.resize(inpainted_512.astype(NPTYPE_FLOAT32), 
    (INPUT_SIZE * MULTIPLE, INPUT_SIZE * MULTIPLE), interpolation = cv2.INTER_LINEAR) 
    low_large = cv2.resize(img_512.astype(NPTYPE_FLOAT32), 
    (INPUT_SIZE * MULTIPLE, INPUT_SIZE * MULTIPLE), interpolation = cv2.INTER_LINEAR)
    residual = (large_img - low_large) * large_mask

    # reconstruct residual map using residual aggregation module
    residual = residual_aggregate(residual, attention, stream)

    # compute large inpainted result
    res_large = low_base + residual
    res_large = np.clip(res_large, 0., 255.)

    # resize large inpainted result to raw size
    res_raw = cv2.resize(res_large, (w, h), interpolation = cv2.INTER_LINEAR)
    
    # paste the hole region to the original raw image
    mask = cv2.resize(mask_512.astype(NPTYPE_FLOAT32), (w, h), interpolation = cv2.INTER_LINEAR)
    mask = np.expand_dims(mask, axis=2)
    
    res_raw = res_raw * mask + raw_img * (1. - mask)
    return res_raw.astype(np.uint8)

@display_time
def readimages(img_path, mask_path):    
    """
    readimages
    """
    raw_img = cv2.imread(img_path) 
    raw_mask = cv2.imread(mask_path) 
    return raw_img, raw_mask

@display_time
def inference(model, input_data):
    resultList = model.execute(input_data)
    inpainted_512 = resultList[0]    
    attention = resultList[1]
    mask_512_new = resultList[2] 

    return inpainted_512, attention, mask_512_new

@display_time
def main(image_dirs, masks_dirs):    
    
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    #acl  init
    acl_resource = AclLiteResource()
    acl_resource.init()
    stream, ret = acl.rt.create_stream()
    check_ret("acl.rt.create_stream", ret)
    #load model
    model = AclLiteModel(MODEL_PATH)
    
    paths_img, paths_mask = get_imgs_masks_file_list(image_dirs, masks_dirs)
    for i in range(len(paths_img)):
        print('==========')        
        raw_img, raw_mask = readimages(paths_img[i], paths_mask[i])
        print("file: % s, shape= % s" % (paths_img[i], raw_img.shape))

        (img_large, mask_large, img_512, mask_512) = pre_process(raw_img, raw_mask)        
        inpainted_512, attention, mask_512_new  = inference(model,[img_512, mask_512,])                

        # post-processing
        res_raw_size = post_process(raw_img, img_large, \
            mask_large, inpainted_512[0], img_512, mask_512_new[0], attention[0], stream)
        filename = os.path.join(OUTPUT_DIR, 'outpaint_' + os.path.basename(paths_img[i]))
        cv2.imwrite(filename, res_raw_size)
        
    print("Execute end")

if __name__ == '__main__':

    image_dir = os.path.join(currentPath, "data" )
    masks_dir = os.path.join(currentPath, "mask")   
    main(image_dir, masks_dir)
 
