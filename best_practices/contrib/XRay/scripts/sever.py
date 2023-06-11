import os
import glob
import time
import argparse
import cv2
import shutil
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description='Semantic Segmentation Testing With Pytorch')
    parser.add_argument('--model', type=str, default='../checkpoints/best_model_910.om', help='The path of model.om')
    parser.add_argument('--ais_infer_dir', type=str, default='/home/user/tools/ais-bench_workload/tool/ais_infer/ais_infer.py', help='The path of ais_infer.py')
    parser.add_argument('--data_dir', type=str, default='../sever_data', help='test img dirs without_gt')

    args = parser.parse_args()
    return args

def normalize(data, mean, std):
    data = data.astype('float32')
    data = data/255
    for i in range(3):
        data[:, :, i] = (data[:, :, i] - mean[i]) / std[i]
    data = np.transpose(data, (2, 0, 1))
    return data

def preprocess(dataset_dir, save_path, img_names):
    patch_size = 512
    num = 0

    for idx, img_name in enumerate(img_names):
        
        img = cv2.imread(os.path.join(dataset_dir, img_name))
        img_name = img_name.split('.')[0]
        h, w, c = img.shape
        for i in range(0, h, patch_size):
            for j in range(0, w, patch_size):
                patch = img[i: i + patch_size, j: j + patch_size, :]
                inpu = cv2.cvtColor(patch, cv2.COLOR_BGR2RGB)
                inpu = normalize(inpu, [.5, .5, .5], [.5, .5, .5])[np.newaxis,:]
                inpu = inpu.astype("float32")
                inpu.tofile(os.path.join(save_path, img_name + f"_{str(num).zfill(3)}.bin"))
                num+=1

def postprocess(img_dir, bin_dir, output_dir):

    resLis = os.listdir(bin_dir)
    resLis.sort()
    patch_size = 512
    start = time.time()

    file_list = []
    for file in resLis:
        if file == "sumary.json":
            continue
        file_list.append(file.split('.')[0][:-2])
    file_list = list(set(file_list))

    for img_name in os.listdir(img_dir):
        img = cv2.imread(os.path.join(img_dir, img_name))
        img = img/4*3
        img_name = img_name.split('.')[0]
        h, w, c = img.shape
        img_bin_list = [i for i in file_list if i.startswith(img_name)]
        img_bin_list.sort()
        for i in range(0, h, patch_size):
            for j in range(0, w, patch_size):
                output_patch_dir = img_bin_list[(i//patch_size)*(h//patch_size)+(j//patch_size)]
                output_patch = np.fromfile(os.path.join(bin_dir, output_patch_dir+'_0.bin'), np.float32).reshape(1,4,512, 512)
                mask = np.zeros((512,512,3))
                output_patch = np.argmax(output_patch,1)[0]
                for c in range(1,4):
                    mask[:,:,c-1] = output_patch==c
                mask = mask.astype("uint8")*63
                img[i: i + patch_size, j: j + patch_size, :] = img[i: i + patch_size, j: j + patch_size, :]+mask

        img = img/img.max()*255
        cv2.imwrite(os.path.join(output_dir, img_name+'.jpg'), img)

    
    end = time.time()

    print("Infer time is {:.3f}s per img".format((end - start)/len(file_list)*36))


if __name__ == '__main__':
    args = parse_args()
    input_dir = os.path.join(args.data_dir, 'input')
    input_bin_dir = os.path.join(args.data_dir, 'input_cache')
    output_bin_dir = os.path.join(args.data_dir, 'output_cache')
    output_dir = os.path.join(args.data_dir, 'output')
    done_upload_flag = os.path.join(args.data_dir, 'done_upload')
    done_infer_flag = os.path.join(args.data_dir, 'done_infer')
    for d in [input_dir, input_bin_dir, output_bin_dir, output_dir]:
        if os.path.exists(d):
            shutil.rmtree(d)
        os.makedirs(d)
    STAND_BY = False
    while(True):
        input_list = os.listdir(input_dir)
        if os.path.exists(done_upload_flag):
            STAND_BY = False
            os.rmdir(done_upload_flag)
            time.sleep(1) #wait file upload compelete
            print(f'inferring {input_list[0]}...')
            preprocess(input_dir, input_bin_dir, input_list)
            res = os.system(f'source ~/env.sh; python3 {args.ais_infer_dir} --model {args.model} --input {input_bin_dir} --batchsize 1 --output {output_bin_dir}')
            
            #directory process
            for f in os.listdir(output_bin_dir):
                if f.endswith('json'):
                    os.remove(os.path.join(output_bin_dir, f))
                else:
                    n_output_bin_dir = os.path.join(output_bin_dir, f)
            postprocess(input_dir, n_output_bin_dir, output_dir)
            os.mkdir(done_infer_flag)
            
        
            for d in [input_bin_dir, output_bin_dir]:
                shutil.rmtree(d)
                os.makedirs(d)
            print("done")
        else:
            time.sleep(2)
            if not STAND_BY:
                print("stand by...")
            STAND_BY = True