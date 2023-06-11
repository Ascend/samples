import timm
import torchvision.transforms as T
import torch.optim as optim
import importlib
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import copy
import yaml
import cv2
import ctools
import gtools
from easydict import EasyDict as edict
import torch.backends.cudnn as cudnn
import argparse
import os,sys

def gazeto3d(gaze):
    # Only used for ETH, which conduct gaze as [pitch yaw].
    assert gaze.size == 2, "The size of gaze must be 2"
    gaze_gt = np.zeros([3])
    gaze_gt[0] = -np.cos(gaze[0]) * np.sin(gaze[1])
    gaze_gt[1] = -np.sin(gaze[0])
    gaze_gt[2] = -np.cos(gaze[0]) * np.cos(gaze[1])
    return gaze_gt


def test(net,testdataset,logpath,Iter):


    if not os.path.exists(logpath):
        os.makedirs(logpath)


    length = len(testdataset); accs = 0; count = 0

        # Open log file ------------------------------------------------
    logname = f"Validation.log"

    count = 0
        # Testing --------------------------------------------------------------
    with torch.no_grad():
        with open(os.path.join(logpath, logname), 'a+') as outfile:
            for j, (data, label) in enumerate(testdataset):

                gts = label.cuda()
                data["face"] = data["face"].cuda()
                results = net(data["face"])

                    # Cal error between each pair of result and gt ------------------
                for k, result in enumerate(results):

                    result = result.cpu().detach().numpy()
                    gt = gts[k].cpu().numpy()

                    accs += gtools.angular(gazeto3d(gt),
                                gazeto3d(result))

                    count += 1

                    result = [str(u) for u in result] 
                    gt     = [str(u) for u in gt]

                    #log = name + [",".join(result)]  +  [",".join(gt)]

                    # outfile.write(" ".join(log) + "\n")
            log = f"[{Iter}] Total Num: {count}, avg: {accs/count}"+"\n"
            outfile.write(log)
            print(log)
    


def main(config):

    #  ===================>> Setup <<=================================

    dataloader = importlib.import_module("reader." + config.reader)

    torch.cuda.set_device(config.device) 
    #device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    cudnn.benchmark = True

    data = config.data
    val_data = config.val
    
    save = config.save
    params = config.params
    

    print("===> Read data <===")

    if data.isFolder:
        data, _ = ctools.readfolder(data)

    dataset = dataloader.loader(
                    data,
                    params.batch_size, 
                    shuffle=True, 
                    num_workers=params.num_workers
                )
    
    testdataset = dataloader.loader(
                    val_data,
                    params.batch_size, 
                    shuffle=False, 
                    num_workers=params.num_workers
                )


    print("===> Model building <===")

    net = timm.create_model("resnet18", pretrained=True, num_classes=2)
    net.cuda()


    # Pretrain 
    pretrain = config.pretrain

    if pretrain.enable and pretrain.device:
        net.load_state_dict( 
                torch.load(
                    pretrain.path, 
                    map_location={f"cuda:{pretrain.device}": f"cuda:{config.device}"}
                )
            )
    elif pretrain.enable and not pretrain.device:
        net.load_state_dict(
                torch.load(pretrain.path)
                )


    print("===> optimizer and criterion building <===")
    optimizer = optim.Adam(
                    net.parameters(),
                    lr=params.lr, 
                    betas=(0.9,0.999)
                )
    criterion = nn.L1Loss().cuda()
  
    scheduler = optim.lr_scheduler.StepLR( 
                    optimizer, 
                    step_size=params.decay_step, 
                    gamma=params.decay
                )

    savepath = os.path.join(save.metapath)

    if not os.path.exists(savepath):
        os.makedirs(savepath)
 
    # =====================================>> Training << ====================================
    print("===> Training <===")

    length = len(dataset); total = length * params.epoch
    timer = ctools.TimeCounter(total)


    optimizer.zero_grad()
    optimizer.step()
    scheduler.step()


    with open(os.path.join(savepath, "train_log"), 'w') as outfile:
        outfile.write(ctools.DictDumps(config) + '\n')

        for epoch in range(1, params.epoch+1):
            for i, (data, anno) in enumerate(dataset):

                # -------------- forward -------------
                for key in data:
                    if key != 'name': data[key] = data[key].cuda()

                anno = anno.cuda()
                out = net(data["face"])
                loss = criterion(out, anno)

                # -------------- Backward ------------
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                rest = timer.step()/3600

                if i % 20 == 0:
                    # test(net,testdataset,savepath,epoch)
                    log = f"[{epoch}/{params.epoch}]: " + \
                          f"[{i}/{length}] " +\
                          f"loss:{loss} " +\
                          f"lr:{ctools.GetLR(optimizer)} " +\
                          f"rest time:{rest:.2f}h"

                    print(log); outfile.write(log + "\n")
                    sys.stdout.flush(); outfile.flush()

            scheduler.step()
            
            test(net,testdataset,savepath,epoch)
            
            if epoch % save.step == 0:
                torch.save(
                        net.state_dict(), 
                        os.path.join(
                            savepath, 
                            f"Iter_{epoch}_{save.model_name}.pth"
                            )
                        )
                
# python train.py -s config_eth.yaml
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Pytorch Basic Model Training')

    parser.add_argument('-s', '--train', type=str,
                        help='The source config for training.')

    args = parser.parse_args()

    config = edict(yaml.load(open(args.train), Loader=yaml.FullLoader))

    print("=====================>> (Begin) Training params << =======================")
    print(ctools.DictDumps(config))
    print("=====================>> (End) Traning params << =======================")

    main(config.train)

