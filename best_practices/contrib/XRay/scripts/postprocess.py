import os
import sys
import time
import argparse
import numpy as np


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resdir", type=str, default="./scripts/new_result/bs1", \
        help="the path of the model inference result")
    parser.add_argument("--labeldir", type=str, default="./scripts/bin_mask/", \
        help="the path of the ground truth mask")
    args = parser.parse_args()
    return args


def batch_pix_accuracy(output, target):
    """pix_acc"""
    # inputs are numpy array, output 4D, target 3D
    predict = np.argmax(output, 1) + 1
    target = target + 1

    pixel_labeled = np.sum(target > 0).item()
    pixel_correct = np.sum((predict == target) * (target > 0)).item()
    assert pixel_correct <= pixel_labeled, "Correct area should be smaller than Labeled"
    return pixel_correct, pixel_labeled


def batch_intersection_union(output, target, nclass):
    """miou"""
    # inputs are numpy array, output 4D, target 3D
    mini = 1
    maxi = nclass
    nbins = nclass
    predict = np.argmax(output, 1) + 1
    target = target.astype(np.float32) + 1

    predict = predict.astype(np.float32) * (target > 0).astype(np.float32)
    intersection = predict * (predict == target).astype(np.float32)
    # areas of intersection and union
    # element 0 in intersection occur the main difference from np.bincount. set boundary to -1 is necessary.
    area_inter = np.histogram(intersection, bins=nbins, range=(mini, maxi))[0]
    area_pred = np.histogram(predict, bins=nbins, range=(mini, maxi))[0]
    area_lab = np.histogram(target, bins=nbins, range=(mini, maxi))[0]
    area_union = area_pred + area_lab - area_inter
    assert np.sum(area_inter > area_union).item() == 0, "Intersection area should be smaller than Union area"
    return area_inter.astype(np.float32), area_union.astype(np.float32)


class SegmentationMetric(object):
    """Computes pix_acc and miou metric scores
    """

    def __init__(self, nclass):
        super(SegmentationMetric, self).__init__()
        self.nclass = nclass
        self.reset()

    def update(self, preds, labels):
        """Updates the internal evaluation result.

        Parameters
        ----------
        labels : 'NumpyArray' or list of `NumpyArray`
            The labels of the data.
        preds : 'NumpyArray' or list of `NumpyArray`
            Predicted values.
        """

        def evaluate_worker(self, pred, label):
            correct, labeled = batch_pix_accuracy(pred, label)
            inter, union = batch_intersection_union(pred, label, self.nclass)

            self.total_correct += correct
            self.total_label += labeled
            self.total_inter += inter
            self.total_union += union
        
        if isinstance(preds, np.ndarray):
            evaluate_worker(self, preds, labels)
        elif isinstance(preds, (list, tuple)):
            for (pred, label) in zip(preds, labels):
                evaluate_worker(self, pred, label)

    def get(self):
        """Gets the current evaluation result.

        Returns
        -------
        metrics : tuple of float
            pix_acc and miou
        """
        pix_acc = 1.0 * self.total_correct / (2.220446049250313e-16 + self.total_label)  # remove np.spacing(1)
        iou = 1.0 * self.total_inter / (2.220446049250313e-16 + self.total_union)
        miou = iou.mean().item()
        return pix_acc, miou

    def reset(self):
        """Resets the internal evaluation result to initial state."""
        self.total_inter = np.zeros(self.nclass)
        self.total_union = np.zeros(self.nclass)
        self.total_correct = 0
        self.total_label = 0


def eval(gl_res_dir, gl_label_dir):

    resLis = os.listdir(gl_res_dir)
    resLis.sort()
    metric = SegmentationMetric(4)
    metric.reset()
    pix_accs, mious = [], []
    start = time.time()
    file_list = []
    for file in resLis:
        if file == "sumary.json":
            continue
        file_list.append(file.split('.')[0][:-2])
    file_list = list(set(file_list))

    for file in file_list:
        output = np.fromfile(os.path.join(gl_res_dir, file+'_0.bin'), np.float32).reshape(1, 4, 512, 512)
        mask_file = file + '.bin'
        target = np.fromfile(os.path.join(gl_label_dir, mask_file), np.uint8).reshape(512, 512)
        metric.update(output, target)
        pix_acc, miou = metric.get()
        pix_accs.append(pix_acc)
        mious.append(miou)
    
    end = time.time()

    print("Infer time is {:.3f}s per img".format((end - start)/len(file_list)*36))
    print("validation pix_acc: {:.3f}, miou: {:.3f}".format(
            np.mean(pix_accs) * 100, np.mean(mious) * 100))


if __name__ == "__main__":
    args = get_args()
    eval(args.resdir, args.labeldir)