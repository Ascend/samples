# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import copy
import logging
import numpy as np
from typing import List, Optional, Union
import torch
from torch.nn import functional as F

from detectron2.config import configurable

from . import detection_utils as utils
from . import transforms as T
from detectron2.structures import BitMasks
from detectron2.structures import Boxes
from detectron2.structures import BoxMode
from detectron2.structures import Instances
from detectron2.structures import Keypoints
from detectron2.structures import PolygonMasks
from detectron2.structures import RotatedBoxes
from detectron2.structures import polygons_to_bitmask


"""
This file contains the default mapping that's applied to "dataset dicts".
"""

__all__ = ["DatasetMapper"]


class DatasetMapper:
    """
    A callable which takes a dataset dict in Detectron2 Dataset format,
    and map it into a format used by the model.

    This is the default callable to be used to map your dataset dict into training data.
    You may need to follow it to implement your own one for customized logic,
    such as a different way to read or transform images.
    See :doc:`/tutorials/data_loading` for details.

    The callable currently does the following:

    1. Read the image from "file_name"
    2. Applies cropping/geometric transforms to the image and annotations
    3. Prepare data and annotations to Tensor and :class:`Instances`
    """

    @configurable
    def __init__(
        self,
        is_train: bool,
        *,
        augmentations: List[Union[T.Augmentation, T.Transform]],
        image_format: str,
        use_instance_mask: bool = False,
        use_keypoint: bool = False,
        instance_mask_format: str = "polygon",
        keypoint_hflip_indices: Optional[np.ndarray] = None,
        precomputed_proposal_topk: Optional[int] = None,
        recompute_boxes: bool = False,
        fix_shape: tuple = None,
        amp: int = 0,
        opt_level: str = "O0"
    ):
        """
        NOTE: this interface is experimental.

        Args:
            is_train: whether it's used in training or inference
            augmentations: a list of augmentations or deterministic transforms to apply
            image_format: an image format supported by :func:`detection_utils.read_image`.
            use_instance_mask: whether to process instance segmentation annotations, if available
            use_keypoint: whether to process keypoint annotations if available
            instance_mask_format: one of "polygon" or "bitmask". Process instance segmentation
                masks into this format.
            keypoint_hflip_indices: see :func:`detection_utils.create_keypoint_hflip_indices`
            precomputed_proposal_topk: if given, will load pre-computed
                proposals from dataset_dict and keep the top k proposals for each image.
            recompute_boxes: whether to overwrite bounding box annotations
                by computing tight bounding boxes from instance mask annotations.
        """
        if recompute_boxes:
            assert use_instance_mask, "recompute_boxes requires instance masks"
        # fmt: off
        self.is_train               = is_train
        self.augmentations          = augmentations
        self.image_format           = image_format
        self.use_instance_mask      = use_instance_mask
        self.instance_mask_format   = instance_mask_format
        self.use_keypoint           = use_keypoint
        self.keypoint_hflip_indices = keypoint_hflip_indices
        self.proposal_topk          = precomputed_proposal_topk
        self.recompute_boxes        = recompute_boxes
        # fix shape
        self.fix_shape              = fix_shape
        self.amp                    = amp
        self.opt_level              = opt_level
        # fmt: on
        logger = logging.getLogger(__name__)
        logger.info("Augmentations used in training: " + str(augmentations))

    @classmethod
    def from_config(cls, cfg, is_train: bool = True):
        augs = utils.build_augmentation(cfg, is_train)
        if cfg.INPUT.CROP.ENABLED and is_train:
            augs.insert(0, T.RandomCrop(cfg.INPUT.CROP.TYPE, cfg.INPUT.CROP.SIZE))
            recompute_boxes = cfg.MODEL.MASK_ON
        else:
            recompute_boxes = False

        ret = {
            "is_train": is_train,
            "augmentations": augs,
            "image_format": cfg.INPUT.FORMAT,
            "use_instance_mask": cfg.MODEL.MASK_ON,
            "instance_mask_format": cfg.INPUT.MASK_FORMAT,
            "use_keypoint": cfg.MODEL.KEYPOINT_ON,
            "recompute_boxes": recompute_boxes,
            "fix_shape": cfg.INPUT.FIX_SHAPE,
            "amp": cfg.AMP,
            "opt_level": cfg.OPT_LEVEL
        }
        if cfg.MODEL.KEYPOINT_ON:
            ret["keypoint_hflip_indices"] = utils.create_keypoint_hflip_indices(cfg.DATASETS.TRAIN)

        if cfg.MODEL.LOAD_PROPOSALS:
            ret["precomputed_proposal_topk"] = (
                cfg.DATASETS.PRECOMPUTED_PROPOSAL_TOPK_TRAIN
                if is_train
                else cfg.DATASETS.PRECOMPUTED_PROPOSAL_TOPK_TEST
            )
        return ret

    def __call__(self, dataset_dict):
        """
        Args:
            dataset_dict (dict): Metadata of one image, in Detectron2 Dataset format.

        Returns:
            dict: a format that builtin models in detectron2 accept
        """
        dataset_dict = copy.deepcopy(dataset_dict)  # it will be modified by code below
        # USER: Write your own image loading if it's not from a file
        image = utils.read_image(dataset_dict["file_name"], format=self.image_format)
        utils.check_image_size(dataset_dict, image)

        # USER: Remove if you don't do semantic/panoptic segmentation.
        if "sem_seg_file_name" in dataset_dict:
            sem_seg_gt = utils.read_image(dataset_dict.pop("sem_seg_file_name"), "L").squeeze(2)
        else:
            sem_seg_gt = None

        aug_input = T.StandardAugInput(image, sem_seg=sem_seg_gt)
        transforms = aug_input.apply_augmentations(self.augmentations)
        image, sem_seg_gt = aug_input.image, aug_input.sem_seg

        image_shape = image.shape[:2]  # h, w
        # Pytorch's dataloader is efficient on torch.Tensor due to shared-memory,
        # but not efficient on large generic data structures due to the use of pickle & mp.Queue.
        # Therefore it's important to use torch.Tensor.
        dataset_dict["image"] = torch.as_tensor(np.ascontiguousarray(image.transpose(2, 0, 1)))
        # put preprocess in dataset
        logger = logging.getLogger(__name__)
        size_divisibility = 32
        pad_value = 0
        pixel_mean = torch.Tensor([103.53, 116.28, 123.675]).view(-1, 1, 1)
        pixel_std = torch.Tensor([1.0, 1.0, 1.0]).view(-1, 1, 1)
        images = (dataset_dict["image"] - pixel_mean) / pixel_std

        dataset_dict["image_size"] = tuple(images.shape[-2:])

        batch_shape = (3, self.fix_shape[1], self.fix_shape[0])
        padding_size = [0, batch_shape[-1] - images.shape[-1],
                        0, batch_shape[-2] - images.shape[-2]]
        padded = F.pad(images, padding_size, value=pad_value)
        batched_imgs = padded.unsqueeze_(0)

        if self.amp and (self.opt_level == "O1" or self.opt_level == "O2"):
            batched_imgs = batched_imgs.to(torch.float16, non_blocking=True)
        dataset_dict["image_preprocess"] = batched_imgs.contiguous()

        if sem_seg_gt is not None:
            dataset_dict["sem_seg"] = torch.as_tensor(sem_seg_gt.astype("long"))

        # USER: Remove if you don't use pre-computed proposals.
        # Most users would not need this feature.
        if self.proposal_topk is not None:
            utils.transform_proposals(
                dataset_dict, image_shape, transforms, proposal_topk=self.proposal_topk
            )

        if not self.is_train:
            # USER: Modify this if you want to keep them for some reason.
            dataset_dict.pop("annotations", None)
            dataset_dict.pop("sem_seg_file_name", None)
            return dataset_dict

        if "annotations" in dataset_dict:
            # USER: Modify this if you want to keep them for some reason.
            for anno in dataset_dict["annotations"]:
                if not self.use_instance_mask:
                    anno.pop("segmentation", None)
                if not self.use_keypoint:
                    anno.pop("keypoints", None)

            # USER: Implement additional transformations if you have other types of data
            annos = [
                utils.transform_instance_annotations(
                    obj, transforms, image_shape, keypoint_hflip_indices=self.keypoint_hflip_indices
                )
                for obj in dataset_dict.pop("annotations")
                if obj.get("iscrowd", 0) == 0
            ]

            instances = utils.annotations_to_instances(
                annos, image_shape, mask_format=self.instance_mask_format
            )

            # After transforms such as cropping are applied, the bounding box may no longer
            # tightly bound the object. As an example, imagine a triangle object
            # [(0,0), (2,0), (0,2)] cropped by a box [(1,0),(2,2)] (XYXY format). The tight
            # bounding box of the cropped triangle should be [(1,0),(2,1)], which is not equal to
            # the intersection of original bounding box and the cropping box.
            if self.recompute_boxes:
                instances.gt_boxes = instances.gt_masks.get_bounding_boxes()

            if not self.is_train:
                dataset_dict["instances"] = \
                    utils.filter_empty_instances(instances)
            else:
                i = utils.filter_empty_instances(instances)
                classes = 80
                max_len = 20
                boxes_num = len(i.gt_boxes)
                if boxes_num < max_len:
                    diff_num = max_len - boxes_num
                    i.gt_boxes.tensor = torch.cat(
                        [i.gt_boxes.tensor,torch.zeros([diff_num, 4])],dim=0)
                    padding_array = np.zeros([diff_num]) + classes
                    i.gt_classes = torch.cat(
                        [i.gt_classes, torch.from_numpy(
                            padding_array).long()],
                        dim=0)
                    if self.use_instance_mask:
                        if isinstance(i.gt_masks, PolygonMasks):
                            i.gt_masks.polygons += [torch.from_numpy(
                                np.zeros([1] ))]*diff_num
                        elif isinstance(i.gt_masks, BitMasks):
                            padding_mask = torch.zeros(
                                [diff_num, i.gt_masks.tensor.shape[1],
                                 i.gt_masks.tensor.shape[2]]).bool()
                            i.gt_masks.tensor = torch.cat(
                                [i.gt_masks.tensor, padding_mask], dim=0)
                            i.gt_masks.tensor = F.pad(
                                i.gt_masks.tensor, padding_size, value=False)
                else:
                    select_idx = torch.randperm(boxes_num)[:max_len]
                    i.gt_boxes.tensor = i.gt_boxes.tensor[select_idx]
                    i.gt_classes = i.gt_classes[select_idx]
                    if self.use_instance_mask:
                        if isinstance(i.gt_masks, PolygonMasks):
                            i.gt_masks.polygons = [
                                i.gt_masks.polygons[idx]
                                for idx in select_idx.numpy().tolist()]
                        elif isinstance(i.gt_masks, BitMasks):
                            i.gt_masks.tensor = i.gt_masks.tensor[select_idx]
                            i.gt_masks.tensor = F.pad(
                                i.gt_masks.tensor, padding_size, value=False)
                dataset_dict["instances"] = i
        return dataset_dict
