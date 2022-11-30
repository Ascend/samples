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
import logging
import math
from typing import List, Tuple
import torch

from detectron2.layers import batched_nms, cat
from detectron2.structures import Boxes, Instances

logger = logging.getLogger(__name__)


def find_top_rpn_proposals(
    proposals: List[torch.Tensor],
    pred_objectness_logits: List[torch.Tensor],
    image_sizes: List[Tuple[int, int]],
    nms_thresh: float,
    pre_nms_topk: int,
    post_nms_topk: int,
    min_box_size: float,
    training: bool,
):
    """
    For each feature map, select the `pre_nms_topk` highest scoring proposals,
    apply NMS, clip proposals, and remove small boxes. Return the `post_nms_topk`
    highest scoring proposals among all the feature maps for each image.

    Args:
        proposals (list[Tensor]): A list of L tensors. Tensor i has shape (N, Hi*Wi*A, 4).
            All proposal predictions on the feature maps.
        pred_objectness_logits (list[Tensor]): A list of L tensors. Tensor i has shape (N, Hi*Wi*A).
        image_sizes (list[tuple]): sizes (h, w) for each image
        nms_thresh (float): IoU threshold to use for NMS
        pre_nms_topk (int): number of top k scoring proposals to keep before applying NMS.
            When RPN is run on multiple feature maps (as in FPN) this number is per
            feature map.
        post_nms_topk (int): number of top k scoring proposals to keep after applying NMS.
            When RPN is run on multiple feature maps (as in FPN) this number is total,
            over all feature maps.
        min_box_size (float): minimum proposal box side length in pixels (absolute units
            wrt input images).
        training (bool): True if proposals are to be used in training, otherwise False.
            This arg exists only to support a legacy bug; look for the "NB: Legacy bug ..."
            comment.

    Returns:
        list[Instances]: list of N Instances. The i-th Instances
            stores post_nms_topk object proposals for image i, sorted by their
            objectness score in descending order.
    """
    num_images = len(image_sizes)
    device = proposals[0].device
    topk_scores_list = []  # #lvl Tensor, each of shape N x topk
    topk_proposals_list = []
    level_ids_list = []  # #lvl Tensor, each of shape (topk,)
    batch_idx = torch.arange(num_images, device=device)
    for level_id, (proposals_i, logits_i) in enumerate(zip(proposals, pred_objectness_logits)):
        Hi_Wi_A = logits_i.shape[1]
        num_proposals_i = min(pre_nms_topk, Hi_Wi_A)
        topk_scores_i, topk_idx = torch.topk(logits_i, num_proposals_i, dim=1)
        topk_proposals_i = proposals_i[batch_idx[:, None].long(),
                                       topk_idx.long()]  # N x topk x 4
        topk_proposals_list.append(topk_proposals_i)
        topk_scores_list.append(topk_scores_i)
        level_ids_list.append(
            torch.full((num_proposals_i,), level_id,
                       dtype=torch.int32, device=device))

    results: List[Instances] = []
    for n, image_size in enumerate(image_sizes):
        level_keep_list = []
        level_boxes_list = []
        level_scores_per_img = []

        for level in range(len(topk_proposals_list)):
            topk_proposals = topk_proposals_list[level]
            topk_scores = topk_scores_list[level]
            level_ids = level_ids_list[level]

            boxes = Boxes(topk_proposals[n])
            scores_per_img = topk_scores[n]
            lvl = level_ids

            if not training:
                valid_mask = torch.isfinite(boxes.tensor).all(dim=1) & torch.isfinite(scores_per_img.float())
                if not valid_mask.all():
                    boxes = boxes[valid_mask]
                    scores_per_img = scores_per_img[valid_mask]
                    lvl = lvl[valid_mask]

            if scores_per_img.dtype != torch.float32:
                scores_per_img = scores_per_img.to(torch.float32)

            keep_mask = batched_nms(boxes.tensor,
                                    scores_per_img, lvl, nms_thresh)

            level_keep_list.append(keep_mask)
            level_boxes_list.append(boxes)
            level_scores_per_img.append(scores_per_img)

        keep_mask = cat(level_keep_list, dim=0)
        boxes = Boxes.cat(level_boxes_list)
        scores_per_img = cat(level_scores_per_img, dim=0)
        scores_per_img = scores_per_img * keep_mask.float()

        topk_scores_i, indice = torch.topk(scores_per_img, post_nms_topk)

        res = Instances(image_size)
        res.proposal_boxes = boxes[indice.long()]
        res.objectness_logits = topk_scores_i
        results.append(res)
    return results

def add_ground_truth_to_proposals(gt_boxes, proposals):
    """
    Call `add_ground_truth_to_proposals_single_image` for all images.

    Args:
        gt_boxes(list[Boxes]): list of N elements. Element i is a Boxes
            representing the gound-truth for image i.
        proposals (list[Instances]): list of N elements. Element i is a Instances
            representing the proposals for image i.

    Returns:
        list[Instances]: list of N Instances. Each is the proposals for the image,
            with field "proposal_boxes" and "objectness_logits".
    """
    assert gt_boxes is not None

    assert len(proposals) == len(gt_boxes)
    if len(proposals) == 0:
        return proposals

    return [
        add_ground_truth_to_proposals_single_image(gt_boxes_i, proposals_i)
        for gt_boxes_i, proposals_i in zip(gt_boxes, proposals)
    ]


def add_ground_truth_to_proposals_single_image(gt_boxes, proposals):
    """
    Augment `proposals` with ground-truth boxes from `gt_boxes`.

    Args:
        Same as `add_ground_truth_to_proposals`, but with gt_boxes and proposals
        per image.

    Returns:
        Same as `add_ground_truth_to_proposals`, but for only one image.
    """
    device = proposals.objectness_logits.device
    gt_logit_value = math.log((1.0 - 1e-10) / (1 - (1.0 - 1e-10)))
    gt_logits = gt_logit_value * torch.ones(len(gt_boxes), device=device)

    # Concatenating gt_boxes with proposals requires them to have the same fields
    gt_proposal = Instances(proposals.image_size)
    gt_proposal.proposal_boxes = gt_boxes
    gt_proposal.objectness_logits = gt_logits
    new_proposals = Instances.cat([proposals, gt_proposal])

    return new_proposals
