English|[中文](README_CN.md)

# Semantic segmentation sample

#### Directory structure and description
This catalog is a sample of semantic segmentation for users' reference. The directory structure and specific instructions are as follows.
- **deeplabv3-MindSpore samples**
  | Sample name  | Sample description  | Characteristic analysis  | support chip |
  |---|---|---|---|
  | [deeplabv3](./deeplabv3)  | segmentation   | Both input and output are pictures, and there are two sets of running codes, mainly introducing opencv processing and dvpp processing.   | Ascend310 |
  | [deeplabv3_postprocess_optimization](./googlenet_imagenet_multi_batch)  | segmentation   | Both input and output are pictures, and there are two sets of running codes, mainly introducing post-processing optimization and python multi-process processing.  | Ascend310 |

- **other samples**
  | sample  | description  | support chip |
  |---|---|---|
  | [deeplabv3_pascal_pic](./deeplabv3_pascal_pic) | Use the deeplabv3+ model to perform semantic segmentation on the input image | Ascend310 |