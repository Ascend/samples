English|[中文](README_CN.md)

#  Data processing related sample

#### Directory structure and description
This catalog contains the media data processing functions supported by Atlas, and each folder corresponds to different functions for users' reference. The directory structure and specific instructions are as follows.
| Sample  | description  | support chip |
|---|---|---|
| [batchcrop](./batchcrop)  | Crop interface, cut out, one picture with multiple frames  | Ascend310，Ascend310P，Ascend910 |
| [crop](./crop)  | Crop interface, cut out the image area that needs to be used from the input image  | Ascend310 |
| [cropandpaste](./cropandpaste)  | Cropandpaste interface, cut out the picture from the input picture, after the cut out picture is scaled, placed in the designated area of the user output picture | Ascend310 |
| [ffmpegdecode](./ffmpegdecode) | Example of calling ffmpeg interface to realize video frame cutting function | Ascend310 |
| [jpegd](./jpegd)  | jpegd interface, to achieve the decoding of .jpg, .jpeg, .JPG, .JPEG pictures  | Ascend310 |
| [jpege](./jpege)  | jpege interface, encoding YUV format pictures into picture files in JPEG compression format  | Ascend310 |
| [resize](./resize)  | resize interface. Zoom in and zoom out the image  | Ascend310 |
| [smallResolution_cropandpaste](./smallResolution_cropandpaste)  | cropandpaste interface. Cut out the specified input picture, and then paste the picture to the output picture  | Ascend310，Ascend310P，Ascend910 |
| [vdec](./vdec)  | vdec interface, to achieve video decoding, output YUV420SP format (including NV12 and NV21) pictures  | Ascend310 |
| [vdecandvenc](./vdecandvenc)  | vdec interface and venc interface, call dvpp's venc and vdec interface to realize video encoding function  | Ascend310 |
| [venc](./venc) | venc interface, encode the original mp4 file data into a video stream in H264/H265 format | Ascend310 |
| [venc_image](./venc_image) | venc interface, encode a picture in YUV420SP NV12 format n times continuously to generate a video stream in H265 format | Ascend310，Ascend310P |