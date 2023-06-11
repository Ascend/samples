/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File AclLiteImageProc.h
*/
#ifndef ACLLITE_IMAGE_PROC_H
#define ACLLITE_IMAGE_PROC_H
#pragma once
#include <cstdint>
#include "AclLiteUtils.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

class AclLiteImageProc {
public:
    /**
    * @brief Constructor
    */
    AclLiteImageProc();
    /**
    * @brief Destructor
    */
    ~AclLiteImageProc();

    AclLiteError Init();
    /**
    * @brief Scale the image to size (width, height)
    * @param [in]: dest: resized image
    * @param [in]: src: original image
    * @param [in]: width: resized image width
    * @param [in]: height: resized image height
    * @return AclLiteError ACLLITE_OK: read success
    * others: resize failed
    */
    AclLiteError Resize(ImageData& dest, ImageData& src,
                        uint32_t width, uint32_t height);
    /**
    * @brief Realize the decoding of .jpg, .jpeg, .JPG, .JPEG image files.
    * @param [in]: destYuv: decoded yuv image
    * @param [in]: srcJpeg: jpeg image to be decoded
    * @return AclLiteError ACLLITE_OK: read success
    * others: JpegD failed
    */
    AclLiteError JpegD(ImageData& destYuv, ImageData& srcJpeg);
    /**
    * @brief JPEGE encodes images in YUV format into image files
    * in JPEG compression format, such as *.jpg.
    * @param [in]: destJpeg: the encoded jpeg image
    * @param [in]: srcYuv: the yuv image to be encoded
    * @return AclLiteError ACLLITE_OK: read success
    * others: JpegE failed
    */
    AclLiteError JpegE(ImageData& destJpeg, ImageData& srcYuv);
    /**
    * @brief Decode png image to rgb image.
    * @param [in]: destRgb: decoded rgb picture
    * @param [in]: srcPng: png image to be decoded
    * @return AclLiteError ACLLITE_OK: read success
    * others: PngD failed
    */
    AclLiteError PngD(ImageData& dest, ImageData& src);
    /**
    * @brief Crop the desired image area from the input image
    * @param [in]: dest: decoded image
    * @param [in]: src: original image
    * @param [in]: ltHorz: X coordinate of the upper left point
    * @param [in]: ltVert: Y coordinate of the upper left point
    * @param [in]: rbHorz: X coordinate of the lower right point
    * @param [in]: rbVert: Y coordinate of the lower right point
    * @return AclLiteError ACLLITE_OK: read success
    * others: Crop failed
    */
    AclLiteError Crop(ImageData& dest, ImageData& src,
                      uint32_t ltHorz, uint32_t ltVert,
                      uint32_t rbHorz, uint32_t rbVert);
    /**
    * @brief crop out the rectangular area determined by
    * (ltHorz, ltVert), (rbHorz, rbVert) two points from the original image,
    * and paste it to the texture area (0, 0) (width, height)
    * @param [in]: dest: image data after CropPaste
    * @param [in]: src: image data to be processed
    * @param [in]: width: the width of the image after the CropPaste
    * @param [in]: height: The height of the image after the CropPaste
    * @param [in]: ltHorz: Determine the X coordinate of
    * the upper left point of the cutout area
    * @param [in]: ltVert: Determine the Y coordinate of
    * the upper left point of the cutout area
    * @param [in]: rbHorz: Determine the X coordinate of
    * the lower right point of the cutout area
    * @param [in]: rbVert: Determines the Y coordinate of
    * the lower right point of the cutout area
    * @return AclLiteError ACLLITE_OK: read success
    * others: CropPaste failed
    */
    AclLiteError CropPaste(ImageData& dest, ImageData& src,
                           uint32_t width, uint32_t height,
                           uint32_t ltHorz, uint32_t ltVert,
                           uint32_t rbHorz, uint32_t rbVert);
    /**
    * @brief Proportional texture, paste the original image data
    * to the area (0, 0) without changing the aspect ratio
    * (rbHorz-ltHorz, ltVert-rbVert)
    * @param [in]: dest: image data after matting
    * @param [in]: src: image data to be processed
    * @param [in]: ltHorz: X coordinate of the upper left point
    * @param [in]: ltVert: Y coordinate of the upper left point
    * @param [in]: rbHorz: X coordinate of the lower right point
    * @param [in]: rbVert: Y coordinate of the lower right point
    * @return AclLiteError ACLLITE_OK: read success
    * others: ProportionPaste failed
    */
    AclLiteError ProportionPaste(ImageData& dest, ImageData& src,
                                 uint32_t ltHorz, uint32_t ltVert,
                                 uint32_t rbHorz, uint32_t rbVert);
    /**
    * @brief Center proportional texture, paste the original image data
    * to the center of the texture area (width, height)
    * without changing the aspect ratio
    * @param [in]: dest: image data after matting
    * @param [in]: src: image data to be processed
    * @param [in]: width: the width of the image in the texture area
    * @param [in]: height: The height of the image in the texture area
    * @return AclLiteError ACLLITE_OK: read success
    * others: ProportionPasteCenter failed
    */
    AclLiteError ProportionPasteCenter(ImageData& dest, ImageData& src,
                                       uint32_t width, uint32_t height);
    void DestroyResource();

protected:
    bool isReleased_;
    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;
    int isInitOk_;
};
#endif