/**
* @file utils.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "utils.h"

bool RunStatus::isDevice_ = false;

Result Utils::AlignSpImage(const PictureDesc &picDesc, uint8_t *inBuffer, uint8_t *imageBuffer,
    uint32_t imageBufferSize, aclrtMemcpyKind memcpyType)
{
    aclError aclRet;
    // copy y data
    for (auto i = 0; i < picDesc.height; i++) {
        aclRet = aclrtMemcpy(inBuffer + i * picDesc.widthStride, picDesc.width,
            imageBuffer + i * picDesc.width, picDesc.width, memcpyType);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("memcpy failed, errorCode = %d", static_cast<int32_t>(aclRet));
            return FAILED;
        }
    }
    // copy uv data
    int32_t uvHigh = picDesc.height;
    int32_t uvWidthCoefficient = 1;
    switch (picDesc.format) {
        case PIXEL_FORMAT_YUV_400:
            uvHigh = 0;
            break;
        case PIXEL_FORMAT_YUV_SEMIPLANAR_420:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_420:
            uvHigh /= 2;
            break;
        case PIXEL_FORMAT_YUV_SEMIPLANAR_422:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_422:
        case PIXEL_FORMAT_YUV_SEMIPLANAR_440:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_440:
            break;
        case PIXEL_FORMAT_YUV_SEMIPLANAR_444:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_444:
            uvWidthCoefficient = 2;
            break;
        default:
            ERROR_LOG("format(%d) cannot support", picDesc.format);
            return FAILED;
    }
    uint8_t *uvSrcData = inBuffer + picDesc.widthStride * picDesc.heightStride;
    uint8_t *uvDstData = imageBuffer + picDesc.width * picDesc.height;
    for (auto i = 0; i < uvHigh; i++) {
        aclRet = aclrtMemcpy(uvSrcData + i * picDesc.widthStride * uvWidthCoefficient, picDesc.width * uvWidthCoefficient,
            uvDstData + i * picDesc.width * uvWidthCoefficient, picDesc.width * uvWidthCoefficient, memcpyType);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("memcpy failed, errorCode = %d", static_cast<int32_t>(aclRet));
            return FAILED;
        }
    }

    return SUCCESS;
}

Result Utils::AlignPackedImage(const PictureDesc &picDesc, uint8_t *inBuffer, uint8_t *imageBuffer,
    uint32_t imageBufferSize, aclrtMemcpyKind memcpyType)
{
    uint32_t coefficient = 1;
    switch (picDesc.format) {
        case PIXEL_FORMAT_YUYV_PACKED_422:
        case PIXEL_FORMAT_UYVY_PACKED_422:
        case PIXEL_FORMAT_YVYU_PACKED_422:
        case PIXEL_FORMAT_VYUY_PACKED_422:
            coefficient = 2;
            break;
        case PIXEL_FORMAT_YUV_PACKED_444:
        case PIXEL_FORMAT_RGB_888:
        case PIXEL_FORMAT_BGR_888:
            coefficient = 3;
            break;
        case PIXEL_FORMAT_ARGB_8888:
        case PIXEL_FORMAT_ABGR_8888:
        case PIXEL_FORMAT_RGBA_8888:
        case PIXEL_FORMAT_BGRA_8888:
            coefficient = 4;
            break;
        default:
            ERROR_LOG("format(%d) cannot support", picDesc.format);
            return FAILED;
    }
    for (auto i = 0; i < picDesc.height; i++) {
        aclError aclRet = aclrtMemcpy(inBuffer + i * picDesc.widthStride, picDesc.width * coefficient,
            imageBuffer + i * picDesc.width * coefficient, picDesc.width * coefficient, memcpyType);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("memcpy failed, errorCode = %d", static_cast<int32_t>(aclRet));
            return FAILED;
        }
    }
    return SUCCESS;
}

void *Utils::GetPicDevBuffer(const PictureDesc &picDesc)
{
    if (picDesc.fileName.empty()) {
        ERROR_LOG("picture file name is empty");
        return nullptr;
    }

    FILE *fp = fopen(picDesc.fileName.c_str(), "rb");
    if (fp == nullptr) {
        ERROR_LOG("open file %s failed", picDesc.fileName.c_str());
        return nullptr;
    }
    fseek(fp, 0, SEEK_END);
    auto fileSize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    void *picDevBuff = nullptr;
    aclError aclRet = acldvppMalloc(&picDevBuff, picDesc.bufferSize);
    if (aclRet !=  ACL_SUCCESS) {
        ERROR_LOG("malloc device data buffer failed, errorCode = %d", static_cast<int32_t>(aclRet));
        fclose(fp);
        return nullptr;
    }

    void *imageBuffer = malloc(fileSize);
    if (imageBuffer == nullptr) {
        fclose(fp);
        (void)acldvppFree(picDevBuff);
        return nullptr;
    }

    fread(imageBuffer, sizeof(char), fileSize, fp);
    fclose(fp);
    fp = nullptr;

    if (!(RunStatus::GetDeviceStatus())) { // app is running in host
        void *inHostBuff = nullptr;
        aclRet = aclrtMallocHost(&inHostBuff, picDesc.bufferSize);
        if (aclRet !=  ACL_SUCCESS) {
            ERROR_LOG("malloc host data buffer failed, errorCode = %d", static_cast<int32_t>(aclRet));
            (void)acldvppFree(picDevBuff);
            free(imageBuffer);
            return nullptr;
        }
        Result ret;
        if (picDesc.format <= PIXEL_FORMAT_YVU_SEMIPLANAR_444) {
            ret = AlignSpImage(picDesc, (uint8_t *)inHostBuff, (uint8_t *)imageBuffer,
                fileSize, ACL_MEMCPY_HOST_TO_HOST);
        } else {
            ret = AlignPackedImage(picDesc, (uint8_t *)inHostBuff, (uint8_t *)imageBuffer,
                fileSize, ACL_MEMCPY_HOST_TO_HOST);
        }
        if (ret != SUCCESS) {
            (void)acldvppFree(picDevBuff);
            (void)aclrtFreeHost(inHostBuff);
            free(imageBuffer);
            return nullptr;
        }
        // if app is running in host, need copy model output data from host to device
        aclRet = aclrtMemcpy(picDevBuff, picDesc.bufferSize, inHostBuff,
            picDesc.bufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("memcpy from host to device failed, errorCode = %d", static_cast<int32_t>(aclRet));
            (void)acldvppFree(picDevBuff);
            (void)aclrtFreeHost(inHostBuff);
            free(imageBuffer);
            return nullptr;
        }
        (void)aclrtFreeHost(inHostBuff);
    } else { // app is running in device
        Result ret;
        if (picDesc.format <= PIXEL_FORMAT_YVU_SEMIPLANAR_444) {
            ret = AlignSpImage(picDesc, (uint8_t *)picDevBuff, (uint8_t *)imageBuffer,
                fileSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
        } else {
            ret = AlignPackedImage(picDesc, (uint8_t *)picDevBuff, (uint8_t *)imageBuffer,
                fileSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
        }
        if (ret != SUCCESS) {
            (void)acldvppFree(picDevBuff);
            free(imageBuffer);
            return nullptr;
        }
    }

    free(imageBuffer);

    return picDevBuff;
}

Result Utils::SaveDvppOutputData(const char *fileName, void *devPtr, uint32_t dataSize)
{
    void *dataPtr = nullptr;
    aclError aclRet;
    if (!(RunStatus::GetDeviceStatus())) {
        aclRet = aclrtMallocHost(&dataPtr, dataSize);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("malloc host data buffer failed, errorCode = %d", static_cast<int32_t>(aclRet));
            return FAILED;
        }

        aclRet = aclrtMemcpy(dataPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("dvpp output memcpy to host failed, errorCode = %d", static_cast<int32_t>(aclRet));
            (void)aclrtFreeHost(dataPtr);
            return FAILED;
        }
    } else {
        dataPtr = devPtr;
    }

    FILE *outFileFp = fopen(fileName, "wb+");
    if (outFileFp == nullptr) {
        ERROR_LOG("fopen out file %s failed.", fileName);
        if (!(RunStatus::GetDeviceStatus())) {
            (void)aclrtFreeHost(dataPtr);
        }
        return FAILED;
    }

    size_t writeSize = fwrite(dataPtr, sizeof(char), dataSize, outFileFp);
    if (writeSize != dataSize) {
        ERROR_LOG("need write %u bytes to %s, but only write %zu bytes.",
            dataSize, fileName, writeSize);
        fclose(outFileFp);
        if (!(RunStatus::GetDeviceStatus())) {
            (void)aclrtFreeHost(dataPtr);
        }
        return FAILED;
    }

    if (!(RunStatus::GetDeviceStatus())) {
        (void)aclrtFreeHost(dataPtr);
    }
    fflush(outFileFp);
    fclose(outFileFp);
    return SUCCESS;
}
