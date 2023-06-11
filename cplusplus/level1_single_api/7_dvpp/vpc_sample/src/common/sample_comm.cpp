/**
 *  Copyright [2021] Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include "sample_comm.h"

aclrtContext g_context = nullptr;

int32_t get_run_mode()
{
    aclError aclRet = aclrtGetRunMode(&g_run_mode);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("Get run mode fail! acl ret:%d\n", aclRet);
        return HI_FAILURE;
    }

    if (g_run_mode == ACL_HOST) {
        SAMPLE_PRT("Running in Host!\n");
    } else if (g_run_mode == ACL_DEVICE) {
        SAMPLE_PRT("Running in Device!\n");
    } else {
        SAMPLE_PRT("Running in Invalid platform! runMode:%u\n", g_run_mode);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

int32_t acl_init()
{
    aclError aclRet = aclInit(nullptr);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclInit failed with %d.\n", aclRet);
        return HI_FAILURE;
    }
    // By default, the program is running on device 0.
    // On a multi-P environment, you can choose target device by the following interface.
    aclRet = aclrtSetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtSetDevice(0) failed with %d.\n", aclRet);
        aclRet = aclFinalize();
        if (aclRet != ACL_SUCCESS) {
            SAMPLE_PRT("finalize acl failed with %d.\n", aclRet);
        }
        return HI_FAILURE;
    }

    aclRet = aclrtCreateContext(&g_context, 0);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("acl create context failed with %d.", aclRet);
        aclRet = aclrtResetDevice(0);
        if (aclRet != ACL_SUCCESS) {
            SAMPLE_PRT("reset device(0) failed with %d.\n", aclRet);
        }
        aclRet = aclFinalize();
        if (aclRet != ACL_SUCCESS) {
            SAMPLE_PRT("finalize acl failed with %d.\n", aclRet);
        }
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

int32_t acl_deinit()
{
    aclError aclRet = aclrtDestroyContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("destroy context failed with %d.", aclRet);
    }
    aclRet = aclrtResetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("reset device(0) failed with %d.\n", aclRet);
    }
    aclRet = aclFinalize();
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("finalize acl failed with %d.\n", aclRet);
    }
    return HI_SUCCESS;
}

int32_t dvpp_mem_malloc(void** addrPtr, uint32_t bufSize)
{
    int32_t ret = hi_mpi_dvpp_malloc(0, addrPtr, bufSize);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("malloc buffer failed, size: %u!\n", bufSize);
    }
    return ret;
}

int32_t dvpp_mem_free(void* addr)
{
    int32_t ret = hi_mpi_dvpp_free(addr);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("free buffer failed, addr = %p!\n", addr);
    }
    return ret;
}

int32_t prepare_input_data(hi_vpc_pic_info& inputPic, const char inputFileName[])
{
    FILE* srcFp = fopen(inputFileName, "rb");
    if (srcFp == nullptr) {
        SAMPLE_PRT("fopen %s failed!\n", inputFileName);
        return HI_FAILURE;
    }

    // malloc input buffer in device
    int32_t ret = dvpp_mem_malloc(&inputPic.picture_address, inputPic.picture_buffer_size);
    if (ret != HI_SUCCESS) {
        fclose(srcFp);
        srcFp = nullptr;
        SAMPLE_PRT("malloc input buffer in device failed!\n");
        return HI_FAILURE;
    }

    if (g_run_mode == ACL_HOST) {
        void* inputAddr = nullptr;
        // malloc input buffer in host
        ret = aclrtMallocHost(&inputAddr, inputPic.picture_buffer_size);
        if (inputAddr == nullptr) {
            SAMPLE_PRT("malloc input buffer in host failed, ret = %d\n", ret);
            fclose(srcFp);
            srcFp = nullptr;
            dvpp_mem_free(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            return ret;
        }
        size_t numread = fread(inputAddr, 1, inputPic.picture_buffer_size, srcFp);
        if (numread < inputPic.picture_buffer_size) {
            SAMPLE_PRT("read input data failed, numread = %zu, data size = %u\n",
                numread, inputPic.picture_buffer_size);
            fclose(srcFp);
            srcFp = nullptr;
            aclrtFreeHost(inputAddr);
            inputAddr = nullptr;
            dvpp_mem_free(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            return HI_FAILURE;
        }

        ret = aclrtMemcpy(inputPic.picture_address, inputPic.picture_buffer_size, inputAddr,
            inputPic.picture_buffer_size, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            SAMPLE_PRT("Copy host memcpy to device failed, ret = %d\n", ret);
            fclose(srcFp);
            srcFp = nullptr;
            aclrtFreeHost(inputAddr);
            inputAddr = nullptr;
            dvpp_mem_free(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            return HI_FAILURE;
        }

        aclrtFreeHost(inputAddr);
        inputAddr = nullptr;
    } else {
        size_t numread = fread(inputPic.picture_address, 1, inputPic.picture_buffer_size, srcFp);
        if (numread < inputPic.picture_buffer_size) {
            SAMPLE_PRT("read input data failed, numread = %zu, data size = %u\n",
                numread, inputPic.picture_buffer_size);
            fclose(srcFp);
            srcFp = nullptr;
            dvpp_mem_free(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            return HI_FAILURE;
        }
    }
    fclose(srcFp);
    srcFp = nullptr;
    return HI_SUCCESS;
}

int32_t handle_output_data(hi_vpc_pic_info& outputPic, uint32_t dstBufferSize, const char outputFileName[])
{
    // copy output width and height configuration
    hi_vpc_pic_info notAlignPic = outputPic;
    hi_vpc_pic_info outputPicHost = outputPic;
    FILE* dstFp = nullptr;
    size_t numWrite = 0;
    if (g_run_mode == ACL_HOST) {
        // we cannot access memory on device, so we malloc memory of the same size at host and copy them
        int32_t ret = aclrtMallocHost(&outputPicHost.picture_address, outputPic.picture_buffer_size);
        if (ret != ACL_SUCCESS) {
            SAMPLE_PRT("malloc output buffer in host failed, ret = %d!\n", ret);
            return HI_FAILURE;
        }
        ret = aclrtMemcpy(outputPicHost.picture_address, outputPic.picture_buffer_size, outputPic.picture_address,
            outputPic.picture_buffer_size, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_SUCCESS) {
            SAMPLE_PRT("Copy memcpy to host failed, ret = %d.\n", ret);
            goto OUT0;
        }
    }

    // malloc real output buffersize
    notAlignPic.picture_address = nullptr;
    notAlignPic.picture_address = malloc(dstBufferSize);
    if (notAlignPic.picture_address == nullptr) {
        SAMPLE_PRT("malloc align buffer failed!");
        goto OUT0;
    }

    // The following function configs stirde, which is the real memory occupied by each line.
    // For example, for yuv 420, width_stride = width. For rgb888, width_stride = width * 3
    configure_stride_and_buffer_size(notAlignPic, 1, 1, false);

    // Due to VPC alignment requirements, the output picture may contain redundant data.
    // The following function writes the real output picture
    if (g_run_mode == ACL_HOST) {
        // write real output picture from outputPicHost to notAlignPic
        get_dst_stride_picture(outputPicHost, notAlignPic);
        // free outputPicHost
        aclrtFreeHost(outputPicHost.picture_address);
        outputPicHost.picture_address = nullptr;
    } else {
        // write real output picture from outputPic to notAlignPic
        get_dst_stride_picture(outputPic, notAlignPic);
    }

    dstFp = fopen(outputFileName, "wb");
    if (dstFp == nullptr) {
        SAMPLE_PRT("fopen %s failed!\n", outputFileName);
        goto OUT1;
    }
    numWrite = fwrite(notAlignPic.picture_address, 1, dstBufferSize, dstFp);
    if (numWrite < dstBufferSize) {
        SAMPLE_PRT("write output data failed, numWrite = %zu, data size = %u\n", numWrite, dstBufferSize);
        goto OUT2;
    }
    fflush(dstFp);
    free(notAlignPic.picture_address);
    notAlignPic.picture_address = nullptr;
    fclose(dstFp);
    dstFp = nullptr;
    return HI_SUCCESS;

OUT2:
    fclose(dstFp);
    dstFp = nullptr;
OUT1:
    free(notAlignPic.picture_address);
    notAlignPic.picture_address = nullptr;
OUT0:
    if (g_run_mode == ACL_HOST) {
        aclrtFreeHost(outputPicHost.picture_address);
        outputPicHost.picture_address = nullptr;
    }
    return HI_FAILURE;
}

void memset_buffer(hi_vpc_pic_info& picInfo)
{
    if (g_run_mode == ACL_HOST) {
        int32_t ret = aclrtMemset(picInfo.picture_address, picInfo.picture_buffer_size, 0,
            picInfo.picture_buffer_size);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("memset_s failed, ret = %d!\n", ret);
        }
    } else {
        memset(picInfo.picture_address, 0, picInfo.picture_buffer_size);
    }
}

uint32_t configure_stride_and_buffer_size(hi_vpc_pic_info& pic, uint32_t widthAlign, uint32_t heightAlign,
    bool widthStride32Align)
{
    if ((widthAlign == 0) || (widthAlign > 128) || ((widthAlign & (widthAlign - 1)) != 0)) { // 最大128
        SAMPLE_PRT("widthAlign = %u, should be power of 2, and between (0, 128]!\n", widthAlign);
        return 0;
    }
    if ((heightAlign == 0) || (heightAlign > 128) || ((heightAlign & (heightAlign - 1)) != 0)) { // 最大128
        SAMPLE_PRT("heightAlign = %u, should be power of 2, and between (0, 128]!\n", heightAlign);
        return 0;
    }

    uint32_t width = pic.picture_width;
    uint32_t height = pic.picture_height;
    uint32_t format = pic.picture_format;
    uint32_t dstBufferSize = 0; // dstBufferSize is the real size
    uint32_t minWidthAlignNum = 32; // min number of width stride is 32

    if (!widthStride32Align) {
        minWidthAlignNum = 1;
    }

    switch (format) {
        case HI_PIXEL_FORMAT_YUV_400:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign);
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            if (pic.picture_width_stride < minWidthAlignNum) {
                pic.picture_width_stride = minWidthAlignNum;
            }
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride;
            dstBufferSize = width * height;
            break;
        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign);
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            if (pic.picture_width_stride < minWidthAlignNum) {
                pic.picture_width_stride = minWidthAlignNum;
            }
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride * 3 / 2; // 3/2 times
            dstBufferSize = width * height * 3 / 2; // the real buffer size is 3/2 times the product of height and width
            break;
        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_440:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_440:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign);
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            if (pic.picture_width_stride < minWidthAlignNum) {
                pic.picture_width_stride = minWidthAlignNum;
            }
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride * 2; // 2 times
            dstBufferSize = width * height * 2; // the real buffer size is 2 times the product of height and width
            break;
        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign);
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            if (pic.picture_width_stride < minWidthAlignNum) {
                pic.picture_width_stride = minWidthAlignNum;
            }
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height * 2; // 2 times
            dstBufferSize = width * height * 2; // the real buffer size is 2 times the product of height and width
            break;
        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_444:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_444:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign);
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            if (pic.picture_width_stride < minWidthAlignNum) {
                pic.picture_width_stride = minWidthAlignNum;
            }
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride * 3; // 3 times
            dstBufferSize = width * height * 3; // the real buffer size is 3 times the product of height and width
            break;
        case HI_PIXEL_FORMAT_YUYV_PACKED_422:
        case HI_PIXEL_FORMAT_UYVY_PACKED_422:
        case HI_PIXEL_FORMAT_YVYU_PACKED_422:
        case HI_PIXEL_FORMAT_VYUY_PACKED_422:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign) * 2; // 2 times
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height;
            dstBufferSize = width * height * 2; // the real buffer size is 2 times the product of height and width
            break;
        case HI_PIXEL_FORMAT_YUV_PACKED_444:
        case HI_PIXEL_FORMAT_RGB_888:
        case HI_PIXEL_FORMAT_BGR_888:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign) * 3; // 3 times
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride;
            dstBufferSize = width * height * 3; // the real buffer size is 3 times the product of height and width
            break;
        case HI_PIXEL_FORMAT_ARGB_8888:
        case HI_PIXEL_FORMAT_ABGR_8888:
        case HI_PIXEL_FORMAT_RGBA_8888:
        case HI_PIXEL_FORMAT_BGRA_8888:
        case HI_PIXEL_FORMAT_FLOAT32:
            pic.picture_width_stride = ALIGN_UP(width, widthAlign) * 4; // 4 times
            pic.picture_height_stride = ALIGN_UP(height, heightAlign);
            pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride;
            dstBufferSize = width * height * 4; // the real buffer size is 4 times the product of height and width
            break;
        default:
            SAMPLE_PRT("unsupport format %u!\n", format);
            pic.picture_buffer_size = 0;
            dstBufferSize = 0;
            break;
    }

    return dstBufferSize;
}

int32_t get_dst_stride_picture(const hi_vpc_pic_info& srcPic, const hi_vpc_pic_info& dstPic)
{
    if (srcPic.picture_format != dstPic.picture_format) {
        SAMPLE_PRT("srcPic.picture_format(%d) should be same with dstPic.picture_format(%d)\n",
            srcPic.picture_format, dstPic.picture_format);
        return HI_FAILURE;
    }

    uint8_t* srcBufY = static_cast<uint8_t*>(srcPic.picture_address);
    uint8_t* dstBufY = static_cast<uint8_t*>(dstPic.picture_address);
    uint8_t* srcBufUV = nullptr;
    uint8_t* dstBufUV = nullptr;

    switch (srcPic.picture_format) {
        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420:
            // copy y component
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width);
            }
            // copy uv component
            srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height_stride;
            dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height_stride;
            for (uint32_t i = 0; i < srcPic.picture_height / 2; ++i) { // 1/2 of height
                memcpy(dstBufUV + i * dstPic.picture_width_stride, srcBufUV + i * srcPic.picture_width_stride,
                    srcPic.picture_width);
            }
            break;

        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422:
            // copy y component
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width);
            }
            // copy uv component
            srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height;
            dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height;
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufUV + i * dstPic.picture_width_stride, srcBufUV + i * srcPic.picture_width_stride,
                    srcPic.picture_width);
            }
            break;

        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_440:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_440:
            // copy y component
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width);
            }
            // copy uv component
            srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height_stride;
            dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height_stride;
            for (uint32_t i = 0; i < srcPic.picture_height / 2; ++i) {
                memcpy(dstBufUV + i * dstPic.picture_width_stride * 2, srcBufUV + i * srcPic.picture_width_stride * 2,
                    srcPic.picture_width * 2);
            }
            break;

        case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_444:
        case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_444:
            // copy y component
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width);
            }
            // copy uv component
            srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height_stride;
            dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height_stride;
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufUV + i * dstPic.picture_width_stride * 2, // 该格式需要乘2
                    srcBufUV + i * srcPic.picture_width_stride * 2, srcPic.picture_width * 2);
            }
            break;

        case HI_PIXEL_FORMAT_YUYV_PACKED_422:
        case HI_PIXEL_FORMAT_UYVY_PACKED_422:
        case HI_PIXEL_FORMAT_YVYU_PACKED_422:
        case HI_PIXEL_FORMAT_VYUY_PACKED_422:
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width * 2);
            }
            break;

        case HI_PIXEL_FORMAT_YUV_PACKED_444:
        case HI_PIXEL_FORMAT_RGB_888:
        case HI_PIXEL_FORMAT_BGR_888:
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width * 3);
            }
            break;

        case HI_PIXEL_FORMAT_ARGB_8888:
        case HI_PIXEL_FORMAT_ABGR_8888:
        case HI_PIXEL_FORMAT_RGBA_8888:
        case HI_PIXEL_FORMAT_BGRA_8888:
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width * 4);
            }
            break;

        case HI_PIXEL_FORMAT_YUV_400:
            for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                    srcPic.picture_width);
            }
            break;

        default:
            SAMPLE_PRT("don't support format %d\n", srcPic.picture_format);
            return HI_FAILURE;
    }

    return HI_SUCCESS;
}

int32_t pre_process(VpcAttr& vpcAttribute)
{
    char inputFileName[FILE_NAME_LEN];
    strcpy(inputFileName, vpcAttribute.inputFileName);
    char outputFileName[FILE_NAME_LEN];
    strcpy(outputFileName, vpcAttribute.outputFileName);

    // 1. config in and out pic's width, height, format, stride, buffer size
    hi_vpc_pic_info inputPic;
    inputPic.picture_width = vpcAttribute.width;;
    inputPic.picture_height = vpcAttribute.height;;
    inputPic.picture_format = static_cast<hi_pixel_format>(vpcAttribute.format);
    hi_vpc_pic_info outputPic = inputPic;
    uint32_t notAlignInBufferSize = configure_stride_and_buffer_size(inputPic, vpcAttribute.inWidthAlign,
        vpcAttribute.inHeightAlign);
    if (notAlignInBufferSize == 0) {
        SAMPLE_PRT("notAlignInBufferSize %u should not be 0!\n", notAlignInBufferSize);
        return HI_FAILURE;
    }

    uint32_t notAlignOutBufferSize = configure_stride_and_buffer_size(outputPic, vpcAttribute.outWidthAlign,
        vpcAttribute.outHeightAlign);
    if (notAlignOutBufferSize == 0) {
        SAMPLE_PRT("notAlignOutBufferSize %u should not be 0!\n", notAlignOutBufferSize);
        return HI_FAILURE;
    }

    // 2. read input pic
    inputPic.picture_address = malloc(inputPic.picture_buffer_size);
    if (inputPic.picture_address == nullptr) {
        SAMPLE_PRT("malloc in buffer failed!\n");
        return HI_FAILURE;
    }

    FILE* srcFp = fopen(inputFileName, "rb");
    if (srcFp == nullptr) {
        SAMPLE_PRT("fopen in file failed!\n");
        free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    size_t numread = fread(inputPic.picture_address, 1, inputPic.picture_buffer_size, srcFp);
    if (numread < inputPic.picture_buffer_size) {
        SAMPLE_PRT("read input data failed, numread = %zu, inputPic.picture_buffer_size = %u!\n",
            numread, inputPic.picture_buffer_size);
        fclose(srcFp);
        srcFp = nullptr;
        free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    outputPic.picture_address = malloc(outputPic.picture_buffer_size);
    if (outputPic.picture_address == nullptr) {
        SAMPLE_PRT("malloc out buffer failed!");
        fclose(srcFp);
        srcFp = nullptr;
        free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    memset(outputPic.picture_address, 0, outputPic.picture_buffer_size);

    // 3. get out pic according to the stride and format
    int32_t transRet = get_dst_stride_picture(inputPic, outputPic);
    if (transRet != 0) {
        SAMPLE_PRT("get_dst_stride_picture failed!");
        fclose(srcFp);
        srcFp = nullptr;
        free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        free(outputPic.picture_address);
        outputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    // 4. write out pic
    FILE* outFp = fopen(outputFileName, "wb+");
    if (outFp == nullptr) {
        SAMPLE_PRT("fopen out file failed");
        fclose(srcFp);
        srcFp = nullptr;
        free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        free(outputPic.picture_address);
        outputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    size_t numWrite = fwrite(outputPic.picture_address, 1, outputPic.picture_buffer_size, outFp);
    if (numWrite < outputPic.picture_buffer_size) {
        SAMPLE_PRT("write output data failed, numWrite = %zu, outputPic.picture_buffer_size = %u\n",
            numWrite, outputPic.picture_buffer_size);
        fclose(srcFp);
        srcFp = nullptr;
        fclose(outFp);
        outFp = nullptr;
        free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        free(outputPic.picture_address);
        outputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    fclose(srcFp);
    srcFp = nullptr;
    fclose(outFp);
    outFp = nullptr;
    free(inputPic.picture_address);
    inputPic.picture_address = nullptr;
    free(outputPic.picture_address);
    outputPic.picture_address = nullptr;
    return 0;
}