/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */

#include <iostream>
#include <stdio.h>
#include "data_type.h"

/**
 * @brief: serialize for ImageInfo
 */
template<class Archive>
void serialize(Archive& ar, ImageInfo& data) {
  ar(data.path);
  ar(data.width);
  ar(data.height);
  ar(data.size);
  if (data.size > 0 && data.data.get() == nullptr) {
    data.data.reset(new u_int8_t[data.size]);
  }
  ar(cereal::binary_data(data.data.get(), data.size * sizeof(u_int8_t)));
}

//HIAI_REGISTER_DATA_TYPE("ImageInfo",ImageInfo);



/**
 * @brief: serialize for Output
 */
template<class Archive>
void serialize(Archive& ar, Output& data) {
  ar(data.size);
  if (data.size > 0 && data.data.get() == nullptr) {
    data.data.reset(new u_int8_t[data.size]);
  }
  ar(cereal::binary_data(data.data.get(), data.size * sizeof(u_int8_t)));
}

//HIAI_REGISTER_DATA_TYPE("Output",Output);

/**
 * @brief: serialize for EngineTrans
 */
template<class Archive>
void serialize(Archive& ar, ErrorInferenceMsg& data) {
  ar(data.error, data.err_msg);
}

//HIAI_REGISTER_DATA_TYPE("ErrorInferenceMsg",ErrorInferenceMsg);


//The new version of serialize function

void GetEvbImageInfoSearPtr(void *input_ptr, std::string& ctrl_str, uint8_t*& data_ptr, uint32_t& data_len) {
    if (input_ptr == nullptr) {
        return;
    }
    EvbImageInfo* imageinfo = (EvbImageInfo*)input_ptr;

    data_ptr = (uint8_t*)imageinfo->data;
    data_len = imageinfo->size;

    std::ostringstream outputStr;
    cereal::PortableBinaryOutputArchive archive(outputStr);
    archive((*imageinfo));

    
    ctrl_str = outputStr.str();
}


//The new version of deserialize function


std::shared_ptr<void> GetEvbImageInfoDearPtr(const char* ctrl_ptr, const uint32_t& ctr_len, const uint8_t* data_ptr, const uint32_t& data_len) {
    if (ctrl_ptr == nullptr) {
        return nullptr;
    }
    std::shared_ptr<EvbImageInfo> imageinfo = std::make_shared<EvbImageInfo>(); 

    std::istringstream inputStream(std::string(ctrl_ptr, ctr_len));
    cereal::PortableBinaryInputArchive archive(inputStream);
    archive((*imageinfo));

    std::shared_ptr<EngineTrans> image_handle = std::make_shared<EngineTrans>(); 
    if (true == imageinfo->is_finished) {
        image_handle->is_finished = true;
        return image_handle;
    }
	  image_handle->format = imageinfo->format;
	  image_handle->channel_id = imageinfo->channel_id;
    image_handle->image_info.path = imageinfo->path;
    // Corresponding to the width and height of the input image (obtained through opencv in the engine file)
    image_handle->image_info.width = imageinfo->width;
    image_handle->image_info.height = imageinfo->height;
  
    image_handle->image_info.size = data_len;
    // Specify the destructor
    image_handle->image_info.data.reset((uint8_t*)data_ptr, hiai::Graph::ReleaseDataBuffer);
    return std::static_pointer_cast<void>(image_handle);
}

HIAI_REGISTER_SERIALIZE_FUNC("EvbImageInfo", EvbImageInfo, GetEvbImageInfoSearPtr, GetEvbImageInfoDearPtr);

/**
 * @brief: serialize for EvbImageInfo
 */

template <class Archive>
void serialize(Archive & ar, EvbImageInfo& data){
  ar(data.format, data.channel_id, data.width, data.height, 
	 data.is_finished, data.path);
}


/**
 * @brief: serialize for EngineTrans
 */
template<class Archive>
void serialize(Archive& ar, EngineTrans& data) {
  ar(data.format, data.channel_id, data.inference_width, data.inference_height,
  data.image_info, data.err_msg, data.inference_res, data.is_finished);
}

HIAI_REGISTER_DATA_TYPE("EngineTrans",EngineTrans);

