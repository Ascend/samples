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
    // 对应输入图片的宽高 （在引擎文件中通过opencv获取）
    image_handle->image_info.width = imageinfo->width;
    image_handle->image_info.height = imageinfo->height;
  
    image_handle->image_info.size = data_len;
    // 指定析构器
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

