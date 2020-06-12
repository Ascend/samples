/**
* @file main.h
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-6-13
*/
#ifndef INC_DATA_RECV_H_
#define INC_DATA_RECV_H_
#include <hiaiengine/api.h>
#include <string>
class CustomDataRecvInterface : public hiai::DataRecvInterface
{
 public:
    /**
    * @ingroup FasterRcnnDataRecvInterface
    * @brief init
    * @param [in]desc:std::string
    */
    CustomDataRecvInterface(const std::string& filename) :
        file_name_(filename) {}

    /**
    * @ingroup FasterRcnnDataRecvInterface
    * @brief RecvData RecvData
    * @param [in]
    */
    HIAI_StatusT RecvData(const std::shared_ptr<void>& message);

 private:
    std::string file_name_;
};
#endif  // INC_DATA_RECV_H_
