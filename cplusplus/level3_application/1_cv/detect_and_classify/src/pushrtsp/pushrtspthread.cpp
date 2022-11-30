
#include "AclLiteApp.h"
#include "pushrtspthread.h"
using namespace cv;
using namespace std;
namespace {
    uint32_t kResizeWidth = 600;
    uint32_t kResizeHeight = 400;
}

PushRtspThread::PushRtspThread(std::string rtspUrl)
{
    g_rtspUrl = rtspUrl;
    ACLLITE_LOG_INFO("PushRtspThread URL : %s", g_rtspUrl.c_str());
}

PushRtspThread::~PushRtspThread()
{
}

AclLiteError PushRtspThread::Init() {
    g_frameSeq = 0;
    AclLiteError ret = g_picToRtsp.AvInit(kResizeWidth, kResizeHeight, g_rtspUrl);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("AvInit rtsp failed");
        return ACLLITE_ERROR;
    }
	g_picToRtsp.BgrDataInint();
    return ACLLITE_OK;
}

AclLiteError PushRtspThread::Process(int msgId, std::shared_ptr<void> msgData) {
    switch(msgId) {
    case MSG_RTSP_DISPLAY:
        DisplayMsgProcess(static_pointer_cast<CarDetectDataMsg>(msgData));
        break;
    case MSG_ENCODE_FINISH:
        SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
        break;
    default:
        ACLLITE_LOG_INFO("Present agent display thread ignore msg %d", msgId);
        break;
    }
    return ACLLITE_OK;
}

AclLiteError PushRtspThread::DisplayMsgProcess(std::shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
    if (carDetectDataMsg->isLastFrame == 1) {
        g_picToRtsp.FlushEncoder();
        SendMessage(carDetectDataMsg->rtspDisplayThreadId, MSG_ENCODE_FINISH, nullptr);
        return ACLLITE_OK;
    }
    if(av_log_get_level() != AV_LOG_ERROR) {
        av_log_set_level(AV_LOG_INFO);
    }
    
    cv::Mat resized_image;
    cv::resize(carDetectDataMsg->frame, resized_image, cv::Size(kResizeWidth, kResizeHeight));

    g_picToRtsp.BgrDataToRtsp(resized_image.data, resized_image.cols * resized_image.rows * 3, g_frameSeq++);
    return ACLLITE_OK;
}