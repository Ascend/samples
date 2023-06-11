
#include "AclLiteApp.h"
#include "pushrtspthread.h"
using namespace cv;
using namespace std;
namespace {
    uint32_t kResizeWidth = 600;
    uint32_t kResizeHeight = 400;
    uint32_t kBgrMultiplier = 3;
}

PushRtspThread::PushRtspThread(std::string rtspUrl)
{
    g_rtspUrl = rtspUrl;
    ACLLITE_LOG_INFO("PushRtspThread URL : %s", g_rtspUrl.c_str());
}

PushRtspThread::~PushRtspThread()
{
    g_picToRtsp.FlushEncoder();
}

AclLiteError PushRtspThread::Init() {
    g_frameSeq = 0;
    XInitThreads();
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
        DisplayMsgProcess(static_pointer_cast<DetectDataMsg>(msgData));
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

AclLiteError PushRtspThread::DisplayMsgProcess(std::shared_ptr<DetectDataMsg> detectDataMsg) {
    if (detectDataMsg->isLastFrame) {
        if(av_log_get_level() != AV_LOG_ERROR) {
            av_log_set_level(AV_LOG_INFO);
        }
        for(int i = 0; i < detectDataMsg->frame.size(); i++) {
            cv::Mat resized_image;
            cv::resize(detectDataMsg->frame[i], resized_image, cv::Size(kResizeWidth, kResizeHeight));

            g_picToRtsp.BgrDataToRtsp(resized_image.data, resized_image.cols * resized_image.rows * kBgrMultiplier, g_frameSeq++);
        }
        SendMessage(detectDataMsg->rtspDisplayThreadId, MSG_ENCODE_FINISH, nullptr);
        return ACLLITE_OK;
    }
    if(av_log_get_level() != AV_LOG_ERROR) {
        av_log_set_level(AV_LOG_INFO);
    }
    for(int i = 0; i < detectDataMsg->frame.size(); i++) {
        cv::Mat resized_image;
        cv::resize(detectDataMsg->frame[i], resized_image, cv::Size(kResizeWidth, kResizeHeight));

        g_picToRtsp.BgrDataToRtsp(resized_image.data, resized_image.cols * resized_image.rows * kBgrMultiplier, g_frameSeq++);
    }
    return ACLLITE_OK;
}