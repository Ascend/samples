#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>
#include "atlas_utils.h"
#include "face_detection.h"
#include "face_detection_inference.h"
#include "face_detection_postprocess.h"

using namespace std;

int AppMainProcess(uint32_t msgId, std::shared_ptr<void> msgData, void* userData);

int main() {  
    AtlasApp& app = CreateAtlasAppInstance();
    if (STATUS_OK != app.Init()) {
        ASC_LOG_ERROR("atlas app init failed");
        return STATUS_ERROR;
    }

    Camera cameracap(0);
    // return num
	int thId = app.CreateAtlasThread(&cameracap);
    if (thId < 0) {
        ASC_LOG_ERROR("Create camera thread failed");
        return STATUS_ERROR;
    }

    cameracap.myThreadId = thId;
    printf("create camera thread ok, thread id %d\n",  cameracap.myThreadId );

    Resolution modelRes = { 416, 416 };


    FaceDetectionInference infer(modelRes);
    cameracap.inferThreadId = app.CreateAtlasThread(&infer);
    printf("create inference thread ok, thread id %d\n", cameracap.inferThreadId);

    FaceDetectionPostprocess postprocess;
    thId = app.CreateAtlasThread(&postprocess, "postprocess");

    SendMessage(cameracap.myThreadId, MSG_START_READ_VIDEO, nullptr);
    app.Wait(AppMainProcess, nullptr);
    
    ASC_LOG_INFO("App exit!");
}

int AppMainProcess(uint32_t msgId, std::shared_ptr<void> msgData, void* userData) {
    switch (msgId) {
        case MSG_VIDEO_DECODE_FINISH:
        {
            AtlasApp& app = GetAtlasAppInstance();
            app.Exit();
            return STATUS_ERROR;
        }
        default:
            break;
    }

    return STATUS_OK;
}
