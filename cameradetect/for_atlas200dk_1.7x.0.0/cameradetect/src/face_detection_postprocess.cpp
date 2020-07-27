#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <vector>
#include <thread>
#include "atlas_utils.h"
#include "face_detection.h"
#include "face_detection_postprocess.h"

using namespace std;
using namespace ascend::presenter;

namespace {
enum BBoxIndex { TopLeftX = 0, TopLeftY, LowerRigltX, LowerRightY, SCORE, LABEL };

const static std::vector<string> yoloLable = { "background", "person", "bicycle", "car", "motorbike",
    "aeroplane","bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag","tie",
    "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana",
        "apple", "sandwich", "orange", "broccoli", "carrot",
        "hot dog", "pizza", "donut", "cake", "chair",
        "sofa", "potted plant", "bed", "dining table", "toilet",
        "TV monitor", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase","scissors",
        "teddy bear", "hair drier", "toothbrush" };

const uint32_t kFasterYolov3OutputsNum = 2;
const uint32_t kPredictItemLen = 16;
const uint32_t kScorePercent = 100;
}

FaceDetectionPostprocess::FaceDetectionPostprocess(){

}

FaceDetectionPostprocess::~FaceDetectionPostprocess(){
    //chan_ = nullptr;
}

int FaceDetectionPostprocess::Init() {
    Channel* chan = nullptr;
    PresenterErrorCode ret = OpenChannelByConfig(chan, "../data/face_detection.conf");
    if (ret != PresenterErrorCode::kNone) {
        ASC_LOG_ERROR("Open channel failed, error %d", (int)ret);
        return STATUS_ERROR;
    }
    chan_.reset(chan);

    return STATUS_OK;
}

float scalx = 1280.0/416.0;
float scaly = 720/416.0;
void FaceDetectionPostprocess::FasterYolov3PostProcess(vector<DetectionResult>& detectionResults,
                             DetectionData* detectResult) {
    std::vector<DataBuffer> outputs = detectResult->output;

    if (outputs.size() != kFasterYolov3OutputsNum) {
        ASC_LOG_ERROR("Detection output size does not match.");
        return;
    }
    uint32_t imageWidth = detectResult->image.width;
    uint32_t imageHeight = detectResult->image.height;
    float *boxNum = reinterpret_cast<float *>(outputs[1].data.get());
    uint32_t num = (uint32_t)(boxNum[0]);
    float *objectInfo = reinterpret_cast<float *>(outputs[0].data.get());

    for (uint32_t i = 0; i < num; i++) {
		float score = objectInfo[SCORE * num + i];
		if (score < 0.7)
            continue;    

        //Detection result
        DetectionResult oneResult;
        // left top
        Point point_lt, point_rb;
        point_lt.x = objectInfo[TopLeftX * num + i]*scalx;
        point_lt.y = objectInfo[TopLeftY * num + i]*scaly;
        // right bottom
        point_rb.x = objectInfo[LowerRigltX * num + i]*scalx;
        point_rb.y = objectInfo[LowerRightY * num + i]*scaly;

        uint32_t object = objectInfo[LABEL * num + i];

        oneResult.lt = point_lt;
        oneResult.rb = point_rb;
        oneResult.result_text = yoloLable[object + 1] + ":" + 
                                to_string(score * kScorePercent) + "%";
        // push back
        detectionResults.emplace_back(oneResult);
		ASC_LOG_INFO("object: area %d, %d, %d, %d, text: %s", 
			         point_lt.x, point_lt.y, point_rb.x, 
                     point_rb.y, oneResult.result_text.c_str());
    }
}

int FaceDetectionPostprocess::SendImage(Channel* channel, 
                                        ImageData& jpegImage,
                                        vector<DetectionResult>& detRes) {
    ImageFrame frame;
    frame.format = ImageFormat::kJpeg;
    frame.width = jpegImage.width;
    frame.height = jpegImage.height;
    frame.size = jpegImage.size;
    frame.data = jpegImage.data.get();
    frame.detection_results = detRes;

    PresenterErrorCode ret = PresentImage(channel, frame);
    // send to presenter failed
    if (ret != PresenterErrorCode::kNone) {
        ASC_LOG_ERROR("Send JPEG image to presenter failed, error %d", (int)ret);                        
        return STATUS_ERROR;
    }

    printf("Send JPEG image to presenter success, ret %d\n", (int)ret);
    return STATUS_OK;
}

int FaceDetectionPostprocess::Process(int msgId, std::shared_ptr<void> msgData) {
    switch (msgId) {
        case MSG_INFERENCE_RESULT:
        {
            shared_ptr<DetectionData> inferData = static_pointer_cast<DetectionData>(msgData);
            vector<DetectionResult> detectionResults;
            FasterYolov3PostProcess(detectionResults, inferData.get());
            SendImage(chan_.get(), inferData->image, detectionResults);
            break;
        }
        case MSG_VIDEO_DECODE_FINISH:
        {
            ASC_LOG_INFO("All frame process finished, notify app to exit!");
            SendMessage(0, MSG_VIDEO_DECODE_FINISH, nullptr);
            return STATUS_ERROR;
        }
        default:
            ASC_LOG_ERROR("Post process thread received unkonow message %d", msgId);
            break;
    }

    return STATUS_OK;
}

