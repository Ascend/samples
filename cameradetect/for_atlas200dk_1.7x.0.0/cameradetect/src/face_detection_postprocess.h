#ifndef _FACE_DETECTION_POSTPROCESS_
#define _FACE_DETECTION_POSTPROCESS_
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <memory>
#include "atlas_utils.h"
#include "presenter/agent/presenter_channel.h"

using namespace std;
using namespace ascend::presenter;

class FaceDetectionPostprocess:public AtlasThread {
public:
	FaceDetectionPostprocess();
	~FaceDetectionPostprocess();
	int Init();
	int Process(int msgId, shared_ptr<void> msgData);
	int SendImage(Channel * channel,ImageData& jpegImage,		
		          vector<DetectionResult>& detRes);
        void FasterYolov3PostProcess(vector<DetectionResult>& detectionResults,
                             DetectionData* detectResult);
private:
	std::shared_ptr<Channel> chan_;
};

#endif
