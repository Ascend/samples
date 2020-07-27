#ifndef _ATLAS_UTILS_H_
#define _ATLAS_UTILS_H_

#include "camera.h"
#include "model_process.h"
#include "thread_safe_queue.h"
//#include "video_decode.h"
#include "dvpp_process.h"
#include "atlas_app.h"

using namespace std;

AtlasApp& CreateAtlasAppInstance();
AtlasApp& GetAtlasAppInstance();
int SendMessage(int dest, int msgId, std::shared_ptr<void> data);
int GetAtlasThreadIdByName(const string& threadName);
#endif

