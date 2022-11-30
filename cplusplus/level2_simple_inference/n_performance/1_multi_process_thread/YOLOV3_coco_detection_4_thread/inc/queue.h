/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef YOLOV3_COCO_DETECTION_4_THREAD_INC_QUEUE_H
#define YOLOV3_COCO_DETECTION_4_THREAD_INC_QUEUE_H

#include <condition_variable>
#include <list>
#include <locale>
#include <mutex>
#include <stdint.h>
#include "opencv2/opencv.hpp"

#include "opencv2/imgproc/types_c.h"
struct message_pre {
    int videoIndex;
    int isLastFrame;
    cv::Mat frame;
    cv::Mat reiszeMat;
};

struct message {
    int videoIndex;
    int isLastFrame;
    cv::Mat frame;
    std::shared_ptr<void> detectData;
    std::shared_ptr<void> boxNum;
};

struct message_video {
    int number;
    cv::Mat resultImage;
};

static const int APP_ERR_QUEUE_STOPED = 1;
static const int APP_ERR_QUEUE_EMPTY = 2;
static const int APP_ERROR_QUEUE_FULL = 3;
static const int DEFAULT_MAX_QUEUE_SIZE = 128;

template <typename T> class Queue {
public:
    Queue(uint32_t maxSize = DEFAULT_MAX_QUEUE_SIZE) : maxSize_(maxSize), isStoped_(false) {}

    ~Queue() {}

    int Pop(T &item)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        while (queue_.empty() && !isStoped_) {
            emptyCond_.wait(lock);
        }

        if (isStoped_) {
            return APP_ERR_QUEUE_STOPED;
        }

        if (queue_.empty()) {
            return APP_ERR_QUEUE_EMPTY;
        } else {
            item = queue_.front();
            queue_.pop_front();
        }

        fullCond_.notify_one();

        return 0;
    }

    int Pop(T& item, unsigned int timeOutMs)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        auto realTime = std::chrono::milliseconds(timeOutMs);

        while (queue_.empty() && !isStoped_) {
            emptyCond_.wait_for(lock, realTime);
        }

        if (isStoped_) {
            return APP_ERR_QUEUE_STOPED;
        }

        if (queue_.empty()) {
            return APP_ERR_QUEUE_EMPTY;
        } else {
            item = queue_.front();
            queue_.pop_front();
        }

        fullCond_.notify_one();

        return 0;
    }

    int Push(const T& item, bool isWait = false)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        while (queue_.size() >= maxSize_ && isWait && !isStoped_) {
            fullCond_.wait(lock);
        }

        if (isStoped_) {
            return 1;
        }

        if (queue_.size() >= maxSize_) {
            return APP_ERROR_QUEUE_FULL;
        }
        queue_.push_back(item);

        emptyCond_.notify_one();

        return 0;
    }

    int Push_Front(const T &item, bool isWait = false)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        while (queue_.size() >= maxSize_ && isWait && !isStoped_) {
            fullCond_.wait(lock);
        }

        if (isStoped_) {
            return APP_ERR_QUEUE_STOPED;
        }

        if (queue_.size() >= maxSize_) {
            return APP_ERROR_QUEUE_FULL;
        }

        queue_.push_front(item);

        emptyCond_.notify_one();

        return 0;
    }

    void Stop()
    {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            isStoped_ = true;
        }

        fullCond_.notify_all();
        emptyCond_.notify_all();
    }

    void Restart()
    {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            isStoped_ = false;
        }
    }

    // if the queue is stoped ,need call this function to release the unprocessed items
    std::list<T> GetRemainItems()
    {
        std::unique_lock<std::mutex> lock(mutex_);

        if (!isStoped_) {
            return std::list<T>();
        }

        return queue_;
    }

    int GetBackItem(T &item)
    {
        if (isStoped_) {
            return APP_ERR_QUEUE_STOPED;
        }

        if (queue_.empty()) {
            return APP_ERR_QUEUE_EMPTY;
        }

        item = queue_.back();
        return 0;
    }

    std::mutex *GetLock()
    {
        return &mutex_;
    }

    int IsFull()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.size() >= maxSize_;
    }

    int GetSize()
    {
        return queue_.size();
    }

    int IsEmpty()
    {
        return queue_.empty();
    }

    void Clear()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.clear();
    }

private:
    std::list<T> queue_;
    std::mutex mutex_;
    std::condition_variable emptyCond_;
    std::condition_variable fullCond_;
    uint32_t maxSize_;

    bool isStoped_;
};

#endif