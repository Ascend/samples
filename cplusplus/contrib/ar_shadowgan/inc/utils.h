#pragma once
#include <iostream>
#include <vector>
#include "acl/acl.h"

#include "opencv2/opencv.hpp"
#include <memory>
#include "opencv2/imgproc/types_c.h"

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stderr, "[ERROR]  " fmt "\n", ##args)
#define RGBU8_IMAGE_SIZE(width, height) ((width) * (height) * 3)
#define GRAY_IMAGE_SIZE(width, height) ((width) * (height) * 1)

template<class Type>
std::shared_ptr<Type> MakeSharedNoThrow() {
    try {
        return std::make_shared<Type>();
    }
    catch (...) {
        return nullptr;
    }
}

#define MAKE_SHARED_NO_THROW(memory, memory_type) \
    do { \
            memory = MakeSharedNoThrow<memory_type>(); \
    }while(0);

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
}Result;

struct ImageData {
    uint32_t width = 0;
    uint32_t height = 0;
    int32_t size = 0;
    void* data;
};


