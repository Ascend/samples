/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File utils.h
* Description: handle file operations
*/
#pragma once

#include <unistd.h>

typedef int AtlasError;

const int ATLAS_OK = 0;
const int ATLAS_ERROR = 1;
const int ATLAS_ERROR_INVALID_ARGS = 2;
const int ATLAS_ERROR_SET_ACL_CONTEXT = 3;
const int ATLAS_ERROR_GET_ACL_CONTEXT = 4;
const int ATLAS_ERROR_CREATE_ACL_CONTEXT = 5;
const int ATLAS_ERROR_CREATE_THREAD = 6;
const int ATLAS_ERROR_CREATE_STREAM = 7;
const int ATLAS_ERROR_GET_RUM_MODE = 8;
const int ATLAS_ERROR_APP_INIT = 9;
const int ATLAS_ERROR_DEST_INVALID = 10;
const int ATLAS_ERROR_INITED_ALREADY = 11;
const int ATLAS_ERROR_ENQUEUE = 12;
const int ATLAS_ERROR_WRITE_FILE = 13;
const int ATLAS_ERROR_THREAD_ABNORMAL = 14;
const int ATLAS_ERROR_START_THREAD = 15;
const int ATLAS_ERROR_ADD_THREAD = 16;

//malloc or new memory failed
const int ATLAS_ERROR_MALLOC = 101;
//aclrtMalloc failed
const int ATLAS_ERROR_MALLOC_DEVICE = 102;

const int ATLAS_ERROR_MALLOC_DVPP = 103;

//access file failed
const int ATLAS_ERROR_ACCESS_FILE = 201;
//the file is invalid
const int ATLAS_ERROR_INVALID_FILE = 202;
//open file failed
const int ATLAS_ERROR_OPEN_FILE = 203;

//load model repeated
const int ATLAS_ERROR_LOAD_MODEL_REPEATED = 301;

const int ATLAS_ERROR_NO_MODEL_DESC = 302;
//load mode by acl failed
const int ATLAS_ERROR_LOAD_MODEL = 303;

const int ATLAS_ERROR_CREATE_MODEL_DESC = 304;

const int ATLAS_ERROR_GET_MODEL_DESC = 305;

const int ATLAS_ERROR_CREATE_DATASET = 306;

const int ATLAS_ERROR_CREATE_DATA_BUFFER = 307;

const int ATLAS_ERROR_ADD_DATASET_BUFFER = 308;

const int ATLAS_ERROR_EXECUTE_MODEL = 309;

const int ATLAS_ERROR_GET_DATASET_BUFFER = 310;

const int ATLAS_ERROR_GET_DATA_BUFFER_ADDR = 311;

const int ATLAS_ERROR_GET_DATA_BUFFER_SIZE = 312;

const int ATLAS_ERROR_COPY_DATA = 313;

const int ATLAS_ERROR_SET_CAMERA = 400;

const int ATLAS_ERROR_CAMERA_NO_ACCESSABLE = 401;

const int ATLAS_ERROR_OPEN_CAMERA = 402;

const int ATLAS_ERROR_READ_CAMERA_FRAME = 403;

const int ATLAS_ERROR_UNSURPPORT_PROPERTY = 404;

const int ATLAS_ERROR_INVALID_PROPERTY_VALUE = 405;

const int ATLAS_ERROR_UNSURPPORT_VIDEO_CAPTURE =406;

const int ATLAS_ERROR_CREATE_DVPP_CHANNEL_DESC = 501;

const int ATLAS_ERRROR_CREATE_DVPP_CHANNEL = 502;

const int ATLAS_ERROR_CREATE_PIC_DESC = 503;

const int ATLAS_ERROR_CREATE_RESIZE_CONFIG = 504;

const int ATLAS_ERROR_RESIZE_ASYNC = 505;

const int ATLAS_ERROR_SYNC_STREAM = 506;

const int ATLAS_ERROR_JPEGE_ASYNC = 507;

const int ATLAS_ERROR_JPEGD_ASYNC = 508;

const int ATLAS_ERROR_FFMPEG_DECODER_INIT = 601;

const int ATLAS_ERROR_OPEN_VIDEO_UNREADY = 602;

const int ATLAS_ERROR_TOO_MANY_VIDEO_DECODERS = 603;

const int ATLAS_ERROR_SET_VDEC_CHANNEL_ID = 604;

const int ATLAS_ERROR_SET_STREAM_DESC_DATA = 605;

const int ATLAS_ERROR_SET_VDEC_CHANNEL_THREAD_ID = 606;

const int ATLAS_ERROR_SET_VDEC_CALLBACK = 607;

const int ATLAS_ERROR_SET_VDEC_ENTYPE = 608;

const int ATLAS_ERROR_SET_VDEC_PIC_FORMAT = 609;

const int ATLAS_ERROR_CREATE_VDEC_CHANNEL = 610;

const int ATLAS_ERROR_CREATE_STREAM_DESC = 611;

const int ATLAS_ERROR_SET_STREAM_DESC_EOS = 612;

const int ATLAS_ERROR_SET_STREAM_DESC_SIZE = 613;

const int ATLAS_ERROR_SET_PIC_DESC_DATA = 614;

const int ATLAS_ERROR_SET_PIC_DESC_SIZE = 615;

const int ATLAS_ERROR_SET_PIC_DESC_FORMAT = 616;

const int ATLAS_ERROR_VDEC_IS_EXITTING = 617;

const int ATLAS_ERROR_VDEC_SET_WIDTH = 618;

const int ATLAS_ERROR_VDEC_WIDTH_INVALID = 619;

const int ATLAS_ERROR_VDEC_HEIGHT_INVALID = 620;

const int ATLAS_ERROR_VDEC_SET_HEIGHT = 621;

const int ATLAS_ERROR_VDEC_ENTYPE_INVALID = 622;

const int ATLAS_ERROR_VDEC_FORMAT_INVALID = 623;

const int ATLAS_ERROR_VDEC_INVALID_PARAM = 624;

const int ATLAS_ERROR_VDEC_SEND_FRAME = 625;

const int ATLAS_ERROR_VDEC_QUEUE_FULL = 626;

const int ATLAS_ERROR_SET_RTSP_TRANS = 627;

const int ATLAS_ERROR_READ_EMPTY = 628;

const int ATLAS_ERROR_VIDEO_DECODER_STATUS = 629;

const int ATLAS_ERROR_DECODE_FINISH = 630;

const int ATLAS_ERROR_H26X_FRAME = 631;






