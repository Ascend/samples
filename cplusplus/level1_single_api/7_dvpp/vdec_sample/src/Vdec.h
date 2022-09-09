/**
 *  Copyright [2021] Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */

#ifndef VDEC_H__
#define VDEC_H__
#include <cstdint>
#include "hi_dvpp.h"

int32_t get_option(int32_t argc, char **argv);
int32_t check_option();
void print_parameter();
int32_t vdec_create();
void vdec_destroy();
int32_t create_send_stream_thread();
int32_t create_get_pic_thread();
void stop_send_stream_thread();
void stop_get_pic_thread();
void* send_stream(void* const chanNum);
void* get_pic(void* const chanNum);
void get_every_frame(int32_t chanId, uint8_t* const inputFileBuf, uint32_t* const frameCount, uint32_t fileSize,
    hi_payload_type type, uint8_t* devData);
void delay_exec(uint64_t execTime, int32_t seconds);
void wait_vdec_end();
void show_decode_performance();
void get_start_time();
void init_outbuffer_lock();
void destroy_outbuffer_lock();
void release_outbuffer(uint32_t chanId);
int32_t hi_dvpp_init();
void hi_dvpp_deinit();
#endif // VDEC_H__