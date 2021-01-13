ifeq (${ASCEND_INSTALL_PATH},)
    ASCEND_PATH := /usr/local/Ascend
else
    ASCEND_PATH := ${ASCEND_INSTALL_PATH}
endif


LOCAL_DIR  := ./
ATC_INCLUDE_DIR := $(ASCEND_PATH)/atc/include
FWK_INCLUDE_DIR := $(ASCEND_PATH)/fwkacllib/include
OPP_INCLUDE_DIR := $(ASCEND_PATH)/opp/op_proto/built-in/inc

LOCAL_MODULE_NAME := ir_build
LOCAL_FWK_MODULE_NAME := fwk_ir_build
CC := g++
CFLAGS := -std=c++11 -g -Wall -D_GLIBCXX_USE_CXX11_ABI=0
SRCS := $(wildcard $(LOCAL_DIR)/src/main.cpp)

INCLUDES := -I $(ASCEND_PATH)/opp/op_proto/built-in/inc \
            -I $(ATC_INCLUDE_DIR)/graph \
            -I $(ATC_INCLUDE_DIR)/ge \
            -I $(ATC_INCLUDE_DIR)/parser \
            -I $(ASCEND_PATH)/atc/include \

FWK_INCLUDES := -I $(ASCEND_PATH)/opp/op_proto/built-in/inc \
            -I $(FWK_INCLUDE_DIR)/graph \
            -I $(FWK_INCLUDE_DIR)/ge \
            -I $(FWK_INCLUDE_DIR)/parser \
            -I $(ASCEND_PATH)/fwkacllib/include \

LIBS := -L ${ASCEND_PATH}/atc/lib64/stub \
    -lgraph \
    -lge_compiler \
    -lfmk_parser \

FWK_LIBS := -L ${ASCEND_PATH}/fwkacllib/lib64/stub \
    -lgraph \
    -lge_runner \
    -lfmk_parser \

ir_build:
	mkdir -p out
	$(CC) $(SRCS) $(INCLUDES) $(LIBS) $(CFLAGS) -o ./out/$(LOCAL_MODULE_NAME)

fwk_ir_build:
	mkdir -p out
	$(CC) $(SRCS) $(FWK_INCLUDES) $(FWK_LIBS) $(CFLAGS) -o ./out/$(LOCAL_FWK_MODULE_NAME)

clean:
