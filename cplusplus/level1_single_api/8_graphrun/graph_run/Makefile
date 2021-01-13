ASCEND_PATH := /usr/local/Ascend

LOCAL_DIR := ./
FWK_INCLUDE_DIR := $(ASCEND_PATH)/fwkacllib/include

LOCAL_MODULE_NAME := graph_run
CC := g++
CFLAGS := -std=c++11 -g -Wall -D_GLIBCXX_USE_CXX11_ABI=0
SRCS := $(wildcard $(LOCAL_DIR)/src/*.cpp)

INCLUDES := -I $(ASCEND_PATH)/opp/op_proto/built-in/inc \
            -I $(FWK_INCLUDE_DIR)/graph \
            -I $(FWK_INCLUDE_DIR)/ge \
	    -I $(ASCEND_PATH)/fwkacllib/include \
            -I $(LOCAL_DIR) \

LIBS := -L ${ASCEND_PATH}/fwkacllib/lib64/stub \
    -lgraph \
    -lge_runner \

graph_run:
	mkdir -p out
	$(CC) $(SRCS) $(INCLUDES) $(LIBS) $(CFLAGS) -o ./out/$(LOCAL_MODULE_NAME)
clean:


