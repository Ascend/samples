TOPDIR      := $(patsubst %,%,$(CURDIR))

ifndef DDK_PATH
$(error "Can not find DDK_PATH env, please set it in environment!.")
endif

ifeq ($(mode),)
mode=AtlasDK
endif

ifeq ($(mode), AtlasDK)
CC := aarch64-linux-gnu-g++
arch := arm
NPU_HOST_LIB = $(HOME)/Ascend/ascend-toolkit/20.1.rc1/arm64-linux/acllib/lib64/stub/
else ifeq ($(mode), ASIC)
CC := g++
arch := x86
NPU_HOST_LIB = $(HOME)/Ascend/ascend-toolkit/20.1.rc1/x86_64-linux/acllib/lib64/stub/
else
$(error "Unsupported mode: "$(mode)", please input: AtlasDK or ASIC.")
endif

LOCAL_MODULE_NAME := libatlasutil.so

LOCAL_DIR  := .
OUT_DIR = ./out/$(arch)
OBJ_DIR = $(OUT_DIR)/obj
DEPS_DIR  = $(OUT_DIR)/deps
LOCAL_LIBRARY=$(OUT_DIR)/$(LOCAL_MODULE_NAME)
RPATH_DIR=/home/HwHiAiUser/ascend_ddk/$(arch)/lib

INC_DIR = \
    -I./include \
    -I$(HOME)/Ascend/driver/ \
	-I$(HOME)/ascend_ddk/$(arch)/include/ \
	-I$(HOME)/ascend_ddk/$(arch)/include/ascenddk/ \
	-I$(HOME)/ascend_ddk/$(arch)/include/ascenddk/presenter/agent/ \
	-I$(DDK_PATH)/acllib/include/ \
	-I$(DDK_PATH)/atc/include/protobuf

CC_FLAGS := $(INC_DIR) -DENABLE_DVPP_INTERFACE -std=c++11 -fPIC -Wall -O2
LNK_FLAGS := \
    -Wl,-rpath-link=$(NPU_HOST_LIB) \
    -Wl,-rpath-link=$(HOME)/ascend_ddk/$(arch)/lib \
	-L$(NPU_HOST_LIB) \
	-L$(HOME)/ascend_ddk/$(arch)/lib \
	-lascendcl \
	-lacl_dvpp \
	-lstdc++ \
	-lpthread \
	-lavcodec \
	-lavformat \
	-lavdevice \
	-lavutil \
	-lswresample \
	-lavfilter \
	-lswscale \
	-shared

ifeq ($(mode), AtlasDK)
LNK_FLAGS += -L$(HOME)/Ascend/driver -lmedia_mini
endif

SRCS_ALL := $(patsubst $(LOCAL_DIR)/%.cpp, %.cpp, $(shell find $(LOCAL_DIR) -name "*.cpp"))
ifeq ($(mode), AtlasDK)
SRCS := $(SRCS_ALL)
CC_FLAGS += -DENABLE_BOARD_CAMARE
else 
SRCS := $(subst src/camera.cpp, ,$(SRCS_ALL))
endif

OBJS := $(addprefix $(OBJ_DIR)/, $(patsubst %.cpp, %.o,$(SRCS)))

ALL_OBJS := $(OBJS)

all: do_pre_build do_build

do_pre_build:
	$(Q)echo - do [$@]
	$(Q)mkdir -p $(OBJ_DIR)

do_build: $(LOCAL_LIBRARY) | do_pre_build
	$(Q)echo - do [$@]

$(LOCAL_LIBRARY): $(ALL_OBJS)
	$(Q)echo [LD] $@
	$(Q)$(CC) $(CC_FLAGS) -o $@ $^ -Wl,--whole-archive -Wl,--no-whole-archive -Wl,--start-group -Wl,--end-group -Wl,-rpath=$(RPATH_DIR) $(LNK_FLAGS)

$(OBJS): $(OBJ_DIR)/%.o : %.cpp | do_pre_build
	$(Q)echo [CC] $@
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CC_FLAGS) $(INC_DIR) -c -fstack-protector-all $< -o $@

install: all
	$(Q)echo [INSTALL] $@
	$(Q)mkdir -p $(HOME)/ascend_ddk/$(arch)/include/atlasutil
	$(Q)mkdir -p $(HOME)/ascend_ddk/$(arch)/lib
	$(Q)cp -R $(LOCAL_DIR)/include/* $(HOME)/ascend_ddk/$(arch)/include/atlasutil/
	$(Q)cp -R $(OUT_DIR)/lib*.so $(HOME)/ascend_ddk/$(arch)/lib/

clean:
	rm -rf $(TOPDIR)/$(OUT_DIR)
