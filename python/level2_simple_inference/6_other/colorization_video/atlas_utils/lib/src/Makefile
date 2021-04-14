TOPDIR      := $(patsubst %,%,$(CURDIR))

ifndef DDK_PATH
$(error "Can not find DDK_PATH env, please set it in environment!.")
endif

ifeq ($(mode),)
mode=AtlasDK
endif

ifeq ($(mode), AtlasDK)
CC := aarch64-linux-gnu-g++
else ifeq ($(mode), ASIC)
CC := g++
else
$(error "Unsupported mode: "$(mode)", please input: AtlasDK or ASIC.")
endif

LOCAL_MODULE_NAME := libatlasutil.so
CC := aarch64-linux-gnu-g++


LOCAL_DIR  := .
OUT_DIR = out
OBJ_DIR = $(OUT_DIR)/obj
DEPS_DIR  = $(OUT_DIR)/deps
LOCAL_LIBRARY=$(OUT_DIR)/$(LOCAL_MODULE_NAME)
OUT_INC_DIR = $(OUT_DIR)/include

INC_DIR = \
    -I$(HOME)/Ascend/driver/ \
	-I$(HOME)/ascend_ddk/include/third_party/ffmpeg/ \
	-I$(DDK_PATH)/../arm64-linux_gcc7.3.0/acllib/include/

CC_FLAGS := $(INC_DIR) -DENABLE_DVPP_INTERFACE -std=c++11 -fPIC -Wall -O2
LNK_FLAGS := \
	-L$(NPU_HOST_LIB) \
	-L$(HOME)/ascend_ddk/host/lib \
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

SRCS := $(patsubst $(LOCAL_DIR)/%.cpp, %.cpp, $(shell find $(LOCAL_DIR) -name "*.cpp"))
ifeq ($(mode), ASIC)
SRCS = $(subst camera.cpp, ,$(SRCS))
endif

OBJS := $(addprefix $(OBJ_DIR)/, $(patsubst %.cpp, %.o,$(SRCS)))

ALL_OBJS := $(OBJS)

all: do_pre_build do_build

do_pre_build:
	$(Q)echo - do [$@]
	$(Q)mkdir -p $(OBJ_DIR)
	$(Q)mkdir -p $(OUT_INC_DIR)

do_build: $(LOCAL_LIBRARY) | do_pre_build
	$(Q)echo - do [$@]

$(LOCAL_LIBRARY): $(ALL_OBJS)
	$(Q)echo [LD] $@
	$(Q)$(CC) $(CC_FLAGS) -o $@ $^ -Wl,--whole-archive -Wl,--no-whole-archive -Wl,--start-group -Wl,--end-group -Wl,-rpath='/home/HwHiAiUser/HIAI_PROJECTS/ascend_lib' $(LNK_FLAGS)


$(OBJS): $(OBJ_DIR)/%.o : %.cpp | do_pre_build
	$(Q)echo [CC] $@
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CC_FLAGS) $(INC_DIR) -c -fstack-protector-all $< -o $@