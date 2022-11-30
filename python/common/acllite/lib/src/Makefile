TOPDIR      := $(patsubst %,%,$(CURDIR))

ifndef CPU_ARCH
$(error "Can not find CPU_ARCH env, please set it in environment!.")
endif

ifndef INSTALL_DIR
$(error "Can not find INSTALL_DIR env, please set it in environment!.")
endif

ifndef THIRDPART_PATH
$(error "Can not find THIRDPART_PATH env, please set it in environment!.")
endif

media_mini_exist = $(shell if [ -f ${INSTALL_DIR}/driver/libmedia_mini.so ]; then echo "exist"; else echo "notexist"; fi;)

ifeq ($(CPU_ARCH), aarch64)
    CC := aarch64-linux-gnu-g++
    OUT_DIR = ../atlas200dk
else ifeq ($(CPU_ARCH), x86_64)
    CC := g++
else
    $(error "Unsupported param: "$(CPU_ARCH))
endif

LOCAL_MODULE_NAME := libpython_acllite.so

LOCAL_DIR  := .
OBJ_DIR = $(OUT_DIR)/obj
DEPS_DIR  = $(OUT_DIR)/deps
LOCAL_LIBRARY=$(OUT_DIR)/$(LOCAL_MODULE_NAME)
OUT_INC_DIR = $(OUT_DIR)/include

INC_DIR = \
    -I./ \
    -I../include \
    -I$(INSTALL_DIR)/driver/ \
    -I$(INSTALL_DIR)/runtime/include/ \
    -I$(THIRDPART_PATH)/include/ \
    -I$(THIRDPART_PATH)/include/presenter/agent/ \
    #-I$(DDK_PATH)/compiler/include/protobuf

CC_FLAGS := $(INC_DIR) -DENABLE_DVPP_INTERFACE -std=c++11 -fPIC -Wall -O2
LNK_FLAGS := \
    -Wl,-rpath-link=$(INSTALL_DIR)/runtime/lib64/stub \
    -Wl,-rpath-link=$(THIRDPART_PATH)/lib \
	-L$(INSTALL_DIR)/runtime/lib64/stub \
	-L$(THIRDPART_PATH)/lib \
	-lascendcl \
    -lacl_dvpp \
	-lstdc++ \
	-lpthread \
	-shared

ifeq ($(media_mini_exist),exist)
LNK_FLAGS += -L${INSTALL_DIR}/driver -lmedia_mini
endif

SRCS_ALL := $(patsubst $(LOCAL_DIR)/%.cpp, %.cpp, $(shell find $(LOCAL_DIR) -name "*.cpp"))
ifeq ($(media_mini_exist),exist)
SRCS := $(SRCS_ALL)
else 
SRCS := $(subst camera.cpp, ,$(SRCS_ALL))
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
	$(Q)$(CC) $(CC_FLAGS) -o $@ $^ -Wl,--whole-archive -Wl,--no-whole-archive -Wl,--start-group -Wl,--end-group -Wl,-rpath='/home/HwHiAiUser/HIAI_PROJECTS/ascend_lib' $(LNK_FLAGS)

$(OBJS): $(OBJ_DIR)/%.o : %.cpp | do_pre_build
	$(Q)echo [CC] $@
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CC_FLAGS) $(INC_DIR) -c -fstack-protector-all $< -o $@

clean:
	rm -rf $(OUT_DIR)/*
