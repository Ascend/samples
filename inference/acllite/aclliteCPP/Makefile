TOPDIR      := $(patsubst %,%,$(CURDIR))

ifndef DDK_PATH
$(error "Can not find INSTALL_DIR env, please set it in environment!.")
endif

ifndef THIRDPART_PATH
$(error "Can not find THIRDPART_PATH env, please set it in environment!.")
endif

media_mini_exist = $(shell if [ -f ${INSTALL_DIR}/driver/libmedia_mini.so ]; then echo "exist"; else echo "notexist"; fi;)

CC := g++

LOCAL_MODULE_NAME := libacllite.so

LOCAL_DIR  := .
OUT_DIR = ./out/x86_64
OBJ_DIR = $(OUT_DIR)/obj
DEPS_DIR  = $(OUT_DIR)/deps
LOCAL_LIBRARY=$(OUT_DIR)/$(LOCAL_MODULE_NAME)
RPATH_DIR=$(THIRDPART_PATH)/lib

INC_DIR = \
    -I./include \
        -I$(THIRDPART_PATH)/include/ \
        -I$(THIRDPART_PATH)/include/ \
        -I$(THIRDPART_PATH)/include/presenter/agent/ \
        -I$(DDK_PATH)/runtime/include/ \
        -I$(DDK_PATH)/driver/

CC_FLAGS := $(INC_DIR) -DENABLE_DVPP_INTERFACE -std=c++11 -fPIC -Wall -O2
LNK_FLAGS := \
    -Wl,-rpath-link=$(DDK_PATH)/runtime/lib64/stub \
    -Wl,-rpath-link=$(THIRDPART_PATH)/lib \
        -L$(DDK_PATH)/runtime/lib64/stub \
        -L$(THIRDPART_PATH)/lib \
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
ifeq ($(media_mini_exist),exist)
LNK_FLAGS += -L${INSTALL_DIR}/driver/ -lmedia_mini
endif

SRCS_ALL := $(patsubst $(LOCAL_DIR)/%.cpp, %.cpp, $(shell find $(LOCAL_DIR) -name "*.cpp"))
ifeq ($(media_mini_exist),exist)
SRCS := $(SRCS_ALL)
CC_FLAGS += -DENABLE_BOARD_CAMARE
else
SRCS := $(subst src/CameraCapture.cpp, ,$(SRCS_ALL))
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
	$(Q)mkdir -p $(THIRDPART_PATH)/include/acllite
	$(Q)mkdir -p $(THIRDPART_PATH)/lib
	$(Q)cp -R $(LOCAL_DIR)/include/* $(THIRDPART_PATH)/include/acllite/
	$(Q)cp -R $(OUT_DIR)/lib*.so $(THIRDPART_PATH)/lib/

clean:
	rm -rf $(TOPDIR)/$(OUT_DIR)