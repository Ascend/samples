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


LOCAL_MODULE_NAME := libpresenteragent.so

LOCAL_DIR := .
OUT_DIR = out
OBJ_DIR = $(OUT_DIR)/obj
DEPS_DIR = $(OUT_DIR)/deps
LOCAL_LIBRARY=$(OUT_DIR)/$(LOCAL_MODULE_NAME)
OUT_INC_DIR = $(OUT_DIR)/include

INC_DIR := \
	-I$(LOCAL_DIR) \
	-I$(LOCAL_DIR)/include \
	-I$(DDK_PATH)/acllib/include \
        -I$(DDK_PATH)/atc/include/protobuf \
	-I/usr/local/include \
	-I$(LOCAL_DIR)/src \

SRCS := $(patsubst $(LOCAL_DIR)/%.cpp, %.cpp, $(shell find $(LOCAL_DIR)/src -name *.cpp))
OBJS := $(addprefix $(OBJ_DIR)/, $(patsubst %.cpp, %.o,$(SRCS)))

PROTO_SRCS = $(patsubst $(LOCAL_DIR)/%.cc, %.cc, $(shell find $(LOCAL_DIR)/proto -name *.pb.cc))
PROTO_OBJS := $(addprefix $(OBJ_DIR)/, $(patsubst %.cc, %.o,$(PROTO_SRCS)))

ALL_OBJS := $(OBJS) \
	$(PROTO_OBJS) \

CC_FLAGS := $(INC_DIR) -std=c++11 -Wall -fPIC -O2

LNK_FLAGS := \
	-Wl,-rpath-link=$(DDK_PATH)/atc/lib64 \
        -L$(DDK_PATH)/atc/lib64 \
        -L$(DDK_PATH)/acllib/lib64 \
	-lprotobuf \
        -lascendcl \
	-shared

all: do_pre_build do_build

do_pre_build:
	$(Q)echo - do [$@]
	$(Q)mkdir -p $(OBJ_DIR)
	$(Q)mkdir -p $(OUT_INC_DIR)

do_build: $(LOCAL_LIBRARY) | do_pre_build
	$(Q)echo - do [$@]

$(LOCAL_LIBRARY): $(ALL_OBJS)
	$(Q)echo [LD] $@
	$(Q)$(CC) $(CC_FLAGS) -o $@ $^ -Wl,--whole-archive -Wl,--no-whole-archive -Wl,--start-group -Wl,--end-group $(LNK_FLAGS)
	$(Q)cp -R $(LOCAL_DIR)/include/* $(OUT_INC_DIR)

$(OBJS): $(OBJ_DIR)/%.o : %.cpp | do_pre_build
	$(Q)echo [CC] $@
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CC_FLAGS) $(INC_DIR) -c -fstack-protector-all $< -o $@


$(PROTO_OBJS) : proto/presenter_message.pb.cc | do_pre_build
	$(Q)echo [CC] $@
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CC_FLAGS) $(INC_DIR) -c -fstack-protector-all $< -o $@

install: all
	$(Q)echo [INSTALL] $@
	$(Q)mkdir -p $(HOME)/ascend_ddk/x86/include
	$(Q)mkdir -p $(HOME)/ascend_ddk/x86/lib
	$(Q)cp -R $(OUT_INC_DIR)/ascenddk* $(HOME)/ascend_ddk/x86/include/
	$(Q)cp -R $(OUT_DIR)/lib*.so $(HOME)/ascend_ddk/x86/lib/

install pcie: 

clean:
	rm -rf $(OUT_DIR)
