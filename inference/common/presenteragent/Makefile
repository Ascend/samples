ifndef CPU_ARCH
$(error "Can not find CPU_ARCH env, please set it in environment!.")
endif

ifndef INSTALL_DIR
$(error "Can not find INSTALL_DIR env, please set it in environment!.")
endif

ifndef THIRDPART_PATH
$(error "Can not find THIRDPART_PATH env, please set it in environment!.")
endif

ifeq ($(mode),)
mode=AtlasDK
endif

ifeq ($(CPU_ARCH), aarch64)
    CC := aarch64-linux-gnu-g++
else ifeq ($(CPU_ARCH), x86_64)
    CC := g++
else
    $(error "Unsupported param: "$(CPU_ARCH))
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
	-I$(INSTALL_DIR)/runtime/include \
	-I$(THIRDPART_PATH)/include \
    -I$(INSTALL_DIR)/compiler/include/protobuf \
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
	-Wl,-rpath-link=$(INSTALL_DIR)/compiler/lib64 \
    -L$(INSTALL_DIR)/compiler/lib64 \
    -L$(INSTALL_DIR)/runtime/lib64 \
	-L$(THIRDPART_PATH)/lib \
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
	$(Q)mkdir -p $(THIRDPART_PATH)/include
	$(Q)mkdir -p $(THIRDPART_PATH)/lib
	$(Q)cp -R $(OUT_INC_DIR)/* $(THIRDPART_PATH)/include
	$(Q)cp -R $(OUT_DIR)/lib*.so $(THIRDPART_PATH)/lib/

install pcie: 

clean:

