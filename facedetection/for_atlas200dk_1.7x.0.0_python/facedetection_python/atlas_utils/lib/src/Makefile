TOPDIR      := $(patsubst %,%,$(CURDIR))

ifndef DDK_PATH
$(error "Can not find DDK_PATH env, please set it in environment!.")
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
	-I$(DDK_PATH)/../arm64-linux_gcc7.3.0/acllib/include/
	

CC_FLAGS := $(INC_DIR) -DENABLE_DVPP_INTERFACE -std=c++11 -fPIC -Wall -O2
LNK_FLAGS := \
	-L$(NPU_HOST_LIB) \
	-L$(HOME)/Ascend/driver \
	-lmedia_mini \
	-lascendcl \
	-lacl_dvpp \
	-shared

SRCS := $(patsubst $(LOCAL_DIR)/%.cpp, %.cpp, $(shell find $(LOCAL_DIR) -name "*.cpp"))
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


clean:
	rm -rf $(TOPDIR)/out
