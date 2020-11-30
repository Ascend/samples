#!/bin/bash

export DDK_PATH=/usr/local/Ascend/ascend-toolkit/latest/
export NPU_HOST_LIB=/usr/local/Ascend/ascend-toolkit/latest/acllib_linux.x86_64/lib64/stub
export LD_LIBRARY_PATH=/usr/local/Ascend/ascend-toolkit/20.0.RC1/x86_64-linux_gcc7.3.0/atc/lib64/:$NPU_HOST_LIB
