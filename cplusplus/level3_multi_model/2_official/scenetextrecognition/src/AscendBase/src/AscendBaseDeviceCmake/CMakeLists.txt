
project(AscendBaseDevice)
set(LIBRARY_OUTPUT_PATH    "../../output")
set(CMAKE_SYSTEM_NAME Linux)
file(GLOB_RECURSE SRCS ../*.cpp)
#for x86
set(CMAKE_CXX_COMPILER /usr/local/Ascend/toolkit/toolchain/linux-x86_64/bin/aarch64-linux-gnu-g++)
#for aarch64
#set(CMAKE_CXX_COMPILER /usr/local/Ascend/toolkit/toolchain/linux-aarch64/do_arm64le_native/bin/aarch64-linux-gnu-g++)
add_library(ascendbasedevice STATIC ${SRCS})