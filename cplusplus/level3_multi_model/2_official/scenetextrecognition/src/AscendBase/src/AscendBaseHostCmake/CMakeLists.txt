
project(AscendBaseHost)
set(LIBRARY_OUTPUT_PATH    "../../output")
include_directories("../Base")
file(GLOB_RECURSE SRCS ../*.cpp)
set(CMAKE_CXX_COMPILER c++)
add_library(ascendbasehost STATIC ${SRCS})