# CMakeList.txt: StitchingImage 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("StitchingImage")

find_package(Poco CONFIG REQUIRED Foundation Util JSON XML)
find_package(OpenCV CONFIG REQUIRED)


# 将源代码添加到此项目的可执行文件。
#add_executable (StitchingImage "StitchingImage.cpp" "StitchingImage.h")
aux_source_directory("src" ALL_DEMO_SRCS)
add_executable (StitchingImage ${ALL_DEMO_SRCS})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET StitchingImage PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
target_link_libraries(StitchingImage PRIVATE Poco::Util Poco::JSON Poco::XML opencv_core opencv_imgcodecs opencv_highgui opencv_calib3d) 

