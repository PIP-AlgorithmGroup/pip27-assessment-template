cmake_minimum_required(VERSION 3.22)
project(T3_signal_identification LANGUAGES CXX)

# 自动编译本题 src/ 下的全部 .cpp，包括考生新增的实现文件。
file(GLOB_RECURSE T3_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# 本地模板只构建待实现的静态库；正式测评入口由测评机提供。
add_library(t3_signal_identification STATIC ${T3_SOURCES})
target_compile_features(t3_signal_identification PUBLIC cxx_std_17)
target_include_directories(t3_signal_identification PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
