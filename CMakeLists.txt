cmake_minimum_required(VERSION 3.21)
project(box2d_unity LANGUAGES C CXX)

if (MSVC)
  target_compile_options(box2d_unity PRIVATE /utf-8)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 让 Box2D 生成静态库（关键：OFF）
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# 引入 Box2D（路径按你的实际位置调整）
add_subdirectory(../box2d ../box2d_build)

add_library(box2d_unity SHARED
    src/box2d_unity.cpp
    src/box2d_unity.h
)

# Windows 导出符号：用 __declspec(dllexport) 更直观
target_compile_definitions(box2d_unity PRIVATE B2U_EXPORTS)

target_include_directories(box2d_unity PRIVATE
    ../box2d/include
)

# 链接 Box2D 静态库目标名通常是 box2d（仓库里就是这个）
target_link_libraries(box2d_unity PRIVATE box2d)

# 统一输出名
set_target_properties(box2d_unity PROPERTIES OUTPUT_NAME "box2d_unity")
