cmake_minimum_required(VERSION 3.21)
project(tdg_project_desktop LANGUAGES CXX)

# This application is latency/CPU sensitive and the default empty build type on
# single-config generators otherwise produces an unoptimised binary.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)
find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(JPEG REQUIRED)
find_package(Threads REQUIRED)

FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.8
)
FetchContent_MakeAvailable(imgui)

add_library(tdg_imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(tdg_imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(tdg_imgui PUBLIC SDL2::SDL2 OpenGL::GL)

add_executable(tdg_direct_desktop
  src/main.cpp
  src/drone_session.cpp
)
target_compile_features(tdg_direct_desktop PRIVATE cxx_std_20)
target_include_directories(tdg_direct_desktop PRIVATE src)
target_link_libraries(tdg_direct_desktop PRIVATE tdg_imgui JPEG::JPEG Threads::Threads)
