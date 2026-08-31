cmake_minimum_required(VERSION 3.22.1)
project(questteleop)

find_package(OpenXR REQUIRED CONFIG)

add_library(android_native_app_glue STATIC
    ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)
target_include_directories(android_native_app_glue PUBLIC
    ${ANDROID_NDK}/sources/android/native_app_glue)

add_library(questteleop SHARED main.cpp)
# -O2 even in debug builds: the app moves ~MB/frame of video data; at -O0
# byte-wise vector ops in the frame loop drop the headset from 90 to 18 FPS.
target_compile_options(questteleop PRIVATE -Wall -Wextra -Wno-unused-parameter -O2)
target_link_libraries(questteleop
    android_native_app_glue
    OpenXR::openxr_loader
    android
    log
    EGL
    GLESv3)

# NativeActivity glue entry point must survive the linker.
set_property(TARGET questteleop APPEND_STRING PROPERTY
    LINK_FLAGS " -u ANativeActivity_onCreate")
