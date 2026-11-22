# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(PROJECT_VER "2026112212")   # R26-B1: OTA 版本闸(YYYYMMDDNN,单调)
project(ESP32_S3_CAM_Mic3_RD60_PMW1 VERSION 2026112212)
