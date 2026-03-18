cmake_minimum_required(VERSION 3.16)
project(radar_orin_refactor LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 86 CACHE STRING "CUDA architectures (use 87 for AGX Orin, 72 for AGX Xavier)")
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(radar_app
    app/main.cpp
    io/UdpReceiver.cpp
    tracking/MultiTargetTracker.cpp
    gpu/RadarPipeline.cu
    gpu/kernels/unpack.cu
    gpu/kernels/power.cu
    gpu/kernels/cfar.cu
    gpu/kernels/monopulse.cu
)

target_include_directories(radar_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/cuda/include
)

target_link_directories(radar_app PRIVATE
    /usr/local/cuda/lib64
)

target_link_libraries(radar_app PRIVATE
    cudart
    cufft
    pthread
)
