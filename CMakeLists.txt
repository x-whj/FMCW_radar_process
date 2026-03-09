cmake_minimum_required(VERSION 3.16)
project(radar_orin_refactor LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)  # 你的 RTX 3070 Laptop


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(radar_app
    app/main.cpp
    io/UdpReceiver.cpp
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