cmake_minimum_required(VERSION 3.10)
set( CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake" )


project(Rasterizer)
add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
set(CMAKE_CXX_STANDARD 17)

# Set OpenCVConfig.cmake Search Directory
set( OpenCV_DIR )
if( NOT CMAKE_CL_64 )
  set( OpenCV_DIR "D:/vcpkg/installed/x86-windows/share/opencv" )
else()
  set( OpenCV_DIR "D:/vcpkg/installed/x64-windows/share/opencv" )
endif()

find_package(OpenCV REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(Rasterizer Eigen3::Eigen)


target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
