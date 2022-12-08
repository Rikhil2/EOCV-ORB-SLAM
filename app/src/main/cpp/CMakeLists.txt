# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.18.1)

# Declares and names the project.

project("eocv_orb_slam")

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opencv-4.6.0-android-sdk/OpenCV-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(orb-slam3)
include_directories(
        .
        orb-slam3
        orb-slam3/include
        orb-slam3/include/CameraModels
        orb-slam3/Thirdparty
        orb-slam3/Thirdparty/boost/include
        orb-slam3/Thirdparty/eigen3/
        orb-slam3/Thirdparty/eigen3/Eigen
        orb-slam3/Thirdparty/eigen3/Eigen/src
        orb-slam3/Thirdparty/openssl/include
        orb-slam3/Thirdparty/Sophus
)


add_library( # Sets the name of the library.
        eocv_orb_slam

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        eocv_orb_slam.cpp)

find_library( # Sets the name of the path variable.
        log-lib

        # Specifies the name of the NDK library that
        # you want CMake to locate.
        log)

target_link_libraries(
        ${PROJECT_NAME}
        orbslam3
        android
        ${log-lib}
        ${CMAKE_DL_LIBS}
)

