cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

set(CMAKE_CXX_STANDARD 11)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS}  -Wall  -O2 -g  ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -O2  -g ")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS}  -Wall  -O0 -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall   -O0 -g")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
LIST(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 2.4.3 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
  endif()
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)

# custom additon of easylogging
set(easylogging_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/easyloggingpp)
add_library(easylogging STATIC
            ${easylogging_DIR}/include/easylogging++.cc
            ${easylogging_DIR}/include/easylogging++.h
            )
target_include_directories(easylogging PUBLIC ${easylogging_DIR}/include)
target_compile_definitions(easylogging PUBLIC ELPP_FEATURE_ALL ELPP_THREAD_SAFE ELPP_FEATURE_CRASH_LOG ELPP_FEATURE_PERFORMANCE_TRACKING)
set_target_properties(easylogging PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_subdirectory(slamantic)
find_package(TBB)


include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${Pangolin_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
            src/System.cc
            src/Tracking.cc
            src/LocalMapping.cc
            src/LoopClosing.cc
            src/ORBextractor.cc
            src/ORBmatcher.cc
            src/FrameDrawer.cc
            src/Converter.cc
            src/MapPoint.cc
            src/KeyFrame.cc
            src/Map.cc
            src/MapDrawer.cc
            src/Optimizer.cc
            src/PnPsolver.cc
            src/Frame.cc
            src/KeyFrameDatabase.cc
            src/Sim3Solver.cc
            src/Initializer.cc
            src/Viewer.cc
            )


target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      ${OpenCV_LIBS}
                      ${Pangolin_LIBRARIES}
                      ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
                      ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
                      slamantic
                      Eigen3::Eigen
                      easylogging
                      )

# Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
               Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
               Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})


add_executable(stereo_vkitti
               Examples/Stereo/stereo_vkitti.cc)
target_link_libraries(stereo_vkitti ${PROJECT_NAME})


add_executable(stereo_cityscapes
               Examples/Stereo/stereo_cityscapes.cc)
target_link_libraries(stereo_cityscapes ${PROJECT_NAME})

add_executable(stereo_euroc
               Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
               Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
               Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
               Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_frame
               Examples/Monocular/mono_frame.cc)
target_link_libraries(mono_frame ${PROJECT_NAME})