#
# copyright Matthias Schoerghuber (AIT)
#
#
# External dependencies
#
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED core imgproc imgcodecs highgui)
# Library name and options
#
# Target name
set(target slamantic)

set(include_path "${CMAKE_CURRENT_SOURCE_DIR}/include/${target}")
set(source_path "${CMAKE_CURRENT_SOURCE_DIR}/src")

set(headers
    ${include_path}/common.hpp
    ${include_path}/SemanticMask.hpp
    ${include_path}/Label.hpp
    ${include_path}/Semantic.hpp
    ${include_path}/SemanticImage.hpp
    ${include_path}/SemanticDescriptor.hpp
    ${include_path}/slamantic.hpp
    ${include_path}/LabelHistogram.hpp
    )

set(sources
    ${source_path}/SemanticMask.cpp
    ${source_path}/Label.cpp
    ${source_path}/Semantic.cpp
    ${source_path}/SemanticImage.cpp
    ${source_path}/PatchLabel.cpp
    ${source_path}/utils/utils.cpp
    ${source_path}/LabelHistogram.cpp
    ${source_path}/SemanticDescriptor.cpp
    ${source_path}/DynamicsFactor.cpp
    )

#
# Create library
##

# Build library
add_library(${target} STATIC
            ${headers}
            ${sources}
            )

# Export library for downstream projects
#export(TARGETS ${target} NAMESPACE ${META_PROJECT_NAME}:: FILE ${PROJECT_BINARY_DIR}/cmake/${target}/${target}-export.cmake)

#
# Include directories
#
target_include_directories(${target}
                           PRIVATE
                           include
                           PUBLIC
                           ${DEFAULT_INCLUDE_DIRECTORIES}

                           INTERFACE
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
                           )

#
# Project options
#
set_target_properties(${target}
                      PROPERTIES
                      ${DEFAULT_PROJECT_OPTIONS}
                      FOLDER "${IDE_FOLDER}"
                      )

#
# Libraries
#
target_link_libraries(${target}
                      PRIVATE

                      PUBLIC
                      ${DEFAULT_LIBRARIES}
                      easylogging
                      Eigen3::Eigen
                      ${OpenCV_LIBS}
                      INTERFACE
                      )

#
# Compile definitions
#
target_compile_definitions(${target}
                           PRIVATE

                           PUBLIC
                           ${DEFAULT_COMPILE_DEFINITIONS}

                           INTERFACE
                           )

#
# Compile options
#

target_compile_options(${target}
                       PRIVATE

                       PUBLIC
                       ${DEFAULT_COMPILE_OPTIONS}

                       INTERFACE
                       )

##
## Linker options
##
target_link_libraries(${target}
                      PRIVATE

                      PUBLIC
                      ${DEFAULT_LINKER_OPTIONS}

                      INTERFACE
                      )

set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)

##
## Deployment
##
#
# Library
#install(TARGETS ${target}
#        EXPORT "${target}-export" COMPONENT dev
#        RUNTIME DESTINATION ${INSTALL_BIN} COMPONENT runtime
#        LIBRARY DESTINATION ${INSTALL_SHARED} COMPONENT runtime
#        ARCHIVE DESTINATION ${INSTALL_LIB} COMPONENT dev
#        )


add_subdirectory(test)