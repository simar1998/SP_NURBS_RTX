cmake_minimum_required(VERSION 3.25)
project(SculptPlane)

set(CMAKE_CXX_STANDARD 20)

set(HEADER_DIR "Eigen")

# Use file(GLOB ...) to add all header files to a variable
file(GLOB HEADER_FILES "${HEADER_DIR}/*.h")

add_executable(SculptPlane main.cpp stl_header.h mesh_calc/bvh.h mesh_calc/binned_sah_builder.h mesh_calc/bbox.h mesh_calc/default_builder.h mesh_calc/executor.h mesh_calc/load_obj.cpp mesh_calc/load_obj.h mesh_calc/mini_tree_builder.h mesh_calc/node.h mesh_calc/platform.h mesh_calc/ray.h mesh_calc/reinsertion_optimizer.h mesh_calc/resource.h mesh_calc/sphere.h mesh_calc/split_heuristic.h mesh_calc/stack.h mesh_calc/stream.h mesh_calc/sweep_sah_builder.h mesh_calc/thread_pool.h mesh_calc/top_down_sah_builder.h mesh_calc/tri.h mesh_calc/utils.h mesh_calc/vec.h simple_example.cpp simple_example.h intersect/MeshIntersect.cpp intersect/MeshIntersect.h gcode/InputProcessing.cpp gcode/InputProcessing.h gcode/Gcode.cpp gcode/Gcode.h gcode/gcodedef/Temp.h gcode/gcodedef/MovementMode.h gcode/gcodedef/Fan.h gcode/gcodedef/Extrusion.h gcode/gcodedef/Tool.h gcode/gcodedef/GcodeDefinitions.h sampling/DiskDistrubution.cpp sampling/DiskDistrubution.h sampling/poisson_disk_sampling.h intersect/RayGenerator.cpp intersect/RayGenerator.h curvefitting/CurveFitting.cpp curvefitting/CurveFitting.h ${HEADER_FILES})

cmake_minimum_required(VERSION 3.5)
project(tinynurbs LANGUAGES CXX)



set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/")

set(BUILD_TESTS ON CACHE BOOL "Build unit tests")
set(GLM_ROOT_DIR "" CACHE STRING "Root directory of GLM (>=0.9.9)")
set(TINYNURBS_USE_OWN_GLM ON CACHE BOOL "Use own GLM library from submodule")
message(STATUS "Variable from cache: ${GLM_ROOT_DIR}")

if(NOT TINYNURBS_USE_OWN_GLM)
    find_package(GLM 0.9.9 REQUIRED MODULE)
endif()

set(HEADER_FILES
        include/tinynurbs/tinynurbs.h
        include/tinynurbs/core/basis.h
        include/tinynurbs/core/check.h
        include/tinynurbs/core/curve.h
        include/tinynurbs/core/evaluate.h
        include/tinynurbs/core/modify.h
        include/tinynurbs/core/surface.h
        include/tinynurbs/io/obj.h
        include/tinynurbs/util/util.h
        include/tinynurbs/util/array2.h
        glm/glm.hpp
        )

set(HEADER_DIR "glm")

# Use file(GLOB ...) to get all header files
file(GLOB HEADER_FILES "${HEADER_DIR}/*.h")

# Include each header file
foreach(HEADER ${HEADER_FILES})
    include_directories(${HEADER})
endforeach()

set(HEADER_DIR "clipper2")

# Use file(GLOB ...) to get all header files
file(GLOB HEADER_FILES "${HEADER_DIR}/*.h")

# Include each header file
foreach(HEADER ${HEADER_FILES})
    include_directories(${HEADER})
endforeach()

set(HEADER_DIR "tests")

# Use file(GLOB ...) to get all header files
file(GLOB HEADER_FILES "${HEADER_DIR}/*.h")

# Include each header file
foreach(HEADER ${HEADER_FILES})
    include_directories(${HEADER})
endforeach()
source_group("Header Files" FILES ${HEADER_FILES})
source_group("CMake Files" FILES CMakeLists.txt)

add_library(tinynurbs INTERFACE)
add_library(tinynurbs::tinynurbs ALIAS tinynurbs )
if(NOT TINYNURBS_USE_OWN_GLM)
    target_link_libraries(tinynurbs INTERFACE glm)
else()
    set(GLM_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm)
    target_include_directories(tinynurbs INTERFACE $<BUILD_INTERFACE:${GLM_INCLUDE_DIRS}>)
endif()
target_include_directories(tinynurbs INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_custom_target(tinynurbs_dummy SOURCES ${HEADER_FILES} CMakeLists.txt)

if(BUILD_TESTS)
    enable_testing()
endif()