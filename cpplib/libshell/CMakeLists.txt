cmake_minimum_required(VERSION 3.1)
project(libshell)

set (CMAKE_CXX_STANDARD 11)
set(MSVC_RUNTIME "dynamic")
set(LIBIGL_EIGEN_VERSION 3.3.7 CACHE STRING "Eigen version")

if(NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)

find_package(LIBIGL QUIET)

# Add your project files
if(MSVC)
	add_definitions(-DEIGEN_STRONG_INLINE=inline)
endif()

file(GLOB LIBFILES src/*.cpp src/SecondFundamentalForm/*.cpp src/MaterialModel/*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIBFILES})

if(LIBIGL_FOUND)
	target_link_libraries(${PROJECT_NAME} igl::core)
	message("libigl found in:" ${LIBIGL_INCLUDE_DIR})
else()
	target_include_directories(${PROJECT_NAME} PRIVATE "$ENV{EIGEN3_INCLUDE_DIR}" )
endif()

# use json
add_subdirectory(external/jsoncpp)
include_directories(external/jsoncpp/include)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# Binary executables
file(GLOB CLIFILES cli/*.cpp cli/*.hpp cli/*.h)
add_executable(cli_${PROJECT_NAME} ${CLIFILES})
target_link_libraries(cli_${PROJECT_NAME} ${PROJECT_NAME} json)

file(GLOB VIEWERFILES viewer/*.cpp viewer/*.hpp viewer/*.h)
add_executable(viewer_${PROJECT_NAME} ${VIEWERFILES})
target_link_libraries(viewer_${PROJECT_NAME} ${PROJECT_NAME} igl::opengl_glfw igl::opengl_glfw_imgui)