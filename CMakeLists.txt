cmake_minimum_required(VERSION 3.3)

set(CMAKE_CXX_STANDARD 20)
project(idTechToolset)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

option(CLI "" ON)
option(GLTF "" ON)
set(EXPOSE_SYMBOLS spike;gltf)

set(TPD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party)

add_subdirectory(${TPD_PATH}/spike)
include(version)
toolset_version()

set_target_properties(spike_cli PROPERTIES OUTPUT_NAME idtech_toolset)

add_spike_subdir(md5mesh)
add_spike_subdir(md5anim)
add_spike_subdir(def)
