cmake_minimum_required(VERSION 3.18)
project(kist_gearsonic_inference CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

# ─── thirdparty: PXREARobotSDK ────────────────────────────────────────────────
add_library(pxrea SHARED IMPORTED)
set_target_properties(pxrea PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/thirdparty/pxrea/lib/libPXREARobotSDK.so
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/thirdparty/pxrea/include
)

# ─── thirdparty: unitree_sdk2 ─────────────────────────────────────────────────
set(BUILD_EXAMPLES OFF CACHE BOOL "Build unitree_sdk2 examples" FORCE)
add_subdirectory(thirdparty/unitree_sdk2)

# ─── pico_vr_reader ───────────────────────────────────────────────────────────
add_library(pico_vr_reader STATIC
    src/pico/pico_vr_reader.cpp
)
target_include_directories(pico_vr_reader PUBLIC include)
target_link_libraries(pico_vr_reader PUBLIC pxrea)

# ─── unitree_state_reader ────────────────────────────────────────────────────
add_library(unitree_state_reader STATIC
    src/unitree/unitree_state_reader.cpp
)
target_include_directories(unitree_state_reader PUBLIC include)
target_link_libraries(unitree_state_reader PUBLIC unitree_sdk2)

# ─── input_handler ───────────────────────────────────────────────────────────
add_library(input_handler STATIC
    src/motion/input_handler.cpp
)
target_include_directories(input_handler PUBLIC include)
target_link_libraries(input_handler PUBLIC pico_vr_reader)

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_pico_vr_reader test/test_pico_vr_reader.cpp)
target_link_libraries(test_pico_vr_reader PRIVATE pico_vr_reader)

add_executable(test_input_handler test/test_input_handler.cpp)
target_link_libraries(test_input_handler PRIVATE input_handler)

add_executable(test_unitree_state_reader test/test_unitree_state_reader.cpp)
target_link_libraries(test_unitree_state_reader PRIVATE unitree_state_reader yaml-cpp)
