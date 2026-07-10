cmake_minimum_required(VERSION 3.18)
project(kist_gearsonic_inference CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(CUDAToolkit REQUIRED)

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

# ─── unitree_command_writer ──────────────────────────────────────────────────
add_library(unitree_command_writer STATIC
    src/unitree/unitree_command_writer.cpp
)
target_include_directories(unitree_command_writer PUBLIC include)
target_link_libraries(unitree_command_writer PUBLIC unitree_sdk2 unitree_state_reader)

# ─── tensorrt_inference ──────────────────────────────────────────────────────
add_library(tensorrt_inference STATIC
    src/tensorrt/InferenceEngine.cpp
)
target_include_directories(tensorrt_inference PUBLIC
    include
)
target_include_directories(tensorrt_inference PRIVATE
    thirdparty/picosha2
)
target_link_libraries(tensorrt_inference PUBLIC
    CUDA::cudart
    nvinfer
    nvonnxparser
)
set_target_properties(tensorrt_inference PROPERTIES CXX_STANDARD 20)

# ─── planner_inference ───────────────────────────────────────────────────────
add_library(planner_inference STATIC
    src/planner/planner_inference.cpp
)
target_include_directories(planner_inference PUBLIC include)
target_link_libraries(planner_inference PUBLIC tensorrt_inference input_handler unitree_state_reader)
set_target_properties(planner_inference PROPERTIES CXX_STANDARD 20)

# ─── input_handler ───────────────────────────────────────────────────────────
add_library(input_handler STATIC
    src/motion/input_handler.cpp
)
target_include_directories(input_handler PUBLIC include)
target_link_libraries(input_handler PUBLIC pico_vr_reader)

# ─── teleop_tracker ──────────────────────────────────────────────────────────
add_library(teleop_tracker STATIC
    src/teleop/teleop_tracker.cpp
)
target_include_directories(teleop_tracker PUBLIC include)
target_link_libraries(teleop_tracker PUBLIC pico_vr_reader)

# ─── control ─────────────────────────────────────────────────────────────────
add_library(control STATIC
    src/control/obs_dict_model.cpp
    src/control/token_encoder.cpp
    src/control/policy_decoder.cpp
    src/control/whole_body_controller.cpp
)
target_include_directories(control PUBLIC include)
target_link_libraries(control PUBLIC tensorrt_inference planner_inference unitree_state_reader teleop_tracker)
set_target_properties(control PROPERTIES CXX_STANDARD 20)

# ─── main ─────────────────────────────────────────────────────────────────────
add_executable(gearsonic src/main.cpp)
target_link_libraries(gearsonic PRIVATE control unitree_command_writer yaml-cpp)
set_target_properties(gearsonic PROPERTIES CXX_STANDARD 20)

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_pico_vr_reader test/test_pico_vr_reader.cpp)
target_link_libraries(test_pico_vr_reader PRIVATE pico_vr_reader)

add_executable(test_input_handler test/test_input_handler.cpp)
target_link_libraries(test_input_handler PRIVATE input_handler)

add_executable(test_unitree_state_reader test/test_unitree_state_reader.cpp)
target_link_libraries(test_unitree_state_reader PRIVATE unitree_state_reader yaml-cpp)

add_executable(test_trt_inference test/test_trt_inference.cpp)
target_link_libraries(test_trt_inference PRIVATE tensorrt_inference)
set_target_properties(test_trt_inference PROPERTIES CXX_STANDARD 20)

add_executable(test_planner_inference test/test_planner_inference.cpp)
target_link_libraries(test_planner_inference PRIVATE planner_inference yaml-cpp)
set_target_properties(test_planner_inference PROPERTIES CXX_STANDARD 20)

add_executable(test_planner_live test/test_planner_live.cpp)
target_link_libraries(test_planner_live PRIVATE planner_inference yaml-cpp)
set_target_properties(test_planner_live PROPERTIES CXX_STANDARD 20)

add_executable(test_obs_dict_model test/test_obs_dict_model.cpp)
target_link_libraries(test_obs_dict_model PRIVATE control yaml-cpp)
set_target_properties(test_obs_dict_model PROPERTIES CXX_STANDARD 20)

add_executable(test_control_dry test/test_control_dry.cpp)
target_link_libraries(test_control_dry PRIVATE control yaml-cpp)
set_target_properties(test_control_dry PROPERTIES CXX_STANDARD 20)

add_executable(test_control_live test/test_control_live.cpp)
target_link_libraries(test_control_live PRIVATE control unitree_command_writer yaml-cpp)
set_target_properties(test_control_live PROPERTIES CXX_STANDARD 20)

add_executable(lowcmd_monitor test/lowcmd_monitor.cpp)
target_link_libraries(lowcmd_monitor PRIVATE unitree_state_reader yaml-cpp)

add_executable(release_motion_service test/release_motion_service.cpp)
target_link_libraries(release_motion_service PRIVATE unitree_state_reader yaml-cpp)

add_executable(test_stage_probe test/test_stage_probe.cpp)
target_link_libraries(test_stage_probe PRIVATE control yaml-cpp)
set_target_properties(test_stage_probe PROPERTIES CXX_STANDARD 20)

add_executable(test_teleop_probe test/test_teleop_probe.cpp)
target_link_libraries(test_teleop_probe PRIVATE teleop_tracker)

add_executable(test_teleop_live test/test_teleop_live.cpp)
target_link_libraries(test_teleop_live PRIVATE teleop_tracker)
