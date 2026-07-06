cmake_minimum_required(VERSION 3.18)
project(kist_gearsonic_inference CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── thirdparty: PXREARobotSDK ────────────────────────────────────────────────
add_library(pxrea SHARED IMPORTED)
set_target_properties(pxrea PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/thirdparty/pxrea/lib/libPXREARobotSDK.so
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/thirdparty/pxrea/include
)

# ─── pico_vr_reader ───────────────────────────────────────────────────────────
add_library(pico_vr_reader STATIC
    src/pico/pico_vr_reader.cpp
)
target_include_directories(pico_vr_reader PUBLIC include)
target_link_libraries(pico_vr_reader PUBLIC pxrea)

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_pico_vr_reader test/test_pico_vr_reader.cpp)
target_link_libraries(test_pico_vr_reader PRIVATE pico_vr_reader)
