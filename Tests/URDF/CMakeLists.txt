cmake_minimum_required(VERSION 3.22)
project(finemote_urdf_tests LANGUAGES CXX)
enable_testing()

get_filename_component(FINEMOTE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(model_header "${CMAKE_CURRENT_BINARY_DIR}/pov_chassis_urdf.hpp")
add_custom_command(OUTPUT "${model_header}"
    COMMAND "${CMAKE_COMMAND}"
        "-DURDF_IN=${FINEMOTE_ROOT}/description/urdf/pov_chassis.urdf"
        "-DURDF_OUT=${model_header}" -DURDF_SYMBOL=pov_chassis
        -P "${FINEMOTE_ROOT}/cmake/embed_urdf.cmake"
    DEPENDS "${FINEMOTE_ROOT}/description/urdf/pov_chassis.urdf"
            "${FINEMOTE_ROOT}/cmake/embed_urdf.cmake"
    VERBATIM)
add_executable(urdf_tests urdf_tests.cpp "${model_header}")
set(fixture_header "${CMAKE_CURRENT_BINARY_DIR}/fixture_urdf.hpp")
add_custom_command(OUTPUT "${fixture_header}"
    COMMAND "${CMAKE_COMMAND}"
        "-DURDF_IN=${CMAKE_CURRENT_SOURCE_DIR}/fixture.urdf"
        "-DURDF_OUT=${fixture_header}" -DURDF_SYMBOL=fixture
        -P "${FINEMOTE_ROOT}/cmake/embed_urdf.cmake"
    DEPENDS fixture.urdf "${FINEMOTE_ROOT}/cmake/embed_urdf.cmake"
    VERBATIM)
target_sources(urdf_tests PRIVATE "${fixture_header}")
target_include_directories(urdf_tests PRIVATE "${FINEMOTE_ROOT}/Algorithms" "${CMAKE_CURRENT_BINARY_DIR}")
set_target_properties(urdf_tests PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_compile_options(urdf_tests PRIVATE -Wall -Wextra -Wpedantic)
if (WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Host tests exercise runtime parsing with several fixed-capacity models.
    target_link_options(urdf_tests PRIVATE "-Wl,--stack,16777216")
endif ()
add_test(NAME urdf COMMAND urdf_tests)
