cmake_minimum_required(VERSION 3.22)

project(${BOARD_NAME} C CXX ASM)

# Structural template parameters in Algorithms/URDF require C++20.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME}
        ${_FINEMOTE_STARTUP_SOURCE}
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(_FINEMOTE_WARNING_FLAGS
        -Wall
        -Wextra
        -Wpedantic
)

add_compile_options(${_FINEMOTE_WARNING_FLAGS})

# --------------- load finemote core ---------------
add_library(finemote_core OBJECT)
add_library(finemote::core ALIAS finemote_core)

# --------------- model: URDF -> compile-time constants ---------------
#
# The URDF is embedded as a C++ raw string literal and parsed during constant
# evaluation by Algorithms/URDF. Only the file contents are copied in here; all
# URDF semantics and validation live in the C++ headers, so this step stays a
# plain text transformation with no extra host dependency.
option(GENERATE_URDF_HEADER "Embed URDF files at build time" ON)

set(FINEMOTE_URDF_DIR "${PROJECT_SOURCE_DIR}/description/urdf" CACHE PATH "Robot description directory")
set(FINEMOTE_MODEL_DIR "${PROJECT_BINARY_DIR}/finemote_model" CACHE PATH "Embedded URDF header directory")
file(GLOB FINEMOTE_URDF_FILES CONFIGURE_DEPENDS "${FINEMOTE_URDF_DIR}/*.urdf")

set(FINEMOTE_MODEL_HEADERS "")
set(FINEMOTE_ROBOTS "")
foreach (_urdf IN LISTS FINEMOTE_URDF_FILES)
    get_filename_component(robot "${_urdf}" NAME_WLE)
    if (NOT robot MATCHES "^[A-Za-z_][A-Za-z0-9_]*$")
        message(FATAL_ERROR "URDF filename must be a C++ identifier: ${_urdf}")
    endif ()
    list(APPEND FINEMOTE_ROBOTS "${robot}")
    set(_header "${FINEMOTE_MODEL_DIR}/${robot}_urdf.hpp")
    if (GENERATE_URDF_HEADER)
        add_custom_command(
                OUTPUT "${_header}"
                COMMAND ${CMAKE_COMMAND}
                        "-DURDF_IN=${_urdf}"
                        "-DURDF_OUT=${_header}"
                        "-DURDF_SYMBOL=${robot}"
                        -P "${PROJECT_SOURCE_DIR}/cmake/embed_urdf.cmake"
                DEPENDS "${_urdf}" "${PROJECT_SOURCE_DIR}/cmake/embed_urdf.cmake"
                COMMENT "Embedding ${robot}.urdf as a C++ raw string literal"
                VERBATIM
        )
    elseif (NOT EXISTS "${_header}")
        message(FATAL_ERROR "GENERATE_URDF_HEADER=OFF requires pre-generated header: ${_header}")
    endif ()
    list(APPEND FINEMOTE_MODEL_HEADERS "${_header}")
endforeach ()

add_custom_target(finemote_model DEPENDS ${FINEMOTE_MODEL_HEADERS})
add_dependencies(finemote_core finemote_model)
add_dependencies(${PROJECT_NAME} finemote_model)
target_include_directories(finemote_core PUBLIC "${FINEMOTE_MODEL_DIR}")
if ("pov_chassis" IN_LIST FINEMOTE_ROBOTS)
    target_compile_definitions(finemote_core PRIVATE FINEMOTE_POV_CHASSIS_MODEL)
endif ()

set(CORE_MODULE
        Algorithms
        Components
        Devices
        Interface
        Services
)

function(add_module module_path)
    # glob source files
    file(GLOB_RECURSE module_sources CONFIGURE_DEPENDS
            "${module_path}/*.c"
            "${module_path}/*.cpp"
            "${module_path}/*.s"
    )
    target_sources(finemote_core PRIVATE ${module_sources})

    # glob header directories
    file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS
            "${module_path}/*.h"
            "${module_path}/*.h++"
            "${module_path}/*.hpp"
    )

    if (module_headers)
        target_include_directories(finemote_core PUBLIC "${module_path}")
    endif ()
endfunction()

foreach (module IN LISTS CORE_MODULE)
    set(module_path "${PROJECT_SOURCE_DIR}/${module}")
    if (NOT IS_DIRECTORY ${module_path})
        message(WARNING "Module path does not exist or is not a directory: ${module_path}")
        continue()
    endif ()
    add_module(${module_path})
endforeach ()

target_compile_options(finemote_core PRIVATE
        "-fno-rtti"
        "-fno-exceptions"
)

message(STATUS "finemote_core initialized with modules: ${CORE_MODULE}")

# --------------- load finemote bsp ---------------
add_subdirectory(${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME})

target_link_libraries(finemote_core PUBLIC finemote::board)

# --------------- load third party libs ---------------
add_subdirectory(ThirdParty)

target_link_libraries(finemote_core PUBLIC
        CMSISDSP
        etl::etl
        FreeRTOS-POSIX
        micro-ROS
)

# ------------------ application ------------------
target_link_libraries(${PROJECT_NAME} PRIVATE
        finemote::core
        finemote::bsp
)

finemote_postprocess(${PROJECT_NAME})
