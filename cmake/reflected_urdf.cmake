if (FINEMOTE_POV_HANDWRITTEN)
    message(FATAL_ERROR "Select either reflected URDF or handwritten POV")
endif ()
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
find_program(FINEMOTE_AST_CLANG NAMES armclang clang++ clang REQUIRED)
get_filename_component(_reflection_input "${FINEMOTE_REFLECTED_URDF}" ABSOLUTE BASE_DIR "${PROJECT_SOURCE_DIR}")
if (NOT EXISTS "${_reflection_input}")
    message(FATAL_ERROR "URDF not found: ${_reflection_input}")
endif ()
set(_reflection_output "${PROJECT_BINARY_DIR}/reflected/robot.cpp")
if (FINEMOTE_CONTROLLERS_YAML)
    target_compile_definitions(finemote_core PRIVATE FINEMOTE_DEVICE_DESCRIPTION)
    if (NOT BOARD_NAME STREQUAL "MC_Board")
        message(FATAL_ERROR "The device description currently supports MC_Board only")
    endif ()
    get_filename_component(_controllers "${FINEMOTE_CONTROLLERS_YAML}" ABSOLUTE BASE_DIR "${PROJECT_SOURCE_DIR}")
    if (NOT EXISTS "${_controllers}")
        message(FATAL_ERROR "Controller YAML not found: ${_controllers}")
    endif ()
    set(_controller_args --controllers "${_controllers}")
endif ()
file(GLOB_RECURSE _reflection_headers CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/Algorithms/*.h" "${PROJECT_SOURCE_DIR}/Algorithms/*.hpp"
    "${PROJECT_SOURCE_DIR}/Components/*.h" "${PROJECT_SOURCE_DIR}/Components/*.hpp"
    "${PROJECT_SOURCE_DIR}/Devices/*.h" "${PROJECT_SOURCE_DIR}/Devices/*.hpp"
    "${PROJECT_SOURCE_DIR}/Services/*.h" "${PROJECT_SOURCE_DIR}/Services/*.hpp"
    "${PROJECT_SOURCE_DIR}/Interface/*.h" "${PROJECT_SOURCE_DIR}/Interface/*.hpp"
    "${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME}/*.h" "${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME}/*.hpp")
file(GLOB _reflection_tools CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tools/urdf_codegen/*.py")
add_custom_command(OUTPUT "${_reflection_output}"
    BYPRODUCTS "${PROJECT_BINARY_DIR}/reflected/interfaces.json"
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/urdf_codegen/generate.py"
        --urdf "${_reflection_input}" ${_controller_args} --compile-db "${PROJECT_BINARY_DIR}/compile_commands.json"
        --clang "${FINEMOTE_AST_CLANG}" --output "${_reflection_output}"
        --metadata "${PROJECT_BINARY_DIR}/reflected/interfaces.json"
    DEPENDS "${_reflection_input}" ${_controllers} ${_reflection_tools} ${_reflection_headers}
        "${PROJECT_BINARY_DIR}/compile_commands.json"
    COMMENT "Reflecting C++17 interfaces and generating the URDF application" VERBATIM)
target_sources(finemote_core PRIVATE "${_reflection_output}")
if (_FINEMOTE_TOOLCHAIN_ID STREQUAL "arm-none-eabi-gcc")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:${PROJECT_NAME}>"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/${PROJECT_NAME}.bin"
        VERBATIM)
endif ()
