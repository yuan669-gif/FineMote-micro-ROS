macro(finemote_toolchain)
    if (NOT DEFINED CMAKE_SYSTEM_PROCESSOR OR CMAKE_SYSTEM_PROCESSOR STREQUAL "")
        message(FATAL_ERROR "CMAKE_SYSTEM_PROCESSOR is not set.")
    endif ()

    # compiler cpu and abi flags
    set(_FINEMOTE_ARCH_FLAGS
            -mcpu=${CMAKE_SYSTEM_PROCESSOR}
            -mthumb
    )

    if (DEFINED FINEMOTE_FPU AND NOT FINEMOTE_FPU STREQUAL "")
        list(APPEND _FINEMOTE_ARCH_FLAGS -mfpu=${FINEMOTE_FPU})
    endif ()

    if (DEFINED FINEMOTE_FLOAT_ABI AND NOT FINEMOTE_FLOAT_ABI STREQUAL "")
        list(APPEND _FINEMOTE_ARCH_FLAGS -mfloat-abi=${FINEMOTE_FLOAT_ABI})
    endif ()

    set(_FINEMOTE_ABI_FLAGS
            ${_FINEMOTE_ARCH_FLAGS}
            -funsigned-char
            -fshort-enums
            -fshort-wchar
    )

    set(_FINEMOTE_SECTION_FLAGS
            -ffunction-sections
            -fdata-sections
    )

    string(JOIN " " _FINEMOTE_ARCH_FLAGS_STRING ${_FINEMOTE_ARCH_FLAGS})
    string(JOIN " " _FINEMOTE_COMMON_FLAGS_STRING
            ${_FINEMOTE_ABI_FLAGS}
            ${_FINEMOTE_SECTION_FLAGS}
    )

    # URDF limits use C++20 structural template parameters. CMake has no
    # compile-feature table for armclang, so pass its language standard explicitly.
    set(_FINEMOTE_CXX_STANDARD_FLAGS -std=c++20)

    set(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} ${_FINEMOTE_COMMON_FLAGS_STRING}")
    set(CMAKE_CXX_FLAGS_INIT
            "${CMAKE_CXX_FLAGS_INIT} ${_FINEMOTE_COMMON_FLAGS_STRING} ${_FINEMOTE_CXX_STANDARD_FLAGS}")
    set(CMAKE_ASM_FLAGS_INIT "${CMAKE_ASM_FLAGS_INIT} ${_FINEMOTE_ARCH_FLAGS_STRING} -masm=auto")

    # startup file
    set(_FINEMOTE_BOARD_DIR "${_FINEMOTE_PROJECT_DIR}/BSP/${BOARD_NAME}")
    file(GLOB _FINEMOTE_STARTUP_CANDIDATES "${_FINEMOTE_BOARD_DIR}/MDK-ARM/startup*.s")
    list(LENGTH _FINEMOTE_STARTUP_CANDIDATES _FINEMOTE_STARTUP_COUNT)
    if (NOT _FINEMOTE_STARTUP_COUNT EQUAL 1)
        message(FATAL_ERROR "Expected exactly one ArmClang startup file under ${_FINEMOTE_BOARD_DIR}/MDK-ARM, found ${_FINEMOTE_STARTUP_COUNT}.")
    endif ()
    list(GET _FINEMOTE_STARTUP_CANDIDATES 0 _FINEMOTE_STARTUP_SOURCE)
    set(_FINEMOTE_STARTUP_SOURCE "${_FINEMOTE_STARTUP_SOURCE}" CACHE INTERNAL "Board startup source" FORCE)

    # linker script
    set(_FINEMOTE_LINKER_SCRIPT "${_FINEMOTE_PROJECT_DIR}/BSP/${BOARD_NAME}/MDK-ARM/${BOARD_NAME}/${BOARD_NAME}.sct")
    if (NOT EXISTS "${_FINEMOTE_LINKER_SCRIPT}")
        message(FATAL_ERROR "ArmClang linker script not found: ${_FINEMOTE_LINKER_SCRIPT}")
    endif ()

    # for armlink
    set(_FINEMOTE_ARMLINK_CPU "${CMAKE_SYSTEM_PROCESSOR}")
    if (DEFINED FINEMOTE_FPU AND NOT FINEMOTE_FPU STREQUAL "")
        if (FINEMOTE_FPU MATCHES "-sp-")
            string(APPEND _FINEMOTE_ARMLINK_CPU ".fp.sp")
        elseif (FINEMOTE_FPU MATCHES "-dp-")
            string(APPEND _FINEMOTE_ARMLINK_CPU ".fp.dp")
        endif ()
    endif ()

    set(_FINEMOTE_BOARD_LINK_OPTIONS
            "--scatter=${_FINEMOTE_LINKER_SCRIPT}"
            "--cpu=${_FINEMOTE_ARMLINK_CPU}"
            "--strict"
            CACHE INTERNAL "ArmClang board link options"
            FORCE
    )

    string(JOIN " " _FINEMOTE_BOARD_LINK_FLAGS ${_FINEMOTE_BOARD_LINK_OPTIONS})
    set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} ${_FINEMOTE_BOARD_LINK_FLAGS}")
endmacro()

function(finemote_postprocess target)
    # map and report files
    target_link_options(${target} PRIVATE
            "--list=$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.map"
            "--map"
            "--summary_stderr"
            "--info=summarysizes"
            "--info=sizes"
            "--info=totals"
            "--info=unused"
            "--info=veneers"
            "--load_addr_map_info"
            "--xref"
            "--callgraph"
            "--symbols"
    )

    # firmware image
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} --i32combined --output "$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.hex" "$<TARGET_FILE:${target}>"
            VERBATIM
    )
endfunction()
