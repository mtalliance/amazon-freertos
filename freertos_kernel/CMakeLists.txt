cmake_minimum_required (VERSION 3.18)

project(FreeRTOS)

set(FreeRTOS_SRC
        event_groups.c
        list.c
        queue.c
        stream_buffer.c
        tasks.c
        timers.c

        $<$<BOOL:${USE_STM32F4_SUPPORT}>:
        portable/GCC/ARM_CM4F_MPU/port.c
        # Add the MPU wrappers to common files once MPU is implemented
        portable/Common/mpu_wrappers.c>

        $<$<BOOL:${USE_STM32H5_SUPPORT}>:
        portable/GCC/ARM_CM33_NTZ/non_secure/port.c
        portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c

        # Remove when integrating to RTU project
        portable/MemMang/heap_4.c
        >
        )

add_library(${PROJECT_NAME} OBJECT ${FreeRTOS_SRC})

if(USE_STM32F4_SUPPORT)
    set(TARGET_SPECIFIC_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
    )
elseif (USE_STM32H5_SUPPORT)
    set(TARGET_SPECIFIC_FLAGS
        -mcpu=cortex-m33
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv5-sp-d16
    )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:RELEASE>
        $<$<CONFIG:Debug>:DEBUG>
)

set(COMMON_FLAGS
        $<$<CONFIG:Debug>:
                -g
        >
        $<$<CONFIG:Release>:
                -Werror
        >
        $<$<STREQUAL:${CMAKE_CXX_COMPILER_ID},GNU>:
                -ffloat-store
        >
        -fsigned-char
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fno-omit-frame-pointer
        -fno-strict-aliasing
        -Wall
        -Wextra
        -Wfloat-equal
)

target_compile_options(${PROJECT_NAME} PRIVATE 
    ${TARGET_SPECIFIC_FLAGS}
    ${COMMON_FLAGS}    
)

target_link_options(${PROJECT_NAME} PRIVATE 
    ${TARGET_SPECIFIC_FLAGS}
    ${COMMON_FLAGS}
)

set_target_properties(
        ${PROJECT_NAME} PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS ON
        CXX_STANDARD_REQUIRED ON
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
)

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

# Append include directories based on target MCU
if(USE_STM32F4_SUPPORT)
    target_include_directories( ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F_MPU
    )
endif()

if(USE_STM32H5_SUPPORT)
    target_include_directories( ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM33_NTZ/non_secure
    )
endif()
