cmake_minimum_required (VERSION 3.18)

option(USE_MPU  "Activate MPU support"  ON )

project(FreeRTOS)

set(FreeRTOS_SRC
    event_groups.c
    list.c
    queue.c
    stream_buffer.c
    tasks.c
    timers.c
    $<IF:$<BOOL:${USE_MPU}>,
        portable/GCC/ARM_CM4F_MPU/port.c
        portable/Common/mpu_wrappers.c
    ,
        portable/GCC/ARM_CM4F/port.c
    >
)

add_library(${PROJECT_NAME} OBJECT ${FreeRTOS_SRC})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:RELEASE>
    $<$<CONFIG:Debug>:DEBUG>
)

set(TARGET_SPECIFIC_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
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
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include/private
    PUBLIC $<IF:$<BOOL:${USE_MPU}>,
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F_MPU
    ,
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F
    >
)
