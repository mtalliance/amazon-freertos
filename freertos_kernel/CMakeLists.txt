cmake_minimum_required (VERSION 3.8)

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

add_library(${PROJECT_NAME} ${FreeRTOS_SRC})

if(USE_STM32F4_SUPPORT)
    set(COMMON_COMPILE_FLAGS "-DUSE_MPU -DCORTEX_M4 -DSTM32F427X -DSTM32F4XX -fno-strict-aliasing -Wall -Wextra -Wfloat-equal")
    set(COMMON_COMPILE_FLAGS_CORTEX "${COMMON_COMPILE_FLAGS} -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-omit-frame-pointer")
elseif (USE_STM32H5_SUPPORT)
    set(COMMON_COMPILE_FLAGS "-DCORTEX_M33 -DSTM32H563X -DSTM32H5XX -fno-strict-aliasing -Wall -Wextra -Wfloat-equal")
    set(COMMON_COMPILE_FLAGS_CORTEX "${COMMON_COMPILE_FLAGS} -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-omit-frame-pointer")
endif()


if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(COMMON_COMPILE_FLAGS "${COMMON_COMPILE_FLAGS} -ffloat-store")
endif()
set(CMAKE_C_FLAGS_DEBUG 	"-DDEBUG ${COMMON_COMPILE_FLAGS_CORTEX} -O0 -g")
set(CMAKE_C_FLAGS_RELEASE 	"-DRELEASE ${COMMON_COMPILE_FLAGS_CORTEX} -Os -Werror")

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
