cmake_minimum_required(3.8)

project(FreeRTOS)

set(FreeRTOS_SRC
        event_groups.c
        list.c
        queue.c
        stream_buffer.c
        tasks.c
        timers.c
        portable/Common/mpu_wrappers.c
        )

add_library(${PROJECT_NAME} FreeRTOS_SRC)
set(COMMON_COMPILE_FLAGS "-fno-strict-aliasing -Wall -Wextra -Wfloat-equal")
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(COMMON_COMPILE_FLAGS "${COMMON_COMPILE_FLAGS} -ffloat-store")
endif()
set(COMMON_COMPILE_FLAGS_CORTEX "${COMMON_COMPILE_FLAGS} -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-omit-frame-pointer")
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
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}../include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}../include/private
        )
