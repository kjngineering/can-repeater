cmake_minimum_required(VERSION 3.6)
set(PRJ_NAME can_appl)
include(STM32Toolchain.cmake)

set(MCU_FAMILY STM32F1xx)
set(MCU_LINE STM32F103xx)
set(MCU_TARGET STM32F103c8t6)

project(${PRJ_NAME} C CXX ASM)

add_definitions(-DSTM32F1)
add_definitions(-D${MCU_LINE})
add_definitions(-DUSE_HAL_LIBRARY)
add_definitions(-DSTM32F103xB)
add_definitions(-DDEBUG)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
file(GLOB USER_SOURCES Core/Src/*.c)
file(GLOB USER_HEADERS Core/Inc/*.h)

file(GLOB LINKER_FILE ${MCU_LINKER_SCRIPT})
file(GLOB HAL_SOURCES Drivers/${MCU_FAMILY}_HAL_DRIVER/Src/*.c)
file(GLOB STARTUP_ASM Core/Startup/startup_stm32f103c8tx.s)
file(GLOB STM32_HEADERS Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include/*.h)
file(GLOB STM32_HEADERS ${STM32_HEADERS} Drivers/CMSIS/Include/*.h)
file(GLOB STM32_HEADERS ${STM32_HEADERS} Drivers/${MCU_FAMILY}_HAL_Driver/Inc/*.h)
file(GLOB STM32_LEGACY_HEADERS Drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy/*.h)


set(SOURCE_FILES  ${USER_SOURCES} ${USER_HEADERS})

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

set(BUILD_TARGET ${PROJECT_NAME}.elf)


add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${USER_HEADERS} ${LINKER_FILE} ${HAL_SOURCES} ${STARTUP_ASM} ${STM32_HEADERS} ${STM32_LEGACY_HEADERS})

target_include_directories(${PROJECT_NAME}.elf PUBLIC Core/Inc)
target_include_directories(${PROJECT_NAME}.elf PUBLIC Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC Drivers/CMSIS/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC Drivers/${MCU_FAMILY}_HAL_Driver/Inc)
target_include_directories(${PROJECT_NAME}.elf PUBLIC Drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy)


set(MAP_FILE ${PROJECT_SOURCE_DIR}/build/output.map)

set_target_properties(${BUILD_TARGET} PROPERTIES LINK_FLAGS 
    "-mcpu=${MCU_ARCH}\
    -mthumb\
    ${OPTIM}\
    -mfloat-abi=${MCU_FLOAT_ABI}\
    -Wl,-gc-sections -T${PROJECT_SOURCE_DIR}/${MCU_LINKER_SCRIPT}\
    -Wl,-Map,${MAP_FILE}")

target_compile_options(${BUILD_TARGET} PUBLIC 
                             -mcpu=${MCU_ARCH}
                             -mthumb -mfloat-abi=${MCU_FLOAT_ABI}
                             ${OPTIM_FLAGS} -g ${OPTIM})


file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)
set(LIST_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.lst)
set(ELF_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.elf)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
                COMMAND    ${CMAKE_COMMAND}
                ARGS       -E copy $<TARGET_FILE:${PROJECT_NAME}.elf> ${ELF_FILE}
            )
