set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(MCU_LINKER_SCRIPT STM32F103C8Tx_FLASH.ld)
set(MCU_ARCH cortex-m3)
set(MCU_FLOAT_ABI soft)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(OPTIM -Os)
set(OPTIM_FLAGS -ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
string(REPLACE ";" " " OPTIM_FLAGS_STR "${OPTIM_FLAGS}")

set(COMMON_FLAGS "-mcpu=${MCU_ARCH} --specs=nano.specs -mfloat-abi=${MCU_FLOAT_ABI} -mthumb ${OPTIM_FLAGS_STR} -g3 ${OPTIM} -fno-common ")


set(COMPILER_DIRECT C:/gnu_arm_eabi_9_q2/bin/)
set(CMAKE_C_COMPILER ${COMPILER_DIRECT}/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER ${COMPILER_DIRECT}/arm-none-eabi-g++.exe)
set(CMAKE_ASM_COMPILER ${COMPILER_DIRECT}/arm-none-eabi-g++.exe)
set(CMAKE_OBJCOPY ${COMPILER_DIRECT}/arm-none-eabi-objcopy.exe CACHE INTERNAL "GCC TOOLCHAIN OBJCOPY")
set(CMAKE_OBJDUMP ${COMPILER_DIRECT}/arm-none-eabi-objdump.exe CACHE INTERNAL "GCC TOOLCHAIN OBJDUMP")
set(CMAKE_SIZE ${COMPILER_DIRECT}/arm-none-eabi-size.exe CACHE INTERNAL "GCC TOOLCHAIN SIZE")

set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu11")
#set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T${MCU_LINKER_SCRIPT}")
