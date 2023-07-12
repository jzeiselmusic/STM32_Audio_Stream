
#todo: turn compilation back into lib and link process
# make sure this can run in stm32 cube ide

cmake_minimum_required(VERSION 3.23)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

# specify toolchain
set(ARM_TOOLCHAIN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/arm_toolchain/arm-gnu-toolchain-12.2.mpacbti-rel1-darwin-arm64-arm-none-eabi)
set(CMAKE_C_COMPILER ${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc)
set(CMAKE_AR ${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-ar)
set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP ${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-objdump)
set(CMAKE_SIZE ${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-size)
set(CMAKE_LINKER ${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-ld)
#
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# adjust the default behavior of the FIND_XXX() commands:
# search programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# search headers and libraries in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# set project name and languages
project(STM32_AUDIO LANGUAGES C ASM)
# set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
# set processor type
set(MCPU cortex-m4)

# add executable create .elf
set(APP_EXECUTABLE_EXTENSION ".elf")

# set compile and link options
set(ASM_FLAGS -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -DDEBUG)
set(COMPILER_FLAGS -mcpu=cortex-m4 -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -c)
set(LINKER_FLAGS -Wl,--gc-sections -static -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lm -Wl,--end-group)

set(COMPILER_DEFS -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_TABLE_TWIDDLECOEF_F32_16 -DARM_TABLE_BITREVIDX_FLT_16 -DARM_TABLE_TWIDDLECOEF_RFFT_F32_32 -DARM_FFT_ALLOW_TABLES -DARM_TABLE_TWIDDLECOEF_F32_32 -DARM_TABLE_BITREVIDX_FLT_32 -DCONFIGTABLE)

# add -D definitions

# set paths to individual libraries
set(CMSIS_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS)
set(STM_HAL_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver)
set(DSP_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/DSP)
set(RTOS_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source)
set(ARM_TOOLCHAIN_INCLUDES ${ARM_TOOLCHAIN_PATH}/arm-none-eabi/include ${ARM_TOOLCHAIN_PATH}/arm-none-eabi/include/newlib-nano)


# ----------------------------------- Main Executable  ----------------------------------- #

set(EXECUTABLE_TARGET ${PROJECT_NAME}${APP_EXECUTABLE_EXTENSION})

add_executable(${EXECUTABLE_TARGET}
    ${DSP_LIB_PATH}/Linear_Filtering/lpf.c
    ${DSP_LIB_PATH}/Src/complex_numbers.c
    ${DSP_LIB_PATH}/Fourier_Transform/fourier.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_cortex.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_dma_ex.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_dma.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_exti.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_flash_ex.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_flash_ramfunc.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_flash.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_gpio.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_i2s_ex.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_i2s.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_pwr_ex.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_pwr.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_rcc_ex.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_rcc.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_spi.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal_uart.c
    ${STM_HAL_LIB_PATH}/Src/stm32f4xx_hal.c
    ${DSP_LIB_PATH}/Src/arm_dot_prod_f32.c
    ${DSP_LIB_PATH}/Src/arm_float_to_q31.c
    ${RTOS_LIB_PATH}/croutine.c
    ${RTOS_LIB_PATH}/event_groups.c
    ${RTOS_LIB_PATH}/list.c
    ${RTOS_LIB_PATH}/queue.c
    ${RTOS_LIB_PATH}/stream_buffer.c
    ${RTOS_LIB_PATH}/tasks.c
    ${RTOS_LIB_PATH}/timers.c
    ${RTOS_LIB_PATH}/CMSIS_RTOS/cmsis_os.c
    ${RTOS_LIB_PATH}/portable/GCC/ARM_CM4F/port.c
    ${RTOS_LIB_PATH}/heap_useNewlib_ST.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Startup/startup_stm32f407vetx.s
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/freertos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
)

target_compile_options(${EXECUTABLE_TARGET} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${COMPILER_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>:${ASM_FLAGS}>
)

target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE 
    ${COMPILER_DEFS}
)

target_include_directories(${EXECUTABLE_TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Fourier_Transform
    ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Linear_Filtering
)

# linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VETX_FLASH.ld)
target_link_options(${EXECUTABLE_TARGET} PRIVATE -mcpu=cortex-m4 -T${LINKER_SCRIPT} ${LINKER_FLAGS})

add_custom_target(main_build
    COMMAND make -j ${EXECUTABLE_TARGET}
    WORKING_DIRECTORY 
)
