# run script if converting .elf to .bin for flashing
./arm_toolchain/arm-gnu-toolchain-12.2.mpacbti-rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-objcopy  -O binary ./build/STM32_AUDIO.elf ./build/STM32_AUDIO.bin 
