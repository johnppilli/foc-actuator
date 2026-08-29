# Standalone firmware build (no CubeIDE needed). Used by CI; CubeIDE users can
# keep using the IDE, which links the same ../foc sources via .project.
#
#   make                       -> build/foc-actuator.elf/.bin/.hex + size
#   make TOOLCHAIN=/path/arm-none-eabi-
#   make flash                 -> st-flash write (needs stlink tools)

TOOLCHAIN ?= arm-none-eabi-
CC      := $(TOOLCHAIN)gcc
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE    := $(TOOLCHAIN)size
BUILD   := build
TARGET  := foc-actuator

C_SRC   := $(wildcard Core/Src/*.c) \
           $(wildcard Drivers/STM32G4xx_HAL_Driver/Src/*.c) \
           $(wildcard ../foc/*.c)
ASM_SRC := Core/Startup/startup_stm32g431cbux.s

INC  := -ICore/Inc -IDrivers/STM32G4xx_HAL_Driver/Inc -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
        -IDrivers/CMSIS/Device/ST/STM32G4xx/Include -IDrivers/CMSIS/Include -I../foc
DEFS := -DUSE_HAL_DRIVER -DSTM32G431xx
MCU  := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CFLAGS  := $(MCU) $(DEFS) $(INC) -std=gnu11 -Os -g3 -Wall -Wextra -Wno-unused-parameter \
           -ffunction-sections -fdata-sections -fstack-usage
LDFLAGS := $(MCU) -TSTM32G431CBUX_FLASH.ld --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--print-memory-usage
LDLIBS  := -lc -lm

# ../foc/x.c -> build/foc/x.o ; Core/Src/x.c -> build/Core/Src/x.o
OBJ := $(patsubst %.c,$(BUILD)/%.o,$(patsubst ../%,%,$(C_SRC))) \
       $(patsubst %.s,$(BUILD)/%.o,$(ASM_SRC))

.PHONY: all clean flash size

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD)/foc/%.o: ../foc/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -Werror -c $< -o $@

$(BUILD)/Core/Src/%.o: Core/Src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJ)
	$(CC) $(OBJ) $(LDFLAGS) $(LDLIBS) -o $@
	$(SIZE) $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

flash: $(BUILD)/$(TARGET).bin
	st-flash write $< 0x08000000

clean:
	rm -rf $(BUILD)
