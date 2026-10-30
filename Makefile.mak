#=======================================================================================================================#
# @file 		- Makefile.mak                                                                            				#
# @author 		- Nam Nguyen Hai										                                     			#
# @date 		- 12-September-2026																						#
# @brief 		- This file supports user for running automatically command to make compilation, debug and				#
# 				flash code easier                                     													#
#                                                                                  										#  
# @note																													#
# 1. 	Only compatible with stm32f4xc, stm32f4xb family chip															#
# 2. 	This file contains multiple targets for user to run in terminal mannual, including:								#
#		- make all			: Compile entrire project to create .hex, .bin, .elf file and display the allocated memory 	#
#		- make clean		: Delete build directory and all the compiled files											#
#		- make flash		: Load program (.elf) to microcontroller through OpenOCD and ST-link						#
#		- make flash_dfu	: Load program (.bin) to microcontroller through USB Type C (Bootloader mode)				#
#		- make size			: Display the program size (RAM/FLASH) 														#
#   	- make debug_server	: Establish a connector to the st-link v2 programmer/debugger								#
#		- make debug_gdb	: Debug using the arm-none-eabi-gdb command line tool.										#
# 3. 	User can configure to enable semihosting, freertos if use by changing this below defined flag to 1				#
# 4. 	In the src folder, user must have a program which contains main() function. If not, the compiler will cause 	#
#		errors. Defaultly, the program main.c which stores main() function.												#
#=======================================================================================================================#

#=======================================USER CONFIGURATION===========================================#
# Default test program to compile in src/
APP_NAME ?= main
# Select compilation mode (Default: 0 - No Semihosting)
USE_SEMIHOSTING ?= 0
# Acknowledge that user need to use FreeRTOS for project (Default: 0 - No use)
USE_FREERTOS ?= 0
# Choose the heap file you want when using RTOS (heap_1, heap_2, heap_3, heap_4, heap_5)
HEAP_FILE ?= heap_4

#===============================================DIRECTORIES=========================================#
# Source Files
SRC_DIR = src
LIB_SRC_DIR = lib/src
DRIVER_SRC_DIR = driver/src
CONFIG_DIR = config
FREERTOS_SRC_DIR = CMSIS/FreeRTOS/src
FREERTOS_HEAP_DIR = CMSIS/FreeRTOS/MemMang
STARTUP_DIR = startup

# Header Files
CMSIS_DEVICE_DIR = CMSIS/device/inc
CMSIS_CORTEX_DIR = CMSIS/arm/inc
CMSIS_FREERTOS_DIR = CMSIS/FreeRTOS/inc
DRIVER_INC_DIR = driver/inc
LIB_INC_DIR = lib/inc
INCLUDE_DIR = $(DRIVER_INC_DIR) $(CMSIS_DEVICE_DIR) $(CMSIS_CORTEX_DIR) $(LIB_INC_DIR) $(CONFIG_DIR)

ifeq ($(USE_FREERTOS), 1)
INCLUDE_DIR += $(CMSIS_FREERTOS_DIR)
endif

# Folder
BUILD_DIR = build
# File names 
EXECUTABLE = $(APP_NAME)	
# Application name
TARGET = $(BUILD_DIR)/$(strip $(EXECUTABLE))

#=================================================TOOLCHAIN========================================#
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

#===============================================MCU================================================#
CPU = cortex-m4
FPU = fpv4-sp-d16
MCU_FLAGS = -mcpu=$(CPU) -mthumb -mfloat-abi=hard -mfpu=$(FPU)

#========================================COMPILER AND LINKER FLAGS=================================#
# Compiler
# -std=gnu11: Use C11 language standard
# -Wall: Show all warnings (if any)
# -g: Embed debug symbols into .o and .elf files to help with debugging
# -Os: Optimize code 
# -ffunction-sections -fdata-sections: Separate functions and variables into their own sections so the Linker
# can remove unused code (Dead code).
# -I: Tells the compiler when it sees .h files included in .c files, search for them in the src, driver/src directories
CFLAGS = -c $(MCU_FLAGS) 
CFLAGS += -std=gnu11 
CFLAGS += -Wall 
CFLAGS += -g -Os
CFLAGS += -ffunction-sections -fdata-sections 
CFLAGS += $(addprefix -I, $(INCLUDE_DIR))

# Linker
# -Wl,-Map=...: Create a map file to view detailed addresses of sections (optional)
# -T: Provide the Linker Script
# --gc-sections: Flag instructing the Linker to remove dead code.
# -lc -lm: Standard C and math libraries
# -u _printf_float: Support displaying the floating-point number
LD_SCRIPTS = stm32f401ccu6.ld
LDFLAGS = $(MCU_FLAGS) 
LDFLAGS += -T $(LD_SCRIPTS)
LDFLAGS += -Wl,-Map=$(TARGET).map,--gc-sections
LDFLAGS += -lc -lm
LDFLAGS += -u _printf_float -u _scanf_float

#========================================SEMIHOSTING================================================#
# --specs=nano.specs: 
# --specs=rdimon.specs:
# -lrdimon:
ifeq ($(USE_SEMIHOSTING), 1)
LDFLAGS += --specs=nano.specs --specs=rdimon.specs -lrdimon
GDB_SEMIHOSTING_CMD = -ex "monitor arm semihosting enable"
else
LDFLAGS += --specs=nano.specs 
endif

#=====================================SOURCE FILES===================================================#
# Scan for .c files in folder component/
APP_SRC = $(SRC_DIR)/$(APP_NAME).c

# Gather source files: Scan for .c files
SUPPORT_SRCS = $(wildcard $(STARTUP_DIR)/*.c) \
               $(wildcard $(DRIVER_SRC_DIR)/*.c) \
			   $(wildcard $(LIB_SRC_DIR)/*.c) \
			   $(wildcard $(CONFIG_DIR)/*.c)

# Compile syscalls.c when NOT using Semihosting.
# syscalls.c lives in the project root
ifeq ($(USE_SEMIHOSTING), 0)
APP_SRC += $(SRC_DIR)/syscalls.c
endif

# Compile FreeRTOS files to run 
ifeq ($(USE_FREERTOS), 1)
SUPPORT_SRCS += $(wildcard $(FREERTOS_SRC_DIR)/*.c) \
				$(FREERTOS_HEAP_DIR)/$(HEAP_FILE).c
endif

SRCS = $(APP_SRC) $(SUPPORT_SRCS)

#====================================OBJECT FILES====================================================#
# C_OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(filter %.c, $(SRCS)))
# Syntax works as follows:
# 1. Run filter function: Filter out all .c files in $(SRCS) 
# -> This only takes .c files and avoids accidentally taking other files like .s, .txt, etc.
# 2. Run substitute function: The % character acts as a "stem" to store the filename part.
# Make takes the first word: src/main.c. It matches with %.c -> Make saves the stem % = src/main
# Assembles the found % into build/%.o -> Result created: build/src/main.o
# 3. Finally assigned to C_OBJS
C_OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(filter %.c, $(SRCS)))
OBJS = $(C_OBJS)

#========================================BUILD TARGETS==============================================#
all: $(BUILD_DIR) $(TARGET).elf $(TARGET).bin $(TARGET).hex size

# Create the build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Rule to compile .c files to .o files
# mkdir -p $(dir $@): Create the directory path of the target $(BUILD_DIR)/%.o
# Example: for build/src/main.o, it creates the build/src directory
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "  CC      $<"
	@$(CC) $(CFLAGS) -o $@ $<

# Link into .elf file
$(TARGET).elf: $(OBJS)
	@echo "  LD      $@"
	@$(CC) -o $@ $^ $(LDFLAGS)
	
# Create .bin file (Pure binary machine code)
# Syntax: objcopy [optional_flags] <input_file> <output_file>
$(TARGET).bin: $(TARGET).elf
	@echo "  OBJCOPY $@"
	@$(OBJCOPY) -O binary $< $@
	
# Create .hex file (Intel hex format)
$(TARGET).hex: $(TARGET).elf
	@echo "  OBJCOPY $@"
	@$(OBJCOPY) -O ihex $< $@
	
# MCU Memory Limits (STM32F401CCU6)
FLASH_SIZE = 262144
RAM_SIZE   = 65536

# Memory size
size: $(TARGET).elf
	@echo ""
	@echo "============================================================"
	@echo "                    MEMORY USAGE REPORT"
	@echo "============================================================"
	@$(SIZE) $<
	@echo ""
	@$(SIZE) --format=berkeley $< | tail -1 | awk \
	  -v flash_total=$(FLASH_SIZE) \
	  -v ram_total=$(RAM_SIZE) \
	  '{ \
	    text=$$1; data=$$2; bss=$$3; \
	    flash_used = text + data; \
	    ram_used   = data + bss; \
	    flash_pct  = flash_used * 100.0 / flash_total; \
	    ram_pct    = ram_used   * 100.0 / ram_total; \
	    printf "  FLASH : %6d / %6d bytes  (%5.1f%%)  [text + data]\n", flash_used, flash_total, flash_pct; \
	    printf "  RAM   : %6d / %6d bytes  (%5.1f%%)  [data + bss]\n",  ram_used,   ram_total,   ram_pct; \
	  }'
	@echo "============================================================"
	@echo ""

# Clean
clean:
	@echo "  CLEAN   $(BUILD_DIR)/"
	@rm -rf $(BUILD_DIR)

# Flash code
flash: 
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $(TARGET).elf verify reset exit"
	
# Flash code through USB Type-C (DFU Mode)
flash_dfu: $(TARGET).bin
	dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D $(TARGET).bin
	
# debug_server: Establish a connector to the st-link v2 programmer/debugger
# This server will stay running until manually closed with Ctrl-C
debug_server:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
	
# debug_gdb: Debug using the arm-none-eabi-gdb command line tool.
# debug_server must be called first and left running. In another termnial, running debug_gdb to run CLI command
debug_gdb:
	arm-none-eabi-gdb --silent \
	-ex "target remote :3333" \
	-ex "monitor reset halt" \
	$(GDB_SEMIHOSTING_CMD) \
	-ex "load" \
	-ex "monitor reset halt" \
	-ex "set print pretty on" \
	$(TARGET).elf
	
# Declare phony targets (prevent conflict with physical files of the same name)
.PHONY: all clean flash size debug_server debug_gdb