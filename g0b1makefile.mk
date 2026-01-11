MCU := G0B1
PART := STM32G0B1xx

MCU_LC := $(call lc,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m0 -mthumb
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/STM32G0xx_HAL_Driver/Src

SRC_DIR_$(MCU) := $(SRC_BASE_DIR_$(MCU)) \
	$(HAL_FOLDER_$(MCU))/Src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/STM32G0xx_HAL_Driver/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Device/ST/STM32G0xx/Include

CFLAGS_$(MCU) += \
	-DHSE_VALUE=8000000 \
	-D$(PART) \
	-DMCU_$(MCU) \
	-DHSE_STARTUP_TIMEOUT=100 \
	-DLSE_STARTUP_TIMEOUT=5000 \
	-DLSE_VALUE=32768 \
	-DDATA_CACHE_ENABLE=1 \
	-DINSTRUCTION_CACHE_ENABLE=0 \
	-DVDD_VALUE=3300 \
	-DLSI_VALUE=32000 \
	-DHSI_VALUE=16000000 \
	-DUSE_FULL_LL_DRIVER \
	-DPREFETCH_ENABLE=1

SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)

# additional CFLAGS and source for DroneCAN (FDCAN support for G0B1)
# Set G0B1_FDCAN_INSTANCE=1 for FDCAN1 (default), or =2 for FDCAN2
# Can be overridden with: make AM32_G0B1_BOOTLOADER_PA2_CAN G0B1_FDCAN_INSTANCE=2
G0B1_FDCAN_INSTANCE ?= 1

CFLAGS_DRONECAN_$(MCU) += \
	-Ibootloader/DroneCAN \
	-Ibootloader/DroneCAN/libcanard \
	-Ibootloader/DroneCAN/dsdl_generated/include \
	-DG0B1_FDCAN_INSTANCE=$(G0B1_FDCAN_INSTANCE)

SRC_DIR_DRONECAN_$(MCU) += bootloader/DroneCAN \
		bootloader/DroneCAN/dsdl_generated/src \
		bootloader/DroneCAN/libcanard

SRC_DRONECAN_$(MCU) := $(foreach dir,$(SRC_DIR_DRONECAN_$(MCU)),$(wildcard $(dir)/*.[cs]))