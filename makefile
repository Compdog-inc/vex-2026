############################################################
# Original VEXcode makefile is commented out per request.  #
# This simplified makefile delegates build to CMake.        #
############################################################

#
# VEXcode makefile 2019_03_26_01 (commented out)
#
# VERBOSE = 1
# include vex/mkenv.mk
# SRC_C  = $(wildcard src/*.cpp)
# SRC_C += $(wildcard src/*.c)
# SRC_C += $(wildcard src/*/*.cpp)
# SRC_C += $(wildcard src/*/*.c)
# OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )
# SRC_H  = $(wildcard include/*.h)
# SRC_A  = makefile
# INC_F  = include
# all: print-arm-ld $(BUILD)/$(PROJECT).bin
# include vex/mkrules.mk
# print-arm-ld:
# 	$(info ********************* VERY IMPORTANT ***********************)
# 	@which arm-none-eabi-ld

# Default target: build via CMake

# configure:
# 	export VEX_SDK_PATH="$HOME/Library Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00"
# 	export PATH="$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/tools/cpp/toolchain_osx64/gcc/bin:$PATH"

.PHONY: all
all:
	cmake --build build -j
