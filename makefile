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
# all: $(BUILD)/$(PROJECT).bin
# include vex/mkrules.mk

# Default target: build via CMake

.PHONY: all
all:
	cmake --build build -j
