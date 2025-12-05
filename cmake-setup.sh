#!/usr/bin/env zsh

# Fail fast and show useful errors
set -euo pipefail

# Base path for VEXcode assets (paths include spaces, so keep quotes)
sdk_base="$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode"

# SDK and toolchain locations
VEX_SDK_PATH="$sdk_base/sdk/cpp/V5/V5_20240802_15_00_00"
TOOLCHAIN_BIN="$sdk_base/tools/cpp/toolchain_osx64/gcc/bin"

# Validate dependencies
if [[ ! -d "$VEX_SDK_PATH" ]]; then
	echo "VEX SDK not found at: $VEX_SDK_PATH" >&2
	echo "Please install the VEXcode C++ V5 SDK or update the path in cmake-setup.sh." >&2
	exit 1
fi

if [[ ! -d "$TOOLCHAIN_BIN" ]]; then
	echo "VEX toolchain bin not found at: $TOOLCHAIN_BIN" >&2
	echo "Please ensure VEXcode tools are installed, or update the path in cmake-setup.sh." >&2
	exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
	echo "cmake is not installed or not on PATH." >&2
	echo "Install via Homebrew: brew install cmake" >&2
	exit 1
fi

# Export environment for this session
export VEX_SDK_PATH
export PATH="$TOOLCHAIN_BIN:$PATH"

echo "Using VEX_SDK_PATH=$VEX_SDK_PATH"
echo "Prepended toolchain to PATH: $TOOLCHAIN_BIN"

# Clean previous build and configure CMake
rm -rf build
cmake -S . -B build \
	-DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-vexv5.cmake" \
	-DVEX_SDK_PATH="$VEX_SDK_PATH" \
	-DHEADERS="8.0.0"

echo "CMake configure complete. Build directory: ./build"
