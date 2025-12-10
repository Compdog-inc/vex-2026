#!/usr/bin/env zsh

# Fail fast and show useful errors
set -euo pipefail

# Base path for VEXcode assets (paths include spaces, so keep quotes)
sdk_base="$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode"

# SDK and toolchain locations
VEX_SDK_PATH="$sdk_base/sdk/cpp/V5/V5_20240802_15_00_00"
TOOLCHAIN_BIN="$sdk_base/tools/cpp/toolchain_osx64/gcc/bin"

# Mode and build type flags
MODE="VEX"                # VEX (default) or HOST
BUILD_TYPE=""             # Optional: Debug (host-only)

# Parse all provided flags
for arg in "$@"; do
	case "$arg" in
		--host|--mac|--native)
			MODE="HOST"
			;;
		--debug)
			BUILD_TYPE="Debug"
			;;
		*)
			echo "Unknown option: $arg" >&2
			echo "Supported: --host|--mac|--native [--debug]" >&2
			exit 1
			;;
	esac
done

# Validate dependencies
if [[ "$MODE" == "VEX" ]]; then
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
fi

if ! command -v cmake >/dev/null 2>&1; then
	echo "cmake is not installed or not on PATH." >&2
	echo "Install via Homebrew: brew install cmake" >&2
	exit 1
fi

if [[ "$MODE" == "VEX" ]]; then
	# Export environment for this session
	export VEX_SDK_PATH
	export PATH="$TOOLCHAIN_BIN:$PATH"
	echo "Using VEX_SDK_PATH=$VEX_SDK_PATH"
	echo "Prepended toolchain to PATH: $TOOLCHAIN_BIN"
  if [[ -n "$BUILD_TYPE" ]]; then
    echo "Note: --debug is ignored for VEX builds (host-only)."
  fi
else
	echo "Configuring for HOST/macOS build (no VEX toolchain)."
  if [[ "$BUILD_TYPE" == "Debug" ]]; then
    echo "Using Debug build type for host (-DCMAKE_BUILD_TYPE=Debug)."
  fi
fi

# Clean previous build and configure CMake
rm -rf build
if [[ "$MODE" == "VEX" ]]; then
	cmake -S . -B build \
			-DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-vexv5.cmake" \
			-DVEX_SDK_PATH="$VEX_SDK_PATH" \
			-DHEADERS="8.0.0" \
			-DVEX=ON
else
	cmake -S . -B build \
			-DVEX=OFF \
			-DCMAKE_C_COMPILER="$(xcrun -find clang)" \
			-DCMAKE_CXX_COMPILER="$(xcrun -find clang++)" \
			${BUILD_TYPE:+-DCMAKE_BUILD_TYPE=$BUILD_TYPE}
fi

echo "CMake configure complete. Build directory: ./build"
