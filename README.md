# 2025_Control — CMake Build

This project has been converted from a Makefile to modern CMake while preserving the VEX V5 SDK toolchain configuration.

## Prerequisites

- VEX SDK installed (default path: `$HOME/sdk`), or export `VEX_SDK_PATH`.
- ARM GNU tools available in PATH: `arm-none-eabi-ld`, `arm-none-eabi-objcopy`, `arm-none-eabi-size`.
- CMake >= 3.20

## Configure & Build (macOS, zsh)

```zsh
cd /Users/vladimir/Documents/vex-vscode-projects/2025_Control
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-vexv5.cmake \
  -DVEX_SDK_PATH="$HOME/sdk" \
  -DHEADERS=8.0.0
cmake --build build -j
```

Artifacts are mirrored to `build/` as:
- `build/<project>.elf`
- `build/<project>.bin`

## Notes

- Standards: C gnu99 and C++ gnu++11 (matching original). 
- If your SDK path differs, pass `-DVEX_SDK_PATH=/path/to/sdk` to `cmake`.
- The legacy `build/` layout is preserved for compatibility with existing workflows.
