# 2026-Control

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

- If your SDK path differs, pass `-DVEX_SDK_PATH=/path/to/sdk` to `cmake`.
