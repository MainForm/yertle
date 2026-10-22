#!/usr/bin/env bash
set -Eeuo pipefail

# The bind-mounted build directory preserves the latest log after --rm.
mkdir -p build/logs
exec > >(tee build/logs/f103-build-latest.log) 2>&1
printf 'Build started: %s\n' "$(date --iso-8601=seconds)"

stage='configure'
trap 'code=$?; printf "\nBUILD FAILED: stage=%s exit_code=%s\n" "$stage" "$code" >&2; exit "$code"' ERR

build_dir='build/stm32f103rb'
elf="$build_dir/firmware/yertle_f103rb"
bin="$elf.bin"

printf '\n[1/3] Configuring F103 firmware...\n'
cmake -S . -B "$build_dir" -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/stm32f103rb-gcc.cmake

stage='compile/link'
printf '\n[2/3] Compiling and linking F103 firmware...\n'
cmake --build "$build_dir" --target firmware

stage='verify artifacts'
printf '\n[3/3] Verifying firmware artifacts...\n'
for artifact in "$elf" "$bin"; do
    if [[ ! -s "$artifact" ]]; then
        printf 'Missing or empty artifact: %s\n' "$artifact" >&2
        false
    fi
done

# Validates that the linked output is readable as an ARM ELF and reports usage.
arm-none-eabi-size "$elf"
stat --printf='Firmware: %n\nSize: %s bytes\nModified: %y\n' "$bin"
printf '\nBUILD SUCCESS: configure, compile/link and artifact checks passed.\n'
printf 'No-work incremental builds are valid; board execution is not tested here.\n'
