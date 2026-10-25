# Temporary PWM0 / explicit torque-OFF sequence audit

This opt-in source overlay preserves the experimental A/B comparison separately
from the production firmware. Delete/revert this diagnostic directory after
the hardware investigation. It is never selected by the normal CMake build.

Within one COMMIT, capture before-PWM0, after-zero-PWM, then after-explicit-OFF
torque switches. Existing EVENT6/v1 contains three per-ID read snapshots.
The added reads perturb timing; this is a paired sequential observation, not
an independent experiment. Failed/missing reads are inconclusive.

The audit BIN always rejects ARM and position goals and never saves its
experimental backend preference to Flash. Real servo registers can change
and rollback can fail; keep the robot supported/unloaded. PWM0 is not presumed
to mean torque OFF. The normal CLI retains F5/F6/H/U/P and adds F7 reporting.
F7 requires this test firmware to obtain EVENT6 records. The key does not
select MCU-side code: F5 under this same test firmware also uses its COMMIT.

From the repository root inside the ARM build container:

```sh
cmake -S . -B build/flash-error-split-sequence \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/stm32f103rb-gcc.cmake \
  -DCMAKE_BUILD_TYPE= \
  -DCMAKE_PROJECT_INCLUDE="$PWD/diagnostics/pwm_sequence_ab/inject.cmake" \
  -DYERTLE_SEQUENCE_AUDIT_DIR="$PWD/diagnostics/pwm_sequence_ab"
cmake --build build/flash-error-split-sequence -j4
```

Flash `build/flash-error-split-sequence/firmware/yertle_f103rb.bin` explicitly.
This is not the normal `build/stm32f103rb/firmware/yertle_f103rb.bin`.
The independent host CMake project here tests hypothetical automatic activation
both disabled and enabled. It does not represent actual motor measurements.

Flash errors and failed rollbacks have the same distinct codes20/21 as the
production source; see `../../FLASH_READ_DIAGNOSTICS.md`.
