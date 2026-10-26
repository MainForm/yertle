# Persistent motor calibration cache

This change only captures/stores offsets. It does not transform position feedback,
F103 PID errors, target raw values, policy observations, motor IDs/signs or poses.

## Storage and boot

- Calibration: 0x0801F000..0x0801F7FF, two independent 1KiB pages.
- Existing mode preference stays at 0x0801F800..0x0801FFFF, unchanged format/address.
- Application linker capacity is 124KiB. Neither settings region is in the BIN.
- motor_device_init loads the latest committed CRC/version-valid snapshot into RAM.
  A valid bit distinguishes a measured zero offset from an unknown motor.
- Store records contain all known IDs, encoded raw offsets, valid mask, generation,
  CRC32 and a commit marker programmed last. Rotation never erases the newest valid
  record. Interrupted programming/erase leaves the previous committed data available.
- Repeated identical reads compare against committed contents and do not program or
  erase Flash. Changed values/first observations create a new snapshot; successful
  reads read-verify persistence before reporting status OK.
- Only DISARMED/no-stop may program/erase calibration Flash. The RAM Flash routine
  masks interrupts only for hardware busy intervals. Mode and calibration pages are
  separate. No EEPROM on a motor is written by capture.

## Every motor offset read path

The motor bus mutex serializes the common validated-read observer. It receives:
GET_MOTOR_REGISTER 31/2; generic ranges containing bytes 31/32 (including 0/32 then
32/24 dumps); separate single-byte reads; dedicated GET_POSITION_OFFSETS; and any
internal read/readback reaching these bytes. Incomplete byte pairs accumulate until
both bytes have been read. Bad CRC/ID/length/servo-error replies never enter the cache.

The supported hardware encoding remains bit11 sign + bits0..10 magnitude; raw above
4095 is rejected rather than decoded as a different firmware revision. Storage/value
failure returns MOTOR_REGISTER status 5 (or dedicated offset status 7). Other protocol
payloads/addresses and message types remain unchanged. HELLO capability 0x200 means
successful calibration-covering GET includes persistent cache update/verification.
PC F8 requires this bit so old firmware cannot falsely claim Flash persistence.

## Verification

Host tests cover empty boot, reboot load, known signed offsets, block/split queries,
unchanged-write avoidance, CRC corruption, interrupted programming/rotation, read/write
failures and real driver response parsing. Existing PWM/mode/diagnostic tests remain.
ARM build checks both reserved regions, SRAM Flash execution and sensor stack paths.
Hardware flashing and motor reads are not performed as part of these tests.

Mass erase during external flashing will erase both caches. Normal application BIN
contains neither; program only the application when preserving existing settings.
After updating firmware, use PC CLI F3 then F8 while released to populate all 12 values.
