# Persistent F103 control selection

The F103 stores only controller backend (native0 / F103 PID1) and selected motor
mask. ARMED, heartbeat lease, epoch, position goals, PID/I/D state and rollback
snapshots are NEVER persisted. Motor EEPROM0x21 and F103 Flash are separate stores.

## Normal operation

- Successful schema3 COMMIT reads back selected motor modes and all torque OFF
  and checks for unresolved EEPROM locks before saving and read-verifying the
  F103 preference. ACK/READY follows. A failed save
  cannot enable ARM and enters the existing rollback path.
- The same setting is a no-op: no Flash program or erase. Control ticks, position
  goals and heartbeats never write Flash.
- Pre-ARM CANCEL restores both the physical motor-mode snapshot and the previous
  preference. If either restore is unconfirmed, the motor FSM remains BLOCKED
  and refuses arming until rollback is confirmed.
  With no prior valid preference, the prior default is native/all12.
- Once actually ARMED, normal stopping retains the selected preference and physical
  modes. Current protection ownership, gains, calibration and wire messages do not change.

## Boot

Before the scheduler starts, the motor controller enters BOOT. On its first
pass in BOOT, the actuator reads the newest valid committed Flash record.
An empty journal or one with no
supported valid record uses position mode for all 12 motors. A Flash read error
enters BLOCKED and reports FAILED to the PC.

After completing the host startup torque OFF request, the actuator commands
torque OFF on all motors, reads it back, verifies selected motor modes,
checks for unresolved EEPROM locks on those IDs and rechecks OFF for all motors. COMMIT uses the same checks
before saving the setting.
Success enters READY.
A mode mismatch or motor I/O failure also enters BLOCKED. EVENT4 reports
MODE_FAILED with the MODE reason. Mode change requests wait until BOOT finishes.

After successful boot verification, a new HELLO/fresh heartbeat/ARMED handshake and
new position targets are still required; torqueON occurs only through the normal
target path. The initial HELLO does not invalidate a restored OFF/ready selection.
The PC may still explicitly PREPARE/SET/GET/COMMIT the same backend; unchanged
EEPROM/Flash values are only verified. Native restoration remains explicit.

## Layout and power interruption

Project linker reserves 0x0801F800..0x0801FFFF (last2KiB of F103RB128KiB Flash).
Application limit is126KiB; linker and build checks reject overlap. No loadable
settings section is included in the application BIN. Vendor linker files are not edited.

Two1KiB pages form an append-only journal of32-byte records: magic, independent
format version1, backend/mask, serial sequence, CRC32 and a final halfword commit
marker. Torn/uncommitted or CRC-invalid records cannot supersede the older valid
record. Full-page rollover erases the OTHER page before appending a new committed
record, retaining the current page until a new record verifies. Sequence wraps
using serial-number comparison. The record format deliberately excludes raw C struct padding.

Flash operations use halfword programming, HSI enabled, bounded DWT-cycle waits,
and a startup-copied SRAM routine with no external calls. IRQ masking covers only
each low-level operation, not UART transactions. Programming/page erase can delay
interrupts and SysTick, so writes are restricted to DISARMED/verified-OFF transitions,
not active motor control. The programming peripheral is relocked after an idle
operation. Physical power/voltage failures can still prevent successful writes.

There is no atomic transaction across motor EEPROM and MCU Flash. Reset during
the cross-device transition may leave mismatched modes: boot verification blocks
control and requires explicit recovery. RAM rollback snapshots do not survive reset.
Mass erase/read-protection changes or a programmer erasing the reserved pages
will clear the preference. A first successful explicit mode COMMIT initializes it.
CRC/journaling are corruption detection/recovery, not a hardware safety guarantee.

## Verification and hardware limits

`tests/pwm/mode_store_test.c` covers invalid/version/CRC/mask records, unchanged
settings, interruption at every append and page-transfer step, partial erase,
repeated page rotation, read failure and sequence wrap. Control/host tests cover
boot OFF/HELLO/ARM gating, mismatch, persistence on COMMIT, pre-ARM cancel, write
failure and failed rollback. Fakes model a two-page Flash; they do not replace
real reset/power-cut/voltage/endurance tests. No hardware flashing is performed.

Build verifies the actual ARM ELF: reserved addresses, image boundary, initial
stack, copied SRAM programming function and absence of external calls from it.
An unoptimized ARM `-fstack-usage` audit measured individual frames: actuator376,
service376, preparation384, store-save120, journal-scan72 and SRAM Flash routine40
bytes. These are individual compiler frames, not a runtime high-water guarantee;
the actuator stack remains2560 bytes and real high-water/timing tests are still needed.

Primary hardware references:
- ST [STM32F103RB](https://www.st.com/en/microcontrollers-microprocessors/stm32f103rb.html)
- ST [PM0075 Flash programming manual](https://www.st.com.cn/resource/en/programming_manual/pm0075-stm32f10xxx-flash-memory-microcontrollers-stmicroelectronics.pdf)
- ST [RM0008 reference manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
