# Sensor mode-write stack correction — 2026-10-25

The production ARM build before this fix had a sensor task allocation of1024
bytes and a real nested mode-write path of1056bytes, even before context-save
and critical-section/RTOS overhead. The old and audit build .bin hashes matched:
67C620180771CFE16562240113D0246A82B52E03D61D1B4C29DCE48160FF0487.

Changes:

- `pwm_control_mode_write_allowed` atomically captures only a permission boolean
  and preparation start time instead of copying the whole control context.
  All ID/value/STAGED/snapshot/DISARMED/stop/TTL gates remain unchanged.
- Sensor stack increases256->384 Cortex-M words (1024->1536bytes). No task
  priority, service period, watchdog, retry count, motor register value, mapping,
  policy or PID gain changes.
- ARM compilation always emits `.su` reports. The post-link script checks the
  sensor's permission, target-read/write and lock-read/write call chains with
  a256byte RTOS/library/context reserve. It also rejects a permission frame
  above64bytes to prevent the full-context copy from returning unnoticed.
- Permission tests cover invalid ID/value, ARMED, stop-requested and the exact
  preparation-TTL boundary. Existing transaction/rollback tests remain enabled.

Current unoptimized Cortex-M3 results:

| Check | Function frames [bytes] | +reserve | Allocation |
| --- | ---: | ---: | ---: |
| Permission |728|984|1536|
| Lock read |920|1176|1536|
| Lock write |880|1136|1536|
| Target read |864|1120|1536|
| Target write |832|1088|1536|

The previously1056byte chain includes `snapshot`; the new permission chain no
longer calls it. The permission frame fell176->24bytes. These are explicitly
checked call paths plus a reserve, not a proof of every possible RTOS/interrupt
path or live stack high-water marks. Re-audit paths if call graphs change.

The existing8KiB RTOS heap is unchanged. An ARM compile-only layout probe gives
TCB84, queue72 and queued goal80bytes. Four task allocations (including idle),
two mutexes and one single-item target queue consume7392bytes including heap
headers/alignment; about0.78KiB remains after heap-end/alignment bookkeeping.
Linked data/BSS and initial MSP/Flash layout also pass. Runtime heap/stack-water
measurements are still required; this is not a claim of measured hardware margin.

Verification: production Docker ARM build, Flash layout check, all5 PWM/control
tests plus12 diagnostics/safe-stop/driver tests passed. The new guard rejects
the old pre-fix ARM reports. PC CLI full143 tests passed without source changes.
No real COM access, reset, motor operation, flashing, or stage-progress wire
message was performed/added by this correction. Flash the new production BIN
explicitly before hardware re-verification; real-cause confirmation is pending.

Historical timeout/retry/diagnostic changes may remain uncommitted in the user's
working tree. This stack correction does not revert or absorb those changes.
