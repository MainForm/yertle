# Idle CLI mode configuration (F5/F6)

HELLO capability bit 0x10 advertises support for configuring a backend while
remaining DISARMED until a later explicit H. Bit0x40 advertises PC-owned mode
retries with one firmware transaction/request. Bit0x80 adds120s DISARMED
preparation with20s preparation-only link silence; combined capabilities0xD9.
Diagnostic v2 remains supported. See MODE_WRITE_RETRIES.md.
No new message types or schema3 transaction bytes are introduced.

After existing 0x0E PREPARE / PC 0x0D writes + 0x0C verification / COMMIT,
service_pwm clears the ARM lease. Valid 0x08/0x09/0x0A and valid 0x0C read requests
may update `last_idle_monitor_tick` only while connected, DISARMED and unleased.
They do not modify `last_host_heartbeat_tick`, establish a lease or enable torque.
Once a real heartbeat establishes a lease, only real heartbeat RX renews it;
continuous diagnostics/sensor polling cannot mask a heartbeat timeout.

Outside DISARMED PREPARING/STAGED, no polling for1000ms still disconnects the link.
Preparation alone permits20000ms silence, while its total120000ms deadline stays
bounded. ARMED heartbeat and target watchdogs are unchanged. The pre-first-ARM
snapshot/Flash rollback runs, while post-ARM normal release retains the saved
backend. Initial HELLO after reboot still restores neither goals nor ARMED.
The CLI must arm an already configured session without an intervening HELLO,
which intentionally cancels a pre-ARM transaction. Old PC beginControl callers
and their combined prepare+arm workflow remain compatible.

No changes to PID parameters, PWM cap, current protections, motor mappings or
calibration are included here. The working tree's pre-existing PWM cap changes
remain separate. Firmware build: `bash scripts/build-f103.sh` in the dev image.
Tests: tests/pwm host integration and tests/diagnostics heartbeat isolation;
real flashing, power cycles and motor operation are not performed by these tests.
