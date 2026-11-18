# Two-phase backend transition (schema 3)

Position goals use message 0x06 (60 bytes) in both backends. The backend and
servo register settings are changed through the transaction below.

## Wire contract

| Request | Reply | Purpose |
| --- | --- | --- |
| 0x0C GET_MOTOR_REGISTER | 0x8F / NACK | Read a servo register |
| 0x0D SET_MOTOR_REGISTER | ACK/NACK + existing 0x90 detail | Write/readback/relock |
| 0x0E SET_CONTROL_MODE | ACK/NACK + EVENT4 | PREPARE, COMMIT or CANCEL |
| 0x0F SET_MOTOR_PWM | ACK/NACK | Direct duty in PWM mode, with a zero-only disarmed check for the selected PWM candidate |

HELLO capability is bit 0x00000008; incompatible older firmware fails closed.
0x0E is exactly nine bytes: schema=3, operation (0 PREPARE / 1 COMMIT /
2 CANCEL), backend (0 native / 1 F103 PID), selected mask u16LE, nonzero epoch u32LE.
All operations are correlated by request sequence, backend, mask and epoch.
EVENT4 remains 38 bytes with schema=3; phases 0..6 retain their numeric values,
7=STAGED, 8=ROLLING_BACK, 9=CANCELED. Value 9 reports a confirmed rollback.
The verified mask describes the restored original modes.

## Transaction

1. PREPARE while DISARMED blocks ARM/targets, verifies all torques OFF and
   snapshots actual per-ID mode registers. It does not write servo mode0x21.
2. In STAGED only, PC sends twelve 0x0D writes [ID,0x21,1,EEPROM-option1,value].
   For an all-motor PID selection, all 12 servos expect 2. A partial PID
   selection expects 2 on selected axes and 0 on other axes. Position expects 0 on all axes. Each driver transaction verifies
   torqueOFF, existing mode and EEPROM lock, writes only a changed value, reads
   it back and restores/verifies the original lock. PC independently GETs readback.
   Public/general register-write permissions remain unchanged.
3. COMMIT commands torque OFF, reads back OFF and all requested modes, checks
   for unresolved EEPROM locks, then rechecks OFF before saving the Flash preference. PC requires a matching
   ACK and READY/OFF status.
4. Only then does PC send the ordinary fresh heartbeat and require ARMED.
   The original snapshot is retained until the first actual ARMED transition.
   Before ARM, CANCEL or a link reset can restore it even if COMMIT ACK was lost.
   After ARM, ordinary release never restores modes.

PREPARE keepalive requires proof of matching PREPARING/STAGED. Writes/readbacks
renew heartbeat only while STAGED. COMMIT/CANCEL wait does not send heartbeat:
a stale PREPARING event must not arm a committed controller whose ACK was lost.
Preparation now has a120000ms total deadline; DISARMED PREPARING/STAGED permits
20000ms link silence, while ordinary/ARMED heartbeat expiry remains1000ms.
The PC owns up to5 explicit target requests per ID, capped by5000ms response,
30000ms per-ID and120000ms global budgets; uncertain writes are reconciled
before another request. F103 executes only one target transaction per request.
Read-only GET may retry once on timeout. PC cleanup has a separate3000ms budget.
See MODE_WRITE_RETRIES.md for current defaults and capability requirements.

A mode-write failure, preparation timeout or pre-ARM link loss invalidates the
session. F103 waits for any queued/in-flight mode write to retire, verifies OFF,
restores the exact snapshot via driver transactions and verifies OFF/locks again.
Explicit CANCEL also confirms a completed automatic rollback for the same epoch.
After rollback, the motor controller stays BLOCKED and records whether recovery
was confirmed. The host sees CANCELED for a confirmed rollback and FAILED
(value 6) when confirmation fails. The PC reports an unconfirmed rollback as
UNCONFIRMED. ACK/NACK and 0x90 retain per-register diagnostic details; EVENT4 exposes
rollback completion and restored/unknown masks. Old successful write-cache entries
are invalidated at a new PREPARE; the four-entry recent mode replay window is bounded.

## Normal stop and limitations

No extra release packet: PC stops targets/heartbeat, then waits for fresh DISARMED
and completed safety action. The F103 target (500 ms) and link (1000 ms) watchdogs
trigger the same stop sequence in either backend. When eligible, the controller
first returns toward raw 2048. The PWM path sends zero duty before both backends
explicitly turn torque OFF and verify the result.
This is not an instantaneous hardware emergency stop/current clamp. PC current
guards and the remaining F103 thermal/voltage/sensor/following-error guards remain.

Rollback snapshots are RAM-only. Power loss/reboot cannot guarantee transaction rollback.
The selected backend/mask are now persisted separately in internal Flash and
loaded at boot; see PERSISTENT_CONTROL_MODE.md. Boot still verifies physical
modes and torqueOFF and never restores ARMED or silently reprograms a mismatch.
Software rollback cannot undo mechanical movement, an interrupted EEPROM write or
an unreachable servo. Support the robot and verify unloaded after matching firmware
and PC builds are installed. No real COM operation or flashing is part of offline tests.
