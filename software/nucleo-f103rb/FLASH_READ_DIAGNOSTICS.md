# Separate Flash read errors from rollback failures

The mode schema and existing request message types are unchanged. The new
codes apply to SET_CONTROL_MODE (0x0E) and to its existing status EVENT.

| Condition | Result / NACK code | Reason code | PC name |
| --- | --- | --- | --- |
| Busy transaction | 11 | existing | query_or_transaction_busy |
| Flash read failure | 20 | 13 | flash_read_failed |
| Previous rollback failure | 21 for a new PREPARE | 14 | rollback_failed |

Flash read failure has its own RAM latch. It never sets the rollback-failure
latch. Boot Flash failure produces FAULT/result20/reason13/epoch0/request0.
An actual initialization clears old session/diagnostic RAM, then re-reads Flash.
No fault/epoch history is added to persistent storage. Flash read failures
found during PREPARE retain their primary result through successful rollback.

Both latches continue to block mode preparation and ARM. This change does not
clear uncertainty or retry unsafe motor configuration automatically. It also
does not diagnose why a particular physical reset was absent from an old log.

The PC names codes20/21 in UI/NACK logs, records separate Flash/rollback status
flags, and avoids canceling a rejected new epoch after an explicit PREPARE
NACK11/8/18/20/21. Ambiguous timeouts and admitted transaction failures retain
the existing conservative cancel/release path. F5/F6/H/U/P/F7 remain available.

Before a hardware retest, flash the matching rebuilt firmware. Use the separate
SEQUENCE_AB_TEST_ONLY BIN for F7 before/PWM0/explicit-OFF measurements. Its ARM
and position-target rejection remains in place; the production BIN is separate.
