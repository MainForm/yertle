# Motor feedback failure snapshots

Feedback SyncRead failures are captured in RAM before motor UART/DMA cleanup.
Each bus (USART1 IDs1-6; USART3 IDs7-12) retains first and latest failure since
boot. One snapshot per failed SyncRead, at its first error; later errors in that
same query do not overwrite it. Successful traffic and diagnostic queries do
not clear records. Register reads/writes and actuator broadcast commands are
excluded, so mode preparation cannot replace feedback failure evidence.

RAM is cleared by MCU reset/power-off/flashing. There is no Flash persistence,
automatic transmission, new command type, retry change or stop-policy change.
Captures and reads do not mask interrupts, read UART DR, or clear hardware flags.
The tuple is advisory: hardware/ISR state may advance between individual reads.

## Collection

After flashing this firmware and starting the matching CLI:
F3 -> reproduce the issue -> wait for control release -> `:uart-stats`.
Do this BEFORE resetting the board. Existing connected/released-only rules
apply. The CLI records `uart_bus_diagnostic` with human-readable `detail`;
the protocol log also contains the raw 0x91 frames. Join halves by
`bus`, `request_sequence`, first/latest kind and failure `id`.

GET_UART_STATS 0x08 uses the existing optional 0x91 response, schema1.
Sections 0..4 are unchanged; sections 5/6 are first-query/hardware and 7/8
are latest-query/hardware. Every added payload is 64 bytes (12-byte existing
header + 13 little-endian uint32 words). Eight extra frames for two buses
cost 512 payload bytes, at most 600 encoded UART bytes (~6ms at 1Mbaud),
only when explicitly requested while released; no change to 50Hz telemetry.
Older decoders ignore unknown sections. New hosts wait up to the existing
150ms optional-extension window when older firmware omits these sections.

## Query half (sections 5 and 7)

Words: [0] snapshot ID / per-bus failure ordinal; [1] failure MCU uptime ms;
[2] whole-query elapsed us; [3] current write/frame wait elapsed us;
[4] configured wait timeout ms; [5] motor_diag_reason code;
[6] address | (read length << 8) | (instruction 0x82 << 16);
[7] expected motor mask; [8] successfully parsed mask before first failure;
[9] bytes received by DMA; [10] bytes consumed by parser;
[11] TX length; [12] TX remaining CNDTR.
Elapsed us uses unsigned DWT cycle differences and CPU frequency (64MHz,
single-wrap limit ~67s; these queries normally take milliseconds).

## Hardware half (sections 6 and 8)

Words: [0] matching snapshot ID; [1] UART SR; [2] BRR; [3] CR1; [4] CR3;
[5] DMA1 ISR (may already have been cleared by its handler); [6] TX CCR;
[7] RX CCR; [8] RX CNDTR; [9] flags: active1, TX-finished2, failed4,
TX-DMA-failed8; [10] latched last UART error SR; [11] last error DWT cycle;
[12] CPU Hz. Last-error fields can refer to an earlier IRQ; they are not
proof of an error in this query. Snapshot ID 0 = none yet / copy busy.
The bounded sequence-version reader never spins waiting for a preempted writer.

## Verification

Host tests inject TX timeout, missing motor reply, partial RX and separate bus
failures; check before-cleanup flags, unchanged first/latest records after
successful queries and old/new wire section bounds. ARM build checks static
sensor/host stack paths. Board-level failure reproduction still requires
flashing and a real run; synthetic tests do not establish the hardware cause.
