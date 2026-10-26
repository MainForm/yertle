# 12-motor 50 Hz PID detail telemetry

This stream reports F103 position-PID calculations after successful PWM UART
transmission, once per fresh sensor sample (nominal20ms). It does not measure
servo-internal PWM, delivered torque or actual motion. Native servo position
mode and direct-PWM bypass produce no PID calculation records. Gains, safety
guards, target/observation/calibration contracts are unchanged.

## Wire and scheduling

Reuses EVENT0x83: subcode7/schema1, payload64, two consecutive motor IDs per
frame, six frames per complete sample. Common16B: event/version, sensor sequence
u32, successful-output uptime_ms u32, original host target sequence u16,
first ID, count2, integration dt_ms, reserved0. Each24B motor record contains
reference/error/P/I/D (five IEEE754 float32 little endian), final signed PWM
i16, flags, reserved0. Reference and error are calibrated position-mode steps;
P/I/D are duty raw per1000 before controller polarity. Flags: valid1,
duty clipping2, reserved4(always0), integral clamp8, antiwindup16, negative polarity32,
reset64. Unselected axes are invalid zero records. Old logs can contain bit4
for the former PWM slew limiter; its wire bit is not reused. Current firmware
has no PWM change-rate limiter and no derivative low-pass filter. D uses the
fresh measured-position difference divided by actual sample dt. Speed/ACC
still profile the internal position reference; the PWM ceiling800, I clamp20,
Kp2/Ki0.5/Kd0.05 and other guards remain unchanged.

EVENT8/schema1 is28B: reserved u16 then uptime, published, pending-coalesced,
completed TX sets, failed TX sets, last sensor sequence (six u32 LE).
Counters are boot-relative and emitted nominally1Hz, including while details
are continuous. HELLO capability0x800 advertises detail support.

One64B detail payload occupies76 bytes including7B header,2B CRC,1B COBS
overhead and2 delimiters. Six frames at50Hz add22,800B/s; stats add40B/s.
With the previous13,385B/s baseline, F103 TX is36,225B/s or36.225% of
1Mbps8N1 ideal capacity. This is a byte budget, not a latency guarantee.

The actuator only publishes cached state. The host sends at most one detail
frame per2ms pass, after safety/RX/queued results/sensor replies/mode status/
heartbeat. Pending critical results and mode/register jobs suppress details.
TX still uses the existing polling driver; one detail frame occupies about
0.76ms, so a command arriving during that frame can wait for it. No extra
servo queries, unbounded queues, new TX task or mandatory telemetry ACKs.
Two bounded snapshots freeze a complete set while retaining the newest
pending set. Under pressure older pending sets are replaced and counted;
failed detail TX discards that partial set instead of retrying stale logs.
PID and its watchdogs never wait for diagnostics.50Hz is the requested
nominal cadence, not a guaranteed rate under critical traffic/backpressure.

## PC and measurement

The existing RX event stream feeds strict schema/finite-value checking and
six-fragment reassembly keyed by sensor sequence/output timestamp/target
sequence. Duplicate fragments are ignored; incomplete/invalid sets counted.
`f103_pid_feedback` JSONL records contain all12 axes, `f103_pid_stats` carries
producer/loss counters. Existing protocol raw payloads are also retained.
The dashboard shows completed12/12-set MCU and PC-receive rates over up to
100 samples, with stale state after1s. Motor position/ACK rows remain.
The PC logger flushes once per drained batch, not once per detail frame.

Flash the normal `build/stm32f103rb/firmware/yertle_f103rb.bin`; start the CLI
with `--control-backend f103-pid`. F3 connects, F5 prepares PWM if needed,
H/U/P remain manual. Capture a continuous active interval of30s or more, then
release normally. No auto-arm or real hardware action is part of unit tests.

Run the offline tool with the generated JSONL:
`python tools/analyze_pid_feedback.py <log.jsonl> --after <active_start_s> --before <active_end_s>`
It separates PC position-target attempt intervals/ACK latency, received full
PID-set intervals, producer output frequency from counter deltas, integration
dt and protocol-logging cost. A dropped telemetry set must not be mistaken for
a missed PID calculation. Mock results validate transport/decoding only;
real Hz and live stack high-water require the matching flashed board.
