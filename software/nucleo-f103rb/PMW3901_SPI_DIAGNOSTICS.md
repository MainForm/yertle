# PMW3901 staged timeout and automatic diagnostics

## Timeout ownership

The sensor task supplies OPTICAL_CALL_BUDGET_US=20000 for each call to
pmw3901_device_service. This is one state-machine service operation, not the
multi-second sensor initialization/retry sequence. The synchronous driver
cooperatively checks that call budget. The task also checks on return and
invalidates the optical sample / schedules recovery on a late return.
This is not an asynchronous task cancellation or a hard real-time guarantee
while a higher-priority interrupt/task is running. Cleanup can finish later.

The driver independently renews a 500us deadline at each TXE, RXNE and BSY
wait, for every address/data byte. Deliberate 50/500us protocol delays do not
consume the next flag wait's budget. The old 2ms transaction deadline is gone.
At a flag deadline, recheck completion before failing, like the UART path.
The conservative task budget remains enforceable even if a flag is ready.
500us is an implementation margin at 1MHz, not a datasheet-mandated maximum;
both values require validation with the real RTOS workload.

Keep SPI mode3/1MHz, pin/DMA ownership, 1s recovery, optical counts/axes and
motor/IMU behavior unchanged. Preserve pre-cleanup SR/CR1/CR2, GPIO, clock,
byte progress, last DR values and maximum polling gap without extra DR reads.

## Automatic collection without a command

Each SPI wait timeout, task-call overrun, invalid argument or unexpected chip
ID records first/latest history and enqueues a snapshot. The sensor/driver
does not write UART. The host task sends one low-priority 0x93 half at most
every 20ms after safety/RX/ACK/results/heartbeat work. A failed UART TX retains
the half and backs off 100ms. PID detail is skipped on an auto-send pass.
Each error is two ~76-byte wire frames, about 1.52ms wire time at 1Mbps total.
Disconnected hosts retain queued records for the next connection. No manual
:uart-stats or PC polling command is needed for automatic JSONL logging.

The bounded SPSC RAM queue holds four error snapshots. When offline or under
prolonged TX failure it may fill; additional events update retained latest
history and increment a dropped-event counter. The counter is reported on
automatic halves (saturated at65535), and snapshot IDs reveal gaps. First,
latest, queue and counters reset on MCU reset. No Flash writes. Logs cannot
be delivered across a physical link outage; pending records need a later
successful connection, and reboot loses RAM. Manual :uart-stats is optional
for querying first/latest history, and adds no periodic overhead.

## Wire compatibility

Response 0x93 stays 64 bytes. Byte0=schema, byte1=section; LE16 at2 and LE32
retrieval MCU uptime ms at4; fourteen LE32 words at8. Schema1 is decoded for
older firmware; new firmware emits schema2. Old hosts reject schema2 safely.
Manual sections0/1=first transaction/hardware and2/3=latest; bytes2..3 are
request sequence. Automatic sections4/5=transaction/hardware and bytes2..3
are the saturated dropped-event count. Never index the 4-part manual array
with automatic sections. The receive event stream carries automatic halves
even during another request, so the CLI logs them independently of queries.

Transaction words, schema2:
0 snapshot ID; 1 failure MCU ms; 2 transaction elapsed us;
3 flag elapsed us (or whole call elapsed for stage5); 4 applicable timeout us
(500 stage,20000 task,0 argument/ID); 5 operation low8 (read1/write2/burst3,
0=no operation) and call elapsed us in upper24 (saturated);
6 address (256=no operation); 7 requested length; 8 byte index (0=address);
9 DR writes; 10 DR reads; 11 reason (TXE1/RXNE2/BSY3/argument4/task_budget5/
sensor_ID6); 12 device state low8 and last wait stage upper bits;
13 config index low16, register bank bits16..23 (255=unknown), expected chip
ID bits24..31 only for stage6. Observed ID is hardware last_rx.
Schema1 words5/12/13 retain their older un-packed meanings.

Hardware words unchanged:0 matching ID,1 DWT cycle,2 SR,3 CR1,4 CR2,
5 GPIOA_IDR,6 GPIOA_ODR,7 APB2ENR,8 APB2RSTR,9 AFIO_MAPR,10 CPU Hz,
11 last TX,12 last RX (256=none),13 maximum polling gap us.
SR RXNE0/TXE1/MODF5/OVR6/BSY7. Match snapshot IDs between halves.
Reads are advisory, not atomic hardware tracing. Poll gaps cannot identify
the exact preempting task/IRQ. No cross-clock PC/MCU timestamp subtraction.

CLI JSONL kind optical_spi_failure_snapshot includes schema, automatic,
dropped_events, section, raw words, named details and both clock timestamps.

## Validation

Production-driver register/DWT mocks verify renewed flag waits, >2ms successful
transactions, completion-after-preemption recheck, 20ms call budget, late return,
ID mismatch, pre-reset capture, queue retention/retry/overflow and history.
Host frame tests exercise connected/ACK/register/mode priority, 20ms pacing,
100ms failure backoff and unchanged control state. Client tests cover legacy
schema1, schema2, malformed lengths/bounds, automatic/manual separation and
preservation of unsolicited receive events. CLI mock emits automatic halves
on a sensor poll and verifies logging without manual :uart-stats.
ARM static stack and Flash/RAM checks are required; hardware timing remains
to be measured after flashing. Motor mapping and policy semantics unchanged.
