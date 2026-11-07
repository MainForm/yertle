# PMW3901 SPI failure capture

The SPI1 deadline remains 2000 us for the entire transaction, including
deliberate protocol delays and preemption. This change records evidence;
it does not add retries, refresh the deadline or change the failure decision.

Before CS release or SPI1 reset, preserve the first and latest failure in RAM.
Successful operations and sensor reinitialization do not clear these records.
An MCU reset clears them. No Flash writes. The producer is the sensor task;
bounded versioned readers run with interrupts enabled. Reading the SPI data
register just for diagnostics is forbidden (could consume data / clear flags).

Existing disarmed GET_UART_STATS (0x08) additionally returns four optional
OPTICAL_SPI_DIAGNOSTICS (0x93) frames. Existing periodic 0x92 is unchanged.
Each 64-byte payload: schema=1 at byte 0, section 0..3 at byte 1, request
sequence LE16 at 2, retrieval MCU uptime LE32 ms at 4, fourteen LE32 words at 8.
Sections: first transaction=0, first hardware=1, latest transaction=2,
latest hardware=3. Match their snapshot ID before interpreting both halves.
ID 0 means absent or concurrently updating. Older hosts ignore the extension;
new clients tolerate older firmware in the existing optional 150ms window.
Four frames add about 300 wire bytes only on idle manual queries.

Transaction words (index: meaning/unit):
0 ID; 1 failure MCU ms; 2 total us; 3 current flag wait us; 4 timeout us;
5 operation (read=1/write=2/burst=3); 6 register address; 7 requested data length;
8 byte index (0=address, 1..N=data); 9 actual DR writes; 10 actual DR reads;
11 stage (TXE=1/RXNE=2/BSY=3/bad_argument=4); 12 device state;
13 config index in low16 and last successfully selected bank in bits16..23
(255=unknown after sensor reset).

Hardware words:
0 matching ID; 1 DWT cycle; 2 SPI_SR; 3 CR1; 4 CR2; 5 GPIOA_IDR;
6 GPIOA_ODR; 7 RCC_APB2ENR; 8 RCC_APB2RSTR; 9 AFIO_MAPR;
10 CPU Hz; 11 last transmitted byte; 12 last received byte
(256=none); 13 maximum consecutive polling-iteration gap us.

SR bits: RXNE=0, TXE=1, MODF=5, OVR=6, BSY=7. Check operation/byte progress,
SPE in CR1, pins/clock, and the selected stage. A large polling gap or a flag
already ready when captured is evidence of possible execution delay/race;
it does not identify the preempting IRQ/task or prove wiring is correct.
Register reads are advisory, not one atomic hardware snapshot. Intentional
50/500us protocol delays are in total elapsed time, not the polling-gap field.
Wrong sensor product ID after successful SPI transfer is last_error=2 and
does not produce a SPI timeout record. No firmware/PC timestamp subtraction.

After flashing, reproduce the issue, release control, then enter :uart-stats
before resetting the MCU. CLI JSONL kind optical_spi_failure_snapshot keeps
all raw words, ID, request sequence, MCU time and a formatted explanation.

Validation: actual driver with mocked registers/DWT tests stalled TXE/RXNE,
BSY, ready-after-preemption at deadline, pre-reset registers, byte counts,
retention and four wire sections. Client tests cover malformed lengths,
section bounds, request correlation, duplicate rejection and legacy replies.
CLI mock validates end-to-end JSONL. ARM stack paths include capture/reply.
These checks do not constitute a hardware reproduction or prove the cause.
