# Design notes

Why the firmware is shaped this way: the recurring decisions behind the
structure, the constraints that produced them, and what they imply for future
changes. Not a changelog and not a task list — it records reasoning that should
outlive the change that motivated it.

Read it before changing a subsystem, to learn which invariants are load-bearing.
Each note has the same shape: context, decision, why, and the consequence for
new work. File references point at the current implementation.

## Layering and the HAL boundary

### One-way layer dependencies
Context: the code spans CubeMX HAL initialisation, board glue, chip drivers and
application logic.
Decision: four layers, bottom-up — `Core`, `Platform`, `Devices`, `App`. A lower
layer never includes a higher one. Only `Core` and `Platform` may include the
STM32 HAL headers; `Devices` and `App` reach hardware only through `bsp_*` and
`drv_*`.
Why: keeps HAL and board specifics out of application and driver logic, so a HAL
or board change is contained and the upper layers stay portable.
Consequence: a new chip goes behind a `drv_*` module, a new board signal behind a
`bsp_*` function; an upper layer that needs `main.h` or `HAL_*` is a design bug,
not a shortcut.

### Board discretes are named, not raw pins
Context: power button, status LEDs, PTT lines and the TX/RX relay are used from
several layers.
Decision: they are exposed as named functions in `Platform/bsp_board.*`; the
active level lives only there.
Why: the pin and its active level belong together; scattering `HAL_GPIO_*` calls
would duplicate that knowledge and break the layer rule.
Consequence: add a new discrete signal to `bsp_board`, never in the consumer.

### Generic infrastructure lives where it is owned, not where it was first used
Context: a bus hook or dispatcher can look device-specific at first.
Decision: generic infrastructure goes to the lowest layer that needs it — the
EXTI and PendSV dispatchers are `Platform` modules, the scheduler is an `App`
module — even if the first consumer is elsewhere.
Why: naming infrastructure after its first consumer creates false coupling and
misleads later readers about its scope.
Consequence: before placing a module, decide ownership by who consumes it, not
by who asked for it first.

## Concurrency and scheduling

### Deferred work runs from the scheduler, not from interrupts
Context: the sample tick and other events arrive in interrupts, but the DSP that
consumes them (FFT, demodulation, noise reduction) is heavy.
Decision: interrupt hooks only copy bytes and latch flags. Work is queued as an
`AppSchedulerEvent` (`App/app_scheduler.h`) and executed in the main loop; a
handler may be notified from any context, an ISR included.
Why: heavy work inside the sample ISR would add hundreds of microseconds of
interrupt latency and starve the UART and sample handlers. A "flags plus PendSV"
variant was rejected because it would duplicate the dispatcher.
Consequence: add an event and register one handler; never call DSP directly from
an interrupt. Keep ISR work to copies and latches.

### One event, one handler, edge semantics
Context: several producers can ask for the same work between two loop passes.
Decision: `App_Scheduler_Notify()` sets a pending bit; handlers run
snapshot-and-cleared before periodic tasks; notifications that arrive before
dispatch coalesce into a single run, and there is no queue.
Why: the work reads current state anyway, so a queue would only add bookkeeping
and stale-payload risk.
Consequence: handlers must be idempotent and read the current state instead of
assuming one call per request; order within a pass is fixed by the enum.

### Interrupt priority is part of the latency budget
Context: an edge-driven peripheral can lose events when it sits below a
long-running ISR.
Decision: high-rate sample and audio interrupts take priority over slower
peripherals; a peripheral whose edges would otherwise be lost is raised rather
than polled.
Why: polling in the main loop cannot recover an edge that was never captured.
Consequence: when adding an edge-driven input, place its priority deliberately
relative to the sample ISR and verify on hardware that every edge is captured.

## Host interface and radio state

### Radio state is staged, then committed by diff
Context: the host sends one logical change as independent blocks of words and
re-sends the whole set periodically.
Decision: `App_RadioState` (`App/app_radio_state.h`) copies the committed state
into a working copy at `Begin()`, command setters mutate the copy, and `Apply()`
diffs the copy against the committed state and touches the hardware only where
something actually changed.
Why: a single diff point avoids per-field dirty flags, which are easy to forget
to set.
Consequence: every host-backed value goes through a setter and is compared in
`Apply()`; calling a driver directly from a command handler bypasses the diff and
desynchronises the hardware from the committed state.

### A staged set commits only once it is runnable
Context: the active mode and the sample-rate setting arrive in different words,
so a valid combination can be split across blocks.
Decision: before commit the pair is coerced to a runnable one (the rate is
lowered to what the mode supports); only an ATU tune holds the whole staged set
pending across blocks.
Why: dropping the set would also lose unrelated edits from the same block, and
the rate each mode can run is a DSP property, not a state-model property.
Consequence: the DSP layer owns the mode-to-rate knowledge and exposes it to the
state model; never duplicate that table in the host layer.

### Host commands are dispatched by a table, not a switch
Context: the host command block has dozens of word slots, and many are unused.
Decision: `app_hostlink.c` keeps an array of handlers indexed by the word slot,
with `NULL` meaning "not supported"; wire parsing lives inside each handler.
Why: a `switch` over dozens of commands does not scale and hides which slots are
implemented.
Consequence: a new command is a handler plus a slot index; leaving a slot `NULL`
is how the code documents "not supported".

### The transport carries no protocol semantics
Context: the host link has a byte transport and a register-map protocol on top of
it.
Decision: the slave transport only receives and sends bytes and latches the
register address (`Platform/bsp_i2c_slave.h`); the register map and the meaning of
every payload live in the caller. Its callbacks run in the ISR and must not block.
Why: separating transport from semantics lets the bus layer be tested and
replaced independently and keeps the interrupt path short.
Consequence: protocol changes belong in the host-link module, never in the
transport; blocking calls (delays, EEPROM access) are forbidden in the callbacks.

### Out-of-range host values are clamped, not rejected
Context: a host word can carry a value outside what the firmware supports.
Decision: host-facing values are clamped to the supported range where they enter
the state model (key speed, tuning offsets, volume and similar); a value is not
refused.
Why: the host re-sends its whole state periodically, so a rejection would only
drop one block and could leave the radio on a stale value; clamping keeps the
radio running on the nearest valid setting.
Consequence: validate by clamping at the setter; reject only when a value cannot
be interpreted at all (for example an unset frequency).

### The radio starts inert until the host configures it
Context: on boot the hardware has no idea what the host wants.
Decision: a single `configured` latch in `AppRadioState`, committed atomically
with the accepted configuration. Until it is set, no RF relays move, no demod
audio plays, and no TX source can key the radio.
Why: otherwise the hardware would run on its boot defaults and emit demodulated
audio before the host has said anything.
Consequence: bring-up happens on the latch edge; any new output that must stay
inert before configuration has to check `IsConfigured()`.

## FPGA bus and sample transport

### The FPGA is a black box behind one driver
Context: the FPGA's internals are not the firmware's contract, and the bus is not
publicly documented.
Decision: all bus access goes through `Devices/drv_fpga.*`; the other layers see
frequencies, sample rate, TX staging and a raw I/Q window, not opcodes or frames.
Why: an undocumented device isolated behind one boundary keeps its quirks from
leaking into the DSP and application code.
Consequence: everything bus-level is implemented in the driver; nothing else
bit-bangs the pins or assembles frames.

### The bus is serviced from the tick interrupt and must stay bounded
Context: PE10 is the sample tick and the only clock for the bus; the bus is
bidirectional on the same pins, so DMA is impossible.
Decision: all bus work happens in `Drv_Fpga_Service()` from the PE10 EXTI
callback; it services RX or TX I/Q and then at most one deferred command per
tick. Control functions only compute words and set a flag; the frame leaves on a
later tick.
Why: DMA is impossible on a bidirectional bit-banged bus, doing the work off-tick
would break sync with the sample clock, and bounding the per-tick work keeps
latency predictable.
Consequence: never add heavy work to the PE10 ISR; anything long is deferred and
issued one item per tick.

### Samples cross the bus as left-justified Q1.31
Context: the FPGA moves 24-bit I/Q samples.
Decision: in firmware they are Q1.31 — 24-bit left-justified into the top bits of
an `int32`; the raw window keeps I and Q in separate buffers, deinterleaved once
in the ISR (`drv_fpga.h`).
Why: Q1.31 is the CMSIS-native format, so no sign-extension step is needed, and
separate buffers let the DSP read contiguous I and Q without a second pass.
Consequence: work in Q1.31 and scale by the Q1.31 full-scale; never expect 24-bit
values in the low bits.

### A rate or branch change commits on a bus tick
Context: the host can change the sample rate while samples flow, and the TX/RX
branch switches on the same bus.
Decision: `SetSampleRate()` only stores the request and commits it on the tick
that carries the matching control frame (applied on the return to RX while
transmitting); `SetTxActive()` requests the branch and the switch happens on the
request's tick, dropping unread TX staging on entry to TX. The raw generation
counter changes when the ring contents become invalid.
Why: consumers must know exactly when the data crossing the ring changes, and
stale TX data from an earlier transmission must never be emitted.
Consequence: consumers compare the raw generation and resync their read offset on
a change; a requested rate or branch is not active until its tick has passed.

## Hardware primitives and shared buses

### One writer per hardware shadow
Context: several features share a 16-bit I/O expander and must not disturb each
other's bits.
Decision: the expander driver is the single owner of a 16-bit output shadow per
address, and every write is a masked read-modify-write against that shadow
(`Devices/drv_pca9535.h`).
Why: independent writers doing raw register writes would clobber bits owned by
another feature; the shadow makes bit ownership explicit.
Consequence: add a new signal as a masked field with its own mask; never write
the whole register from a client.

### Slow hardware runs as a non-blocking automaton
Context: relays and similar actuators need a pulse sequence that lasts many
milliseconds, while the main loop must keep running.
Decision: long actuator operations are non-blocking state machines advanced by a
periodic task — relay pulses, the band-filter automaton, the ATU sequence — and a
mid-sequence request latches a target that is then played out, so "applied"
always means the hardware really reached the state.
Why: blocking the main loop for milliseconds per relay would starve the rest of
the firmware, and marking a code as applied before it has been played out would
desynchronise the hardware from the state.
Consequence: a new slow actuator is a task-driven automaton with a latch; never
busy-wait on it and never report it applied before it is.

## Receive chain

### The RX chain is orchestrated in one module and fed from the sample ISR
Context: raw I/Q arrives in the sample ISR and must be demodulated at the same
cadence.
Decision: `Sdr_Rx_Process()` (`App/SDR/sdr_rx.h`) runs from the I2S DMA ISR,
reads the driver raw ring with its own offset and lag, and owns the rate plans and
mode dispatch; the radio layer only pushes committed config and scales the
resulting mono block into the stereo DAC output.
Why: keeps the soft real-time path free of logs, allocations and blocking, and
keeps all mode knowledge in one place.
Consequence: no logging or blocking in the ISR path; a new mode is a demodulator
plus a rate plan, wired only in `sdr_rx`.

### Decimation picks the cheapest filter that meets the budget
Context: the raw rate is many times the audio rate.
Decision: multistage decimation, half-band FIRs for the power-of-two stages and
an IIR front end where a sharp cut and deep stopband matter; all coefficients are
generated offline.
Why: half-band filters cut the multiply count roughly in half with a fixed,
well-behaved response, while IIR needs far fewer taps where a steep transition is
required.
Consequence: choose the filter family from the rate ratio; do not spend taps on a
stage a half-band can serve.

### A shared spectral noise estimate serves every demodulator
Context: several blocks (AGC, squelch, level) need a noise floor that does not
depend on the demodulator.
Decision: `sdr_spectrum` computes and publishes the spectral noise estimate; AGC
and squelch read it instead of estimating from demodulated audio.
Why: an audio-based estimate depends on the demodulator and on the AGC gain,
which makes calibration mode-specific and unstable.
Consequence: put noise-related calibration in the spectrum module, not in each
demodulator.

### Demodulated audio is normalized to unit scale
Context: different demodulators produce different natural output levels.
Decision: the demodulated audio has a unit nominal level; the AGC drives the
signal towards it, and for FM the demodulated level is already at that reference.
Downstream volume and DAC scaling assume this normalized level.
Why: a common reference lets one volume and DAC scaling serve every mode and
keeps the AGC target mode-independent.
Consequence: a new demodulator must output at the unit nominal level (or be
brought there by its AGC group); compensate level in the demodulator, not in the
volume path.

## Transmit chain and ALC

### The output is a scaled copy of the input; every ALC layer is an absolute limit
Context: the PA drive must track the setpoint and stay protected at the same
time.
Decision: the published TX samples are scaled by the modulator factor times a
per-band level offset, and the Q15 gain applied at emission is the minimum of
independent layers — slow trim, fast peak limiter, direct-power clamp,
reflected-power clamp, external-ALC backoff (`App/app_alc.h`).
Why: composited as a minimum, the layers never stack, protection always wins, and
each layer can be reasoned about on its own.
Consequence: a new regulation or protection layer composes with `min` and must be
an absolute gain limit; never multiply two regulators together.

### The transmitted signal is a scaled copy of a unit-scale input
Context: modulator, audio compressor, clipper and the mode-specific headroom all
act on the TX signal before the FPGA.
Decision: the modulator output is normalized to unit scale (FM is constant
modulus at that level). The compressor may drive slightly above it and the
clipper trims the excess; AM and FM additionally lower the level so peaks that
re-grow after the clipper and the channel filter do not overmodulate. The single
shift into the Q1.31 domain is applied once, at the float-to-fixed boundary, and
the spectrum is taken from this pre-ALC normalized signal.
Why: one normalized domain keeps the ALC transfer and the per-band level
calibration meaningful and mode-independent, and giving AM/FM explicit headroom
prevents overmodulation after filtering.
Consequence: a new mode outputs at unit scale and takes its own headroom; never
fold a fixed level shift into a modulator.

### The learned transfer survives key-down and band changes
Context: an ALC that restarts open-loop overshoots on the first elements of every
transmission.
Decision: the estimated transfer `K`, the slow gain, the input average and the
protection clamps are kept across key-down, a temporary zero setpoint and a band
change; per-band variation lives in the calibrated level offset, so `K` is
band-independent.
Why: keying a PA open-loop causes overshoot; and the input-to-output transfer is a
slowly varying property that does not need to be re-learned between transmissions.
Consequence: reset only the transients at key-down; treat `K` and the slow layer
as persistent state.

### Tune carriers bypass the modulator
Context: the ATU and the SWR scan need a steady low-power carrier, and the ATU
switches relays while tuning.
Decision: both modes use the same forced low-power tune state — the setpoint is
forced, the user value saved and restored, the reflected-power clamp skipped and
the learned transfer frozen; the ATU additionally blanks the carrier while relays
move (`App/app_alc.h`, `App/app_ptt.h`).
Why: relay gaps and a swept VFO would corrupt the transfer estimate, and the
carrier must not exist while contacts are switching.
Consequence: a new tuning mode reuses this state instead of writing its own
carrier; anything that keys a signal for measurement must freeze `K`.

### PTT is arbitrated with first-wins and a forced-RX latch
Context: many sources can request TX (microphone, foot switch, CAT, CW keyer,
calibration, ATU, SWR scan).
Decision: one arbiter (`App/app_ptt.*`) combines the sources, latches the first
source that becomes active and holds it until every source is released; a
protection trip forces RX and latches "blocked" so a still-held source cannot
re-key.
Why: a source that is still held after a forced RX must not immediately restart
transmission, and the decision must not be scattered across sources.
Consequence: a new TX source is an enum entry plus a setter; the transition is
decided in the main loop, never inside an ISR or a command handler.

## Protection and safety

### Protections run on a periodic tick against a cached snapshot
Context: protections read I2C sensors (gauge, charger) and ADC channels, and
several modules need the result.
Decision: a periodic tick owns the shared sensor bus, runs the sensor burst at
its own divider, compares thresholds every tick against a cached snapshot, and
publishes the snapshot read-only (`App/app_protect.h`). Other modules read the
snapshot and its derived flags, never the sensors.
Why: a single owner of the shared bus avoids races, and a cached snapshot gives
every consumer a consistent view without re-reading hardware.
Consequence: add a protection by extending the snapshot and the tick; never read
the protection sensors from another module.

### Unknown sensor data is treated as unsafe
Context: a sensor read can fail or return implausible data.
Decision: an invalid reading is not "no problem": missing gauge or charger data
inhibits charging, and an invalid PA-temperature reading inhibits TX
(`App/app_protect.h`).
Why: fail-safe — a radio that cannot verify a limit must not assume it is safe.
Consequence: default every new sensor to the unsafe action when its data is
missing; validity is a first-class field in the snapshot.

### A protection trip latches; release needs a valid recovery reading
Context: some conditions are transient, others are one-shot events.
Decision: the PA over-temperature TX inhibit is a latch, released only by a valid
reading at or below the release threshold; the reflected-power VSWR trip is a
one-shot event that forces RX; the ALC clamps sit beside them as non-latching
limits.
Why: a transient dip must not re-enable TX immediately, and a one-shot event must
not re-fire on every sample.
Consequence: choose latching, one-shot or continuous deliberately for a new
protection; a release requires a positive safe reading, not merely the absence of
the trip.

## Power, watchdog and persistence

### Power-off runs from the tick, not the main loop
Context: the main loop can be busy or stuck, but power-off must always be
reachable.
Decision: the power-button scan, the auto-off countdown and the power-off
sequence itself run in the periodic tick (`App/app_power.h`); any context can
request power-off with a one-shot call.
Why: a safety-critical action must not depend on the health of the main loop.
Consequence: put shutdown paths in the tick; a request is a flag and the sequence
runs there.

### The application never owns what an earlier stage configured
Context: the bootloader configures some peripherals before jumping to the
application.
Decision: the watchdog is started and configured by the bootloader; the
application only writes the reload key (`Platform/bsp_iwdg.h`). The reset-cause
register is cleared solely by the IAP hand-off.
Why: re-configuring a peripheral another stage owns silently breaks the contract
between stages and can cause resets or lost hand-off state.
Consequence: for a peripheral set up before the jump, touch only what the contract
allows; a deviation must be explicit and documented.

### Long blocking operations kick the watchdog at a bounded cadence
Context: an EEPROM or flash operation can run longer than the watchdog timeout.
Decision: any loop that can block a meaningful fraction of the timeout kicks the
watchdog inside the loop at a bounded cadence — once per EEPROM page, per flash
chunk (`Platform/bsp_iwdg.h`). Normal runtime is covered by the housekeeping
task.
Why: a long but legitimate operation must not trip the watchdog.
Consequence: a new long operation is responsible for its own bounded kicks; do
not extend the timeout to hide it.

### NVRAM region layout is an external contract
Context: persistent data lives in an external EEPROM in fixed regions — some
per-unit calibration, some shared with the factory image.
Decision: regions are exposed as packed structs that match the EEPROM image
byte-for-byte in a fixed field order; per-unit values come from the board's
EEPROM and are never hard-coded; one region carries a CRC with a single shared
implementation (`Devices/drv_nvram.h`).
Why: the layout is read and written by other stages and by per-unit factory data,
so reordering or padding breaks compatibility, and a value that belongs to the
board must not be baked into the firmware.
Consequence: extend regions only into reserved space, keep the structs as the
single layout definition, and read per-unit values at init.

## Telemetry and framing

### The telemetry packet layout is an external contract
Context: the host parses the telemetry packet by fixed byte offset.
Decision: `App/app_telemetry.h` pins every field offset with `_Static_assert`;
the struct is the host packet, not a local convenience.
Why: the host reads raw bytes, so any reordering or resize silently corrupts the
stream.
Consequence: extend into reserved fields, never reorder; keep the asserts in sync
with the layout.

### CRC is computed over 32-bit words to match the host
Context: the host checks the packet with a CRC of its own.
Decision: the hardware CRC unit runs in word format (`Platform/bsp_crc.h`), and
the module self-tests against two known vectors at init.
Why: a byte-format unit walks the raw byte stream and yields a different value;
the ramped self-test vector catches a regression to byte format, which an
all-zero vector cannot.
Consequence: keep the CRC in word format; change it only together with the host
and its test vectors.

### SLIP framing has a single trailing delimiter
Context: the host parser splits the stream on the frame delimiter.
Decision: a frame is the escaped payload plus one trailing delimiter byte, with
no leading delimiter.
Why: a leading delimiter puts an empty frame between packets and desynchronises
the host parser.
Consequence: keep encoding trailing-only; treat the maximum-frame constant as the
upper bound for the buffer.

### Telemetry rides the spectrum-ready event and drops when the host is slow
Context: the flow is driven by the FFT cadence, and the host link can stall.
Decision: the packet is sent after the spectrum-ready event over asynchronous DMA
with SLIP framing, dropping the frame when the channel is busy.
Why: a stalled host must not stall the radio; a dropped frame is preferable to
blocked DSP.
Consequence: never block on telemetry; a lost frame is expected, not an error.

## Logging and diagnostics

### Logging is compile-time filtered macros
Context: log calls sit on hot paths and in the field.
Decision: `LOG_*` macros (`Platform/bsp_log.h`) expand to `((void)0)` above a
compile-time threshold, so disabled calls, format strings and arguments never
reach the binary; there is no runtime level switch.
Why: a runtime filter would keep the format strings and the branch in the image,
costing space and time on paths that must stay cheap.
Consequence: anything that must appear in a release build is `INFO` or higher;
do not design a runtime filter.

### One event, one line, at the layer that owns the decision
Context: a driver error can be visible to both the driver and its caller.
Decision: the driver logs detail at `DEBUG`/`TRACE` and returns an error code; the
caller logs the outcome at `WARN`/`ERROR` without repeating the driver text. Each
logging file carries its own `LOG_TAG`.
Why: duplicate lines make a serial log hard to read and put the same fact in two
places that can drift apart.
Consequence: log the decision, not the mechanics; pass the code upward instead of
logging it twice.

## Generated coefficients and host tooling

### Filter coefficients are generated offline, not computed at runtime
Context: the target needs many fixed filters, and a design step on the MCU is
expensive and hard to verify.
Decision: Python generators under `tools/` emit the coefficient headers
(`sdr_*_filters.h`, `sdr_iir_prototypes.h`); the runtime only scales a prototype
to the current sample rate. Generated headers are not edited by hand.
Why: a small fixed prototype set plus a cheap runtime transform gives predictable
code size and cost, and keeps the DSP baffling out of the build.
Consequence: change filter parameters through the generator and regenerate; never
hand-edit a generated header.

### Host-side tools mirror the target path
Context: the signal chain must be checked without a radio on the bench.
Decision: host tests and scripts reproduce the same DSP steps and coefficient
data as the target, and are used as the reference for changes (for example, the
analog A/B of a new decimator against the previous one).
Why: verifying a filter or framing change against a reference on the host is
faster and repeatable, and catches regressions before hardware.
Consequence: when changing a generated block, update the host reference data and
compare, rather than trusting a by-eye check on the device.
