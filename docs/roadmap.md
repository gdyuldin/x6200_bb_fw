# Roadmap and ideas

Proposals for future work, published so the community can pick them up. Nothing
here is implemented. Each idea states its goal, the current state and the
extension points it would use, a feasibility analysis where one exists, and the
acceptance criteria a contribution has to meet.

Contributions follow the same rules as the rest of the project: see
[developer.md](developer.md) for the layout and build, and
[design-notes.md](design-notes.md) for the invariants a change must respect.
Open an issue before a large change so the approach can be agreed first.

## Audio "insert" over the USART

### Goal

Use the UART that currently carries telemetry for a host round-trip of audio,
with a band of up to 3 kHz in each direction:

- RX insert: the demodulated audio is sent to the host; the host does the final
  decoding and sends the result back; the firmware plays it on the speaker.
- TX insert: the microphone audio is sent to the host; the host forms the
  transmit signal and sends it back; the firmware uses it in the modulator.

The intent is to let the host own the final decode and encode while the firmware
keeps the RF path, so a more capable host can run better algorithms without new
firmware.

### Current state and extension points

- The telemetry UART is `USART2` (`BSP_UART_AUX`): 1 500 000 baud, 8N1, FIFO,
  asynchronous TX DMA and circular RX.
- Telemetry framing already reserves a COBS mode: `App_Telemetry_SetFraming`
  (`App/app_telemetry.h`) accepts `APP_TELEMETRY_FRAMING_COBS` but returns
  `-ENOTSUP` today; implementing it is one of the pieces.
- The RX chain already has a 12 kHz rate (`SDR_RX_RATE_12K`) and the audio-block
  infrastructure; the insert path taps and injects at the audio boundary rather
  than inside a demodulator.

### Bandwidth analysis

150 000 bytes/s are available per direction (1.5 Mbaud, ten bits per byte).
Per direction, mono, as a fraction of one direction:

| Sample rate | int16 | 8-bit | int24 |
|---|---|---|---|
| 8 kHz | 16 kB/s (11 %) | 8 kB/s (5 %) | 24 kB/s (16 %) |
| 12 kHz | 24 kB/s (16 %) | 12 kB/s (8 %) | 36 kB/s (24 %) |
| 16 kHz | 32 kB/s (21 %) | 16 kB/s (11 %) | 48 kB/s (32 %) |
| 24 kHz | 48 kB/s (32 %) | 24 kB/s (16 %) | 72 kB/s (48 %) |
| 48 kHz | 96 kB/s (64 %) | 48 kB/s (32 %) | 144 kB/s (96 %) |

A 3 kHz band needs at least a 6 kHz Nyquist rate; 8 kHz is the classic voice
rate and 12 kHz is the comfortable choice — its transition band is 3..6 kHz and
it matches an existing rate. COBS framing costs about one byte per 254 (~0.4 %).

### Format and rate

- **int16 linear PCM, little-endian**, is recommended: 96 dB SNR, trivial to
  pack, no exponent, unambiguous for a host.
- 8-bit mu-law halves the rate for speech-like signals if bandwidth ever matters;
  it is companding, so it fits the TX path less well.
- **bfloat16 is not suitable for audio**: eight mantissa bits give about 48 dB of
  relative precision, worse than int16, and convert none of the audio's fixed
  range into anything useful. fp16 is better than bfloat16 but still has no
  advantage over int16 for a source that is effectively 16-bit.
- int24 only if a very high spurious-free range is needed on TX (for example SSB
  opposite-sideband suppression); at a 3 kHz band int16 is normally enough, at
  +50 % bytes.
- Rate: 12 kHz as the default, 8 kHz if the host prefers the classic voice rate.

### Coexistence with telemetry

Telemetry (MCU to host) shares one direction with the uplink audio. With the
packet layout in `App/app_telemetry.h`:

| Telemetry | Packet | At 25 Hz | MCU to host with int16 @ 12 kHz |
|---|---|---|---|
| 512 bins | 552 B | 13.8 kB/s | 24.0 + 13.8 = 37.8 kB/s (25 %) |
| 1024 bins | 1064 B | 26.6 kB/s | 24.0 + 26.6 = 50.6 kB/s (34 %) |

Both directions together are about 50 kB/s. This fits a full-duplex link
comfortably; it also fits a half-duplex link (about 50 % utilisation) but then
needs frame arbitration and buffering, because audio must not be dropped while
telemetry may.

### Design sketch

- Add a framing mode for the audio stream (COBS) with a small per-frame header
  and a length field; the direction is implicit in the wire. Audio should take
  priority over telemetry on the shared direction.
- RX insert taps the demodulated mono block before it is scaled to the DAC; TX
  insert injects host audio where the modulator would read the microphone. Both
  hook at the audio boundary, not inside a demodulator.
- Block size sets the round-trip latency: a 10 ms block at 12 kHz is 240 bytes,
  about 1.6 ms each way on the wire, and the host's own processing dominates. The
  block size is the main latency lever.
- In insert mode the firmware's demodulator/modulator work is reduced to
  transport, so CPU load and the telemetry/spectrum duty cycle should be
  re-checked on hardware.

### Open questions

- Is the host link truly full-duplex, or is a half-duplex protocol needed?
- What round-trip latency is acceptable, and what block size delivers it?
- How much telemetry (bins and rate) to keep while insert is active?
- Is int16 precision acceptable on TX, or is int24 needed?

### Acceptance criteria

Build passes, and on hardware: int16 audio at 12 kHz round-trips in both
directions with telemetry at 25 Hz and 1024 bins, without disturbing the audio.
