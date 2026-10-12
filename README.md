# X6200 base-band firmware

**English** | [Русский](README.ru.md)

## Disclaimer

This is an independent, unofficial firmware project for the base-band board of
the X6200 transceiver. It is **not affiliated with, authorized by, endorsed by,
or connected to Xiegu** in any way. "Xiegu" and "X6200" are trademarks of their
respective owners and are used here only to describe hardware compatibility.

Flashing or running this firmware is done **entirely at your own risk**. The
software is provided **"as is", without warranty of any kind**, express or
implied. The authors and contributors accept **no liability** for any damage to
your radio or other hardware, for data loss, or for any legal or regulatory
consequences.

You are solely responsible for operating within the radio regulations of your
country, including transmit privileges, band plans and power limits. Use a dummy
load or a suitable antenna, and mind RF exposure.

## What is this

A from-scratch replacement for the original base-band firmware of the X6200 base
board (an STM32H725 microcontroller with an FPGA in the RF path). It runs the
receive and transmit signal chains, the power and temperature protections, the
automatic antenna tuner control, and the interface to the host (the radio's GUI
application).

The FPGA is treated as a black box: the firmware streams baseband I/Q to and
from it and configures tuning and sample rate over a parallel bus. The RF
down-conversion, up-conversion and sample-rate decimation happen inside the FPGA.

## Features

- **RX**: LSB, USB (including digital voice modes), CW/CWR, AM/SAM, NFM (with
  selectable 15 / 10 / 7 kHz bandwidth profiles) and WFM demodulation, with
  channel filtering, noise blanker, spectral noise reduction, notch, AGC and
  squelch, plus a spectrum/waterfall stream for the host.
- **TX**: SSB, CW, AM/SAM and NFM modulation, with an audio compressor,
  automatic level control (per-band level, fast peak protection, reflected-power
  clamp and external-ALC back-off), PA bias temperature compensation, and
  host-driven tune carriers for the antenna tuner and SWR scan. NFM deviation
  follows the selected RX bandwidth profile; the 7 kHz profile transmits flat,
  without pre-emphasis.
- **Protections**: battery and PA temperature, cell/pack voltage and
  state-of-charge, VSWR/reflected power, and a portable power-management and
  safe-shutdown sequence.
- **Control**: I2C command interface from the GUI application and a UART
  telemetry/waterfall link back to it.
- **Service**: a serial console with calibration modes for PA bias, TX level and
  the audio compressor.

## Not implemented yet

This is a technical preview. A few settings the host GUI can send are accepted
over the control interface but are currently ignored by the firmware:

- **Equalizer** — the five-band equalizers for RX, WFM and the microphone
  (`rxeq`, `rxeqwfm`, `miceq`). The values are received, but no equalization is
  applied.
- **VOX** — voice-operated transmit settings (gain, anti-vox, delay, enable).
  VOX does not key the transmitter.
- **AGC fine settings** — AGC knee, slope, hang and time constant. Only the AGC
  speed selection is used.
- **Per-mode RX filter presets** — the firmware stores a single RX filter
  window and applies the edges the host sends; it does not keep separate
  SSB/CW/AM/WFM windows. For NFM the window width selects the bandwidth profile
  (see the RX features).

Everything else the host can set — VFOs, mode, AGC speed, RF gain, TX power,
volume, RX filter window, squelch, noise blanker, noise reduction, notch,
compressor, keyer, RIT/XIT, ATU, PA bias, split, voice recorder, SWR scan and
charger — is supported.

## Documentation

| Document | Contents |
|---|---|
| [docs/hardware.md](docs/hardware.md) | Board overview and the blocks that matter for the RF path |
| [docs/rx-path.md](docs/rx-path.md) | Receive branch: supported demodulations and the DSP chain |
| [docs/tx-path.md](docs/tx-path.md) | Transmit branch: modes, ALC, PA and tune carriers |
| [docs/protection.md](docs/protection.md) | Temperature, voltage, VSWR and power protections |
| [docs/calibration.md](docs/calibration.md) | Console calibration modes and how to run them |
| [docs/host-protocol.md](docs/host-protocol.md) | Host I2C protocol, register map and how to add a control |
| [docs/developer.md](docs/developer.md) | Repository layout, building and where to extend |

## Releases

Pre-built firmware binaries are published on the GitHub Releases page.
<!-- TODO(author): add the Releases link once the first release is published. -->

## Building

Requires the `arm-none-eabi` toolchain, CMake and Ninja:

```sh
cmake --preset Debug
cmake --build build/Debug
```

Use `--preset Release` for the release build. See
[docs/developer.md](docs/developer.md) for the repository layout and where to
extend the firmware.

## License

This project is distributed under the GNU General Public License v3.0. See
[LICENSE.txt](LICENSE.txt).
