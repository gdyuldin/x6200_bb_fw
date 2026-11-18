# Development

Notes for working on the firmware: how the code is organised, how to build it,
and where to extend it.

## Repository layout

The code is split into layers, bottom-up. A lower layer never includes a higher
one, and only `Core/` and `Platform/` touch the STM32 HAL headers directly.

| Directory | Prefix | Role |
|---|---|---|
| `Core/` | — | CubeMX-generated HAL and clock/GPIO initialisation |
| `Platform/` | `bsp_` | Board support: UART, logging, I2C buses, GPIO, time, boot hand-off |
| `Devices/` | `drv_` | Chip drivers: PMIC, codec, IO expander, ADC monitor, FPGA, NVRAM |
| `App/` | `app_` | Application logic and the receive/transmit signal chains |

`App/SDR/` is a file group inside the application layer (the `sdr_` prefix),
compiled into the same target.

## Building

Requires the `arm-none-eabi` toolchain, CMake and Ninja:

```sh
cmake --preset Debug
cmake --build build/Debug
```

Use `--preset Release` for the release build. Debug enables the verbose log
levels; Release keeps only informational and above.

## Where to extend

| Task | Start here |
|---|---|
| Understand why a subsystem is built this way | [docs/design-notes.md](design-notes.md) |
| Add or change a host control | [docs/host-protocol.md](host-protocol.md) |
| Change the receive or transmit chain | [docs/rx-path.md](rx-path.md), [docs/tx-path.md](tx-path.md) |
| Change a protection threshold or behaviour | [docs/protection.md](protection.md) |

`docs/design-notes.md` records the recurring decisions and their reasoning
(layering, concurrency, host state, the FPGA bus and hardware primitives, the
signal chains, protections, power/watchdog/persistence, telemetry, logging,
generated coefficients). Read it before changing a subsystem, to learn which
invariants are load-bearing.

Two habits keep changes easy to review:

- **Logging** goes through the `LOG_*` macros, each file defining its own
  `LOG_TAG`. An event is logged once, at the layer that owns the decision.
- **Deferred work** runs from the scheduler, not from an interrupt handler. The
  interrupt hooks only copy bytes and latch flags.
