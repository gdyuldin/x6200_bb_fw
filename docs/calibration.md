# Calibration modes

The firmware provides a serial console with a few factory/service calibration
modes. They are used to trim the PA bias, the transmit level and the audio
compressor, typically while watching an external instrument (for example an
ammeter for the bias currents).

## Reaching the console

The console runs on the debug serial port. In normal operation that port belongs
to the GUI application, which uses it for CAT control, so the console cannot be
reached until the GUI is stopped from owning the port. On the host, one known way
to do this is to hold the port-select line low:

```sh
watch -n 0.2 "echo 0 | tee /sys/class/gpio/gpio138/value > /dev/null"
```

Run this on the host, then open the serial port with a terminal program. Other
ways to release the port from the GUI are possible. Stop the command above when
you are done so the GUI can take the port back.

The console is key-driven: every key gives an immediate status line. The active
mode's data is only printed while that mode is entered, so leaving a mode also
silences its periodic output.

## Global keys

| Key | Action |
|---|---|
| `?` | Show the list of modes, or the active mode's keys |
| `q` or `Esc` | Leave the current mode |
| `o` | Enter the PA bias mode |
| `a` | Enter the ALC (transmit level) mode |
| `c` | Enter the audio compressor mode |

## PA bias mode (`o`)

| Key | Action |
|---|---|
| `d` / `f` | Select the drive or the final channel |
| `+` / `-` | Change the selected channel by one step (10 DAC codes) |
| `0` | Toggle the selected channel off |
| `t` | Toggle transmission with no signal (PA keyed, no modulation) |
| `s` | Save the current values to NVRAM |
| `p` | Print the current status |
| `q` | Leave the mode |

The starting values are read from the calibration data in NVRAM. Adjust them
against an external ammeter, keying the PA with `t` (the transmitter is driven
without a signal). The values are volatile until saved with `s`; saving writes
them to NVRAM.

## ALC (transmit level) mode (`a`)

| Key | Action |
|---|---|
| `+` / `-` | Change the band level by 0.5 dB |
| `0` | Restore the factory level for the current band |
| `>` / `<` | Change the measured-power correction by 0.5 dB |
| `=` | Restore the factory power correction for the current band |
| `p` | Print the current status |
| `q` | Leave the mode |

While transmitting, the mode prints the level, the learned transfer and slow
gain, the measured forward power, the setpoint and the PA temperature once per
second. The level adjusts the per-band drive; the power correction compensates
the frequency response of the coupler/detector so the displayed power matches the
true power on the higher bands.

## Audio compressor mode (`c`)

| Key | Action |
|---|---|
| `+` / `-` | Change the compressor threshold by 1 dB |
| `e` | Enable or disable the compressor |
| `0` | Restore the default threshold |
| `p` | Print the current status |
| `q` | Leave the mode |

While transmitting, the mode prints the input and output RMS levels, the applied
gain and the threshold once per second.

## Safety notes

- Calibration transmissions key the PA. Always use a dummy load or a suitable
  antenna, and mind RF exposure.
- Bias and level changes are held in RAM until saved. Only the bias mode has its
  own save key (`s`); the ALC level and power correction stay in RAM and are
  persisted through the host's calibration save path.
- Saving writes to the board EEPROM. Do not power off in the middle of a save.
