# Host protocol

The base board is an I2C slave to the radio's host application (the GUI). This
document describes the wire protocol, the register map, and how to add a new
control.

## Transport

The host link is I2C3, 7-bit address `0x72`. A transaction starts with the
register address, then the payload:

| Direction | Frame |
|---|---|
| write | `[0x72+W][reg 2 B big-endian][data]` |
| read | `[0x72+W][reg 2 B big-endian][Sr][0x72+R][data]` |

The register address is two bytes, big-endian. The payload is a raw
little-endian memory image: multi-byte fields are stored low byte first. A write
longer than one word auto-increments `reg`, so a whole block can be sent in one
transaction.

The write and read hooks run in the I2C interrupt, so they only copy bytes and
latch flags. Blocking work (saving calibration, power-off, entering the
bootloader) is deferred to the main loop through the scheduler.

## Base register map

### Read

| Range | Size | Contents |
|---|---|---|
| `0x0000..0x0080` | 129 B | identity strings (see below) |
| `0x0100..0x02FA` | 507 B | calibration block |
| `0x2000` | 1 B | ready flag, reads `1` |
| other | 1 B | one zero byte |

The identity block (`0x0000..0x0080`) holds zero-terminated strings, each up to
32 bytes: hardware version at `+0x01`, FPGA version at `+0x21`, firmware version
at `+0x41`, firmware date at `+0x61`.

### Write

| Range | Contents |
|---|---|
| `0x0000..0x00D7` | command block, one 32-bit word per command at `index*4` |
| `0xF000..0xF1FA` | calibration block, wraps modulo its size |
| `0xFFFE` | host command, 16-bit code (see below) |

Address `0x0000` is direction-dependent: a write is the command block, a read is
the identity block. The meaning of each command word is defined by the handlers
and index enum at the top of `App/app_hostlink.c`.

### Host commands (`0xFFFE`)

| Code | Action |
|---|---|
| `0x8002` | reset AGC statistics |
| `0x8003` | dump calibration summary to the log |
| `0x8004` | start an ATU tune |
| `0x8005` | save the calibration block |
| `0x8009` | power off |
| `0x800A` | enter the bootloader (IAP) |

## Extended page

The base map has no spare room for new controls, so the firmware adds a separate
page for commands the base protocol does not carry. The page is **write-only**:
nothing is read back from it. It lives outside the base ranges so the two never
collide.

| Range | Contents |
|---|---|
| `0x4000..0x4007` | magic string `R2RFE_FW`, opens the page |
| `0x4008` | layout version, 32-bit |
| `0x4100..0x41FF` | command block, one 32-bit word per command at `index*4` |

The page is closed until the magic string is written: until the eight bytes of
`0x4000..0x4007` match, every command word is ignored. A write to `0x4000` can
also carry the version word in the same transaction.

The magic is compared byte by byte, so it is independent of the payload's
endianness. The version word records the layout revision; increment it when a
change is not purely additive, so the host can tell the layouts apart.

## Adding a control

A control is one 32-bit word in the extended command block. The base command
block in `App/app_hostlink.c` implements the same pattern (a word-index enum, a
handler table, and a shadow diff in the main-loop task) and is a good reference.

1. **Reserve a word.** Pick the next free index in the extended command block and
   fix its address at `0x4100 + index*4`. The block holds
   `HOSTLINK_EXT_CMD_WORDS` words.
2. **Write a handler.** Add a `static void cmd_xxx(uint32_t value)` in
   `App/app_hostlink_ext.c` and unpack the word into fields with explicit masks
   and shifts, in the same style as the existing handlers.
3. **Register it.** Put the handler in the matching slot of the handler table
   `s_knob_handlers` at that index. Unused slots stay `0` and are skipped.
4. **Apply the setting.** Call the subsystem that owns the value. For a radio
   setting, write through the `App_RadioState_*` setters; for a value that must
   survive a power cycle, route it through the matching `App_Nvram_*` call. The
   transport layer is not touched.
5. **Drive it from the host.** Send the magic once, then write the word — either
   on its own by addressing it, or together with the rest of the block from
   `0x4100`.
6. **Build.**

The command block is dispatched from the main loop, not the interrupt: the write
hook only records the changed word.

## Conventions

- One 32-bit word per control. Pack fields low-to-high: the lowest bit field
  occupies the lowest bits. Document the masks next to the handler.
- Keep the extended page additive. Only a non-additive layout change needs the
  version word bumped.
- A control added to the extended page never affects the base host GUI: without
  the magic the page stays closed, and until then the base protocol behaves
  exactly as before.
