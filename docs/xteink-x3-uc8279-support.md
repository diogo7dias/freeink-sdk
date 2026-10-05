# Xteink X3 — UC8279d controller variant

Newer X3 production units (Xteink heads-up, July 2026) ship the same ESP32-C3
board and 792×528 glass with a **UC8279d** panel controller in place of the
UC8253. Everything else — pinout, ADC ladder input, BQ27220/DS3231/QMI8658
peripherals, SD wiring — is unchanged. No serial or date cutover is known; the
boot log says which one a unit has. The variant has its own sibling profile,
`BoardConfig::XTEINK_X3_UC8279` (`Board::XteinkX3Uc8279`).

Build: nothing new — `-DFREEINK_DEVICE_X3=1` links both X3 drivers
(`FREEINK_DRIVER_UC8253_X3` and `FREEINK_DRIVER_UC8279`); which one runs is
decided at boot.

## Runtime detection

After the X3 I2C fingerprint, `detectX3DisplayController()` runs the shared
UC81xx probe (`probeDisplayController()` in `XteinkDetect.cpp`) over the X3
display pins (SCLK 8 / SDA 10 / CS 21 / DC 4 / RST 5 / BUSY 6). It bit-bangs a
half-duplex read: the command goes out with DC low, then SDA is released to
input (pull-up) and the controller shifts each bit out on the SCLK falling
edge. Each pass resets the controller, then reads **FLG (0x71)** and five bytes
of **VER (0x70)**.

- A pass matches when FLG is a driven idle status (not `0x00`/`0xFF`, `BUSY_N`
  bit 0 set) and VER is non-uniform. No `CHIP_VER` value is required: a shipping
  X4 Pro UC8179 answered `00 00 01 FF FF`.
- Pass 1 screens with a 1 ms reset; on the X3 (UC8253 profile active) a miss is
  retried once with the vendor's 50 ms reset. Pass 2 repeats the read, with the
  50 ms reset when pass 1 matched. Both must match and agree byte for byte.
- When FLG is driven, **RMTP (0xA2)** dumps the MTP header (one dummy byte,
  then MTP from `0x000`). A unit whose VER reads all `FF` is still accepted when
  the MTP starts with the `0xA5` key, which only a programmed UC81xx returns.
- A UC8253 has none of these reads: SDA floats to its pull-up, both passes
  miss, and the UC8253 stays selected. Disagreeing passes are inconclusive and
  also keep the UC8253.

What a UC8279 unit logs at boot:

```
[XTDET] bus probe VER=00 00 66 00 00 FLG=13 -> UltraChip
[XTDET] promoted UC8253 -> UC8279
```

A UC8253 unit logs `-> default controller` and no promotion. `NVS hw_calib/screenType`
is logged for reference only; a full flash from another unit can overwrite it.

**Upstream differs.** Free-Ink `main` replaced this matcher with the stock
V6.3.15 protocol (RESET high 10 ms / low 50 ms / high 50 ms, wait up to 300 ms
for BUSY, read three VER bytes; byte 3 `0x66` = UC8279, `0xFF` = UC8253, FLG and
MTP not read). That change is not merged into this branch.

## Driver — `Uc8279Driver`

KW mode: 1-bpp, DTM1 = OLD plane, DTM2 = NEW plane, differential refresh — the
same paradigm as the UC8253 X3 driver, with a near-identical command set
(PSR/PON/POF, DTM1 `0x10`, DSP `0x11`, DRF `0x12`, DTM2 `0x13`, CDI `0x50`,
PTIN/PTL/PTOUT `0x91`/`0x90`/`0x92`, DSLP `0x07`+`0xA5`). BUSY uses the X3
two-phase wait (`BusyPolarity::X3TwoPhase`).

The module's MTP is blank, so the driver programs it explicitly with the
initialization and waveforms recovered from stock firmware
(`lut/Uc8279X3Luts.h`):

- `PSR REG=1` (external LUTs); the init script sets power, booster, PLL and a
  792×528 partial window. Every RAM write and refresh runs inside that window
  (PTIN + 9-byte PTL) to avoid the controller's native 800×600 stride; there is
  no TRES.
- B/W refreshes use the recovered GC (full) and DU (fast) banks, loaded into
  `0x20`–`0x24` before each refresh. Both diff against DTM1, which is re-synced
  to the shown frame after every refresh. The first two content paints after
  boot are forced to GC so the splash cannot ghost through.
- CDI is `0x97` on the first refresh after init, `0xD7` after.
- Grayscale: 4-level AA via the XTF_AA tables (LSB to DTM1, MSB to DTM2), with
  the XTF_PRE_BW_MID conditioning pass (`CCSET 0xE0` / `TSSET 0xE5`) before it.
  Whole-plane and strip uploads are supported (`supportsStripGrayscale()`).
- `displayStart()` / `displayFinish()` split refreshes are supported.

Tone separation and refresh quality still need checking on hardware.

## Simulator

The lector QEMU simulator models this variant as board `x3uc8279`
(`-global uc8253.uc8279=on`). It answers the boot probe with the values
above (VER `00 00 66 00 00`, FLG `13`, MTP key `A5`), so the firmware promotes to
the UC8279 driver; the X3 e2e suite runs on both X3 panels and checks which
controller was chosen. It does not model the UC8279's LUTs or gray look.

## Useful UC8279 features not yet wired

- **AUTO (0x17)**: `PON→DRF→POF(→DSLP)` as one command — could shave host
  round-trips on sleepy ESL-style updates.
- **PBC (0x44)**: panel-break check via the CHKGI/CHKGO wire loop, if the
  module bonds it.
- **CRC (0x72)**: MTP integrity check over `0x000–0xFFF`.
- On-chip temperature readback (**TSC 0x40**) if the consumer ever wants it.
