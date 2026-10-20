// Uc8253X3Driver: the 4-level grayscale / anti-aliasing paths (base, precondition, plane writes,
// gray refresh, cleanup, revert). Split from Uc8253X3Driver.cpp, which keeps the black-and-white
// refresh paths; same class.

#include <BoardConfig.h>

#include "Uc8253X3Cmd.h"
#include "Uc8253X3Driver.h"

namespace freeink {

void Uc8253X3Driver::displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
  // OEM V5.6.33 grayscale base update: write the new frame to DTM2 and fire
  // the "AA-pre-BW(mid)" bank as a differential refresh against the old frame
  // still held in DTM1. Changed pixels get the strong 0xAA/0x55 transition
  // drives, unchanged pixels the gentle 0x20/0x10 reinforcement -- leaving
  // the whole region in the calibrated state the gray nudge bank expects.
  // When the controller state cannot support a clean differential (DTM1
  // unsynced after AA, boot full-syncs pending, or an explicit resync
  // request), fall back to the normal display path and follow it with the
  // settle flavor of the same bank (DTM1 == DTM2 after display()'s post-
  // refresh sync, so only the gentle WW/BB cells fire).
  if (_inGrayscaleMode) {
    // grayscaleRevert scrubs the panel to white and leaves BOTH DTM planes
    // all-white with _redRamSynced set, so DTM1 matches the displayed state
    // and the differential below is valid by construction (white-baseline,
    // the same pattern the full sync uses).
    grayscaleRevert(bus, fb);
  }
  // _grayState.lsbValid means grayscale planes were written over DTM1/DTM2
  // since the last display — the controller RAM no longer holds the displayed
  // BW frame even though _redRamSynced may still read true, so the
  // differential would mis-drive; take the clean fallback path instead.
  const bool cleanBaseNeeded =
      !_redRamSynced || _grayState.lsbValid || _forceFullSyncNext || _initialFullSyncsRemaining > 0;
  if (cleanBaseNeeded) {
    display(bus, fb, nullptr, fallback, /*turnOff=*/false);
    loadBankCdi(bus, 0xA9, 0x07, _cfg.preBwMid);
    triggerRefresh(bus, turnOff);
    return;
  }
  bus.sendPlaneFlipped(CMD_DTM2, fb, _h, _wb);
  loadBankCdi(bus, 0xA9, 0x07, _cfg.preBwMid);
  triggerRefresh(bus, turnOff);
  // Keep the driver invariant that DTM1 mirrors the displayed frame; the gray
  // plane writes that normally follow overwrite both planes anyway.
  bus.sendPlaneFlipped(CMD_DTM1, fb, _h, _wb);
  bus.cmd(CMD_DATA_STOP);
  _redRamSynced = true;
}

void Uc8253X3Driver::preconditionGrayscale(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  // OEM V5.6.33 "AA-pre-BW(mid)" pass: gentle settle of the displayed BW
  // frame (DTM1 == DTM2 == frame after display()'s post-refresh DTM1 sync)
  // that leaves particles receptive to the weak grayscale nudge waveform.
  // Without it a strong base refresh sets pixels too firmly for the gray
  // drive to move. Windowed to the gray region via PTL exactly like the OEM
  // loader (PTIN -> window -> CDI/bank -> refresh -> PTOUT). The PTL Y range
  // is in GATE space (logical row y lives at gate H-1-y, see
  // writeGrayscalePlaneStrip); X is byte-aligned outward since PTL horizontal
  // resolution is 8 pixels.
  if (w == 0 || h == 0 || x >= _w || y >= _h) return;
  // The settle is only meaningful (and only safe) when both DTM planes hold
  // the displayed BW frame. Skip when grayscale planes have been written over
  // them (lsbValid), a grayscale refresh left the RAM unsynced, or the gray
  // bank is still loaded — firing the mid bank's strong BW/WB drives against
  // gray-coded state pairs would corrupt the region.
  if (_inGrayscaleMode || !_redRamSynced || _grayState.lsbValid) return;
  const uint16_t xEndLogical = static_cast<uint16_t>(((x + w - 1) < (_w - 1)) ? (x + w - 1) : (_w - 1));
  const uint16_t yEndLogical = static_cast<uint16_t>(((y + h - 1) < (_h - 1)) ? (y + h - 1) : (_h - 1));
  const uint16_t xs = static_cast<uint16_t>(x & ~7u);
  const uint16_t xe = static_cast<uint16_t>(xEndLogical | 7u);
  const uint16_t gateYStart = static_cast<uint16_t>((_h - 1) - yEndLogical);
  const uint16_t gateYEnd = static_cast<uint16_t>((_h - 1) - y);
  const uint8_t win[9] = {static_cast<uint8_t>(xs >> 8),
                          static_cast<uint8_t>(xs & 0xFF),
                          static_cast<uint8_t>(xe >> 8),
                          static_cast<uint8_t>(xe & 0xFF),
                          static_cast<uint8_t>(gateYStart >> 8),
                          static_cast<uint8_t>(gateYStart & 0xFF),
                          static_cast<uint8_t>(gateYEnd >> 8),
                          static_cast<uint8_t>(gateYEnd & 0xFF),
                          0x01};
  bus.cmd(CMD_PARTIAL_IN);
  bus.cmdData(CMD_PARTIAL_WINDOW, win, 9);
  loadBankCdi(bus, 0xA9, 0x07, _cfg.preBwMid);
  triggerRefresh(bus, /*turnOff=*/false);
  bus.cmd(CMD_PARTIAL_OUT);
}

void Uc8253X3Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  if (!lsb) {
    _grayState.lsbValid = false;
    return;
  }
  bus.sendPlaneFlipped(CMD_DTM1, lsb, _h, _wb);  // LSB plane -> "old" RAM
  bus.cmd(CMD_DATA_STOP);
  _grayState.lsbValid = true;
}

void Uc8253X3Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  if (!msb || !_grayState.lsbValid) return;
  bus.sendPlaneFlipped(CMD_DTM2, msb, _h, _wb);  // MSB plane -> "new" RAM
  bus.cmd(CMD_DATA_STOP);
}

void Uc8253X3Driver::writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                              uint16_t numRows) {
  if (!rows || numRows == 0) return;
  // PTL partial-window in GATE space (logical row y lives at gate H-1-y), rows
  // emitted bottom-first so they land at the same gates the full-frame write
  // uses. Fixes the AA/image banding from windowing in logical space.
  const uint8_t ramCmd = (plane == GrayPlane::Lsb) ? CMD_DTM1 : CMD_DTM2;
  const uint16_t xEnd = static_cast<uint16_t>(_w - 1);
  const uint16_t yEndLogical = static_cast<uint16_t>(yStart + numRows - 1);
  const uint16_t gateYStart = static_cast<uint16_t>((_h - 1) - yEndLogical);
  const uint16_t gateYEnd = static_cast<uint16_t>((_h - 1) - yStart);
  const uint8_t win[9] = {0x00,
                          0x00,
                          static_cast<uint8_t>(xEnd >> 8),
                          static_cast<uint8_t>(xEnd & 0xFF),
                          static_cast<uint8_t>(gateYStart >> 8),
                          static_cast<uint8_t>(gateYStart & 0xFF),
                          static_cast<uint8_t>(gateYEnd >> 8),
                          static_cast<uint8_t>(gateYEnd & 0xFF),
                          0x01};
  bus.cmd(CMD_PARTIAL_IN);
  bus.cmdData(CMD_PARTIAL_WINDOW, win, 9);
  bus.cmd(ramCmd);
  bus.beginTxn();
  for (int r = static_cast<int>(numRows) - 1; r >= 0; r--) {
    bus.rawWriteBytes(rows + static_cast<uint32_t>(r) * _wb, _wb);
  }
  bus.endTxn();
  bus.cmd(CMD_PARTIAL_OUT);
  if (plane == GrayPlane::Lsb) _grayState.lsbValid = true;
}

void Uc8253X3Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                                 bool factoryMode) {
  (void)fb;
  (void)lut;
  if (!_grayState.lsbValid) return;

  // Differential grayscale leaves the gray bank loaded, so the next BW turn must
  // revert first; factory absolute mode self-cleans.
  _inGrayscaleMode = !factoryMode;
  if (factoryMode) {
    // No dedicated absolute-grayscale bank on this driver — the OEM standalone
    // "X3灰阶" flow needs a different panel init (PSR/PWR/VCOM rails) and DTM
    // framing that isn't ported. Factory mode reuses the _full B/W bank.
    loadBankCdi(bus, 0x29, 0x07, _cfg.full);
  } else {
    loadBankCdi(bus, 0x29, 0x07, _cfg.gc);  // OEM 4-level nudge bank
  }
  triggerRefresh(bus, turnOff);

  _redRamSynced = false;
  _forceFullSyncNext = false;
  _forcedConditionPassesNext = 0;
  _grayState.lsbValid = false;
}

void Uc8253X3Driver::cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) {
  if (!bw) return;
  // Rebase both planes from the restored BW buffer (same data to DTM1 + DTM2).
  bus.sendPlaneFlipped(CMD_DTM2, bw, _h, _wb);
  bus.cmd(CMD_DATA_STOP);
  bus.sendPlaneFlipped(CMD_DTM1, bw, _h, _wb);
  bus.cmd(CMD_DATA_STOP);
  // Both planes now hold the rebased BW frame; this is the per-page cleanup the
  // tiled AA reader path runs, so leaving lsbValid true here is what pins
  // cleanBaseNeeded on in steady state.
  _grayState.lsbValid = false;
  _redRamSynced = true;
  _forceFullSyncNext = false;
  _forcedConditionPassesNext = 0;
  _inGrayscaleMode = false;
}

void Uc8253X3Driver::grayscaleRevert(EpdBus& bus, const uint8_t* fb) {
  (void)fb;
  if (!_inGrayscaleMode) return;
  _inGrayscaleMode = false;
  // Scrub to clean white: both planes white + the _half scrub bank (CDI 0xA9).
  bus.fillPlane(CMD_DTM1, 0xFF, _h, _wb);
  bus.cmd(CMD_DATA_STOP);
  bus.fillPlane(CMD_DTM2, 0xFF, _h, _wb);
  bus.cmd(CMD_DATA_STOP);
  loadBankCdi(bus, 0xA9, 0x07, _cfg.half);
  triggerRefresh(bus, false);
  // Both planes are now all-white (BW-coded), not grayscale planes — clear
  // lsbValid to match, or it stays true forever and forces the cleanBaseNeeded
  // path every page.
  _grayState.lsbValid = false;
  _redRamSynced = true;
}

}  // namespace freeink
