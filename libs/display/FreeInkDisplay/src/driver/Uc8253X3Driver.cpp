#include "Uc8253X3Driver.h"

#include <BoardConfig.h>

#include "../lut/Uc8253X3Luts.h"
#include "Uc8253X3Cmd.h"

namespace freeink {

const Uc8253X3Config& uc8253X3DefaultConfig() {
  static const Uc8253X3Config cfg = {
      {lut_x3_vcom_normal, lut_x3_ww_normal, lut_x3_bw_normal, lut_x3_wb_normal, lut_x3_bb_normal},
      {lut_x3_vcom_half, lut_x3_ww_half, lut_x3_bw_half, lut_x3_wb_half, lut_x3_bb_half},
      {lut_x3_vcom_fast, lut_x3_ww_fast, lut_x3_bw_fast, lut_x3_wb_fast, lut_x3_bb_fast},
      {lut_x3_vcom_full, lut_x3_ww_full, lut_x3_bw_full, lut_x3_wb_full, lut_x3_bb_full},
      {lut_x3_vcom_gc, lut_x3_ww_gc, lut_x3_bw_gc, lut_x3_wb_gc, lut_x3_bb_gc},
      {lut_x3_vcom_aa_pre_bw_mid, lut_x3_ww_aa_pre_bw_mid, lut_x3_bw_aa_pre_bw_mid, lut_x3_wb_aa_pre_bw_mid,
       lut_x3_bb_aa_pre_bw_mid},
      42,  // controller accepts 42 bytes of each 43-byte array
  };
  return cfg;
}

// Resolution comes from the active BoardProfile (XTEINK_X3), exactly like every
// other driver — the X3 profile is selected at runtime by setDisplayX3() before
// begin() constructs this singleton, so ACTIVE already holds 792x528 here.
Uc8253X3Driver::Uc8253X3Driver(const Uc8253X3Config& cfg)
    : _cfg(cfg),
      _w(BoardConfig::ACTIVE.displayWidth),
      _h(BoardConfig::ACTIVE.displayHeight),
      _wb(BoardConfig::ACTIVE.displayWidth / 8),
      _bufferSize(static_cast<uint32_t>(BoardConfig::ACTIVE.displayWidth / 8) * BoardConfig::ACTIVE.displayHeight) {}

uint32_t Uc8253X3Driver::spiHz() const {
  // X3 (UC8253) runs the SPI bus at 16 MHz.
  return BoardConfig::ACTIVE.displaySpiHz != 0 ? BoardConfig::ACTIVE.displaySpiHz : 16000000;
}

PanelGeometry Uc8253X3Driver::geometry() const { return {_w, _h, _wb, _bufferSize}; }

void Uc8253X3Driver::loadBank(EpdBus& bus, const Uc8253LutBank& bank) {
  bus.cmdData(CMD_LUT_VCOM, bank.vcom, _cfg.lutLen);
  bus.cmdData(CMD_LUT_WW, bank.ww, _cfg.lutLen);
  bus.cmdData(CMD_LUT_BW, bank.bw, _cfg.lutLen);
  bus.cmdData(CMD_LUT_WB, bank.wb, _cfg.lutLen);
  bus.cmdData(CMD_LUT_BB, bank.bb, _cfg.lutLen);
}

void Uc8253X3Driver::loadBankCdi(EpdBus& bus, uint8_t cdi0, uint8_t cdi1, const Uc8253LutBank& bank) {
  bus.cmdData2(CMD_VCOM_DATA_INTERVAL, cdi0, cdi1);
  loadBank(bus, bank);
}

// waitBusy() and waitRefreshComplete() both return on the first LOW->HIGH edge, and the
// X3 raises BUSY between the phases of its non-differential waveforms. So a caller that
// only waits once can carry on while the panel is still driving, and the writes it makes
// next land on a live waveform.
//
// Measured: a HALF that internally ran a full sync ended its post-work with BUSY low
// (diagnostic bit 5), and the FAST after it reported BUSY low before it had written a
// single byte (bit 2) and ran 513 ms of waveform where a warm X3 FAST needs 566 ms,
// leaving the frame underneath visible.
//
// No-op on a panel that really has finished, which is every differential pass.
void Uc8253X3Driver::waitPanelIdle(EpdBus& bus) {
  if (digitalRead(bus.pins().busy) == LOW) _lastDiagnostic |= kBusyLowAfterWait;
  // Idle is BUSY staying high, not BUSY going high once. Every wait in this driver
  // returns on a single rising edge, which is why moving one earlier or later never
  // helped: the panel drops BUSY again between the phases of a waveform, and it also
  // drops it while it digests a plane write. Requiring the line to hold high for a
  // window is the only test that separates "finished" from "between phases".
  const uint16_t waited = bus.waitIdleStable(kIdleStableMs, kBusyDrainTimeoutMs);
  _lastSettleWaitMs = static_cast<uint16_t>(_lastSettleWaitMs + waited);
  if (waited > kIdleStableMs + 2) _lastDiagnostic |= kSettleWaited;
}

void Uc8253X3Driver::triggerRefresh(EpdBus& bus, bool turnOff) {
  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" X3_PON");
    _isScreenOn = true;
  }
  bus.cmd(CMD_DISPLAY_REFRESH);
  bus.waitBusy(" X3_DRF");
  waitPanelIdle(bus);
  if (turnOff) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" X3_POF");
    _isScreenOn = false;
  }
}

void Uc8253X3Driver::initController(EpdBus& bus) {
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(0x3F);
  bus.data(0x0A);
  bus.cmd(CMD_RESOLUTION);
  bus.data(0x03);
  bus.data(0x18);
  bus.data(0x02);
  bus.data(0x58);
  bus.cmd(CMD_GATE_SOURCE_START);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);
  bus.cmd(CMD_POWER_OFF_SEQ);
  bus.data(0x20);
  bus.cmd(CMD_POWER_SETTING);
  bus.data(0x07);
  bus.data(0x17);
  bus.data(0x3F);
  bus.data(0x3F);
  bus.data(0x17);
  bus.cmd(CMD_VCOM_DC);
  bus.data(0x24);
  bus.cmd(CMD_BOOSTER_SOFT_START);
  bus.data(0x25);
  bus.data(0x25);
  bus.data(0x3C);
  bus.data(0x37);
  bus.cmd(CMD_PLL_CONTROL);
  bus.data(0x09);
  bus.cmd(CMD_LV_SELECTION);
  bus.data(0x02);

  // UC8253 has no auto-write RAM clear (unlike SSD1677); fill both planes white
  // so the first differential refresh diffs against white, not stale content.
  bus.fillPlane(CMD_DTM1, 0xFF, _h, _wb);
  bus.cmd(CMD_DATA_STOP);
  bus.fillPlane(CMD_DTM2, 0xFF, _h, _wb);
  bus.cmd(CMD_DATA_STOP);

  _isScreenOn = false;
}

void Uc8253X3Driver::begin(EpdBus& bus) {
  bus.reset(50);  // X3 needs an extra settle after reset
  _redRamSynced = false;
  // One forced clean after begin(), matching the UC8279/SSD1677 siblings' one-shot
  // _needFullClear. This was 2, which made the paint AFTER the first one a full sync too,
  // whatever mode it asked for (see the doFullSync condition in displayStart). On a
  // consumer that boots into a splash that is measurable: the splash consumed sync #1 and
  // the first real screen still paid sync #2, so a FAST refresh that costs 435 ms once
  // warm cost 2989 ms — the X4 running the identical boot paid 526 ms for the same paint.
  // The remaining sync still clears whatever the panel physically held (the sleep screen),
  // which is what the initial forced clean is for.
  _initialFullSyncsRemaining = 1;
  _forceFullSyncNext = false;
  _forcedConditionPassesNext = 0;
  _inGrayscaleMode = false;
  _grayState = {};
  // The panel holds whatever the last session left on it and the controller's RAM does
  // not describe it, so the first paint after begin() drives every pixel (see displayStart).
  _promoteFirstPaint = true;
  initController(bus);
}

void Uc8253X3Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  displayStart(bus, fb, prev, mode, turnOff);
  displayFinish(bus, fb);
}

bool Uc8253X3Driver::displayStart(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  (void)prev;
  // First paint after begin(): the panel's content is unknown to the controller (deep-sleep
  // wake, silent reboot, sleep face still up), so take the scrub that drives every pixel.
  //
  // This used to key on !_isScreenOn instead. That flag is also false after every idle
  // powerOff() (main.cpp drops the rails 2 s after the last input), and a page is read for
  // longer than that, so once idle power-off was restored on the X3 every page turn became
  // a half scrub: 24 frames of same-rail drive on every pixel plus the 200 ms settle, slower
  // than the differential it replaced, invisible to DisplayRefreshPolicy (which still counted
  // the pass as FAST) and never a discharge. The panel keeps its image with the rails down, so
  // a paint after idle is an ordinary differential; displayStart() re-powers the pump.
  if (_promoteFirstPaint && !turnOff) {
    mode = RefreshMode::Half;
    _promoteFirstPaint = false;
  }
  if (_inGrayscaleMode) {
    grayscaleRevert(bus, fb);
  }

  // Probed before this refresh writes a single byte, so it separates "the previous pass
  // is still driving the panel" from "our own LUT and plane writes pulled BUSY low".
  _lastDiagnostic = (digitalRead(bus.pins().busy) == LOW) ? kBusyLowOnEntry : 0;
  _lastSettleWaitMs = 0;

  const bool fastMode = (mode == RefreshMode::Fast);
  const bool halfMode = (mode == RefreshMode::Half);
  const bool driveAll = _driveAllNext && fastMode;
  _driveAllNext = false;
  const bool forcedFullSync = _forceFullSyncNext;
  const bool doFullSync =
      (!fastMode && !halfMode) || !_redRamSynced || _initialFullSyncsRemaining > 0 || forcedFullSync;
  const bool doHalfSync = halfMode && !doFullSync;
  _grayState.lastBaseWasPartial = !doFullSync;

  if (doFullSync) {
    // _full OEM bank with DTM1 == DTM2 == target: every pixel is an "unchanged" cell, and
    // those are the two cells that flash. WW runs black 24 frames then white; BB runs
    // white 24 frames then black. Every pixel therefore crosses both rails whatever the
    // panel held, in the same ~62 frames (a LUT's length is its frame count, not the cell).
    //
    // DTM1 used to be filled white. Then only white targets (WW) ever flashed, and every
    // black target took WB, black 20 frames from wherever it was and never the white rail:
    // text pixels and the dark part of a wallpaper were never discharged by the full or by
    // the clean the reader requests every N pages, and carried net black DC besides.
    loadBankCdi(bus, 0x29, 0x07, _cfg.full);
    bus.sendPlaneFlipped(CMD_DTM1, fb, _h, _wb);
    bus.cmd(CMD_DATA_STOP);
    bus.sendPlaneFlipped(CMD_DTM2, fb, _h, _wb);
  } else if (doHalfSync) {
    // _half scrub: WW==BW, WB==BB -> drive every pixel to target ignoring DTM1.
    loadBankCdi(bus, 0xA9, 0x07, _cfg.half);
    bus.sendPlaneFlipped(CMD_DTM2, fb, _h, _wb);
  } else {
    // _fast turbo differential; DTM1 retains the previous frame.
    loadBankCdi(bus, 0x29, 0x07, _cfg.fast);
    if (_darkBackground || driveAll) {
      // Inverted content: a differential fast idles unchanged pixels, so the
      // light residue of every white->black transition parks in the black
      // background and accumulates between full syncs. Rewrite DTM1 as the
      // complement of the target: every pixel classifies as changed and is
      // re-driven toward its target — optically invisible on pixels already at
      // their endpoint. displayFinish()'s DTM1 resync restores the true
      // baseline afterwards. Same mechanism as the SSD1677/Paper Mono drivers'
      // dark-background paths.
      bus.sendPlaneFlippedInverted(CMD_DTM1, fb, _h, _wb);
      bus.cmd(CMD_DATA_STOP);
    }
    bus.sendPlaneFlipped(CMD_DTM2, fb, _h, _wb);
  }

  // doFullSync re-powers the charge pump even if already on (higher current).
  if (!_isScreenOn || doFullSync) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" X3_PON");
    _isScreenOn = true;
  }
  bus.cmd(CMD_DISPLAY_REFRESH);
  // Confirm the waveform actually started (BUSY dropped LOW) before handing the
  // CPU back, so displayFinish()'s waitBusy() only rides out the second
  // (LOW->HIGH) phase. Short timeout: a missed edge just falls through to the
  // full two-phase wait in displayFinish().
  {
    const int8_t busyPin = bus.pins().busy;
    const unsigned long t0 = millis();
    while (digitalRead(busyPin) == HIGH && millis() - t0 < 50) delay(1);
    if (digitalRead(busyPin) == HIGH) _lastDiagnostic |= kAssertionNotSeen;
  }
  _pendingTurnOff = turnOff;
  _pendingDoFullSync = doFullSync;
  _pendingFastMode = fastMode;
  _pendingRefresh = true;
  return true;
}

void Uc8253X3Driver::displayFinish(EpdBus& bus, const uint8_t* fb) {
  if (!_pendingRefresh) return;
  _pendingRefresh = false;
  const bool turnOff = _pendingTurnOff;
  const bool doFullSync = _pendingDoFullSync;
  const bool fastMode = _pendingFastMode;

  // ISR-backed wait: displayStart() already confirmed BUSY dropped LOW, so the
  // waveform is running and waitRefreshComplete() will wake on the exact
  // completion edge rather than polling at 1 ms granularity.
  bus.waitRefreshComplete(" X3_DRF");

  // The X3 drives BUSY in more than one pulse for its non-differential waveforms, and the
  // wait above returns on the first high edge. The panel can still be working, and
  // everything below -- the DTM1 resync, and then the next refresh's plane writes and its
  // trigger -- would land on a live waveform. Measured: the FAST after a HALF reported
  // BUSY already low at its trigger and ran 204 ms of waveform where an X3 FAST needs
  // 566 ms, leaving the frame underneath visible.
  //
  // So wait out any further activity HERE, before this refresh's own post-work, rather
  // than in front of the next trigger. Waiting there is too late: that refresh's planes
  // have already been written to a busy controller by then.
  waitPanelIdle(bus);

  if (turnOff) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" X3_POF");
    _isScreenOn = false;
  }

  if (!fastMode) delay(200);

  uint8_t postConditionPasses = 0;
  if (doFullSync) {
    if (_forceFullSyncNext) postConditionPasses = _forcedConditionPassesNext;
    else if (_initialFullSyncsRemaining == 1) postConditionPasses = 1;
  }
  if (postConditionPasses > 0) {
    const uint16_t xEnd = static_cast<uint16_t>(_w - 1);
    const uint16_t yEnd = static_cast<uint16_t>(_h - 1);
    const uint8_t w[9] = {0x00, 0x00, static_cast<uint8_t>(xEnd >> 8), static_cast<uint8_t>(xEnd & 0xFF),
                          0x00, 0x00, static_cast<uint8_t>(yEnd >> 8), static_cast<uint8_t>(yEnd & 0xFF),
                          0x01};
    loadBankCdi(bus, 0xA9, 0x07, _cfg.normal);  // _normal: OEM normal loader CDI 0xA9
    // The pass is a hard re-drive of the black pixels: DTM1 white against the frame in DTM2
    // makes each of them a W->B transition. The full sync above leaves DTM1 == frame (every
    // cell unchanged), which would turn this pass into two idle cells, so put the white
    // baseline back for it. DTM1 is rewritten from the frame right after.
    bus.fillPlane(CMD_DTM1, 0xFF, _h, _wb);
    bus.cmd(CMD_DATA_STOP);
    for (uint8_t i = 0; i < postConditionPasses; i++) {
      bus.cmd(CMD_PARTIAL_IN);
      bus.cmdData(CMD_PARTIAL_WINDOW, w, 9);
      bus.sendPlaneFlipped(CMD_DTM2, fb, _h, _wb);
      bus.cmd(CMD_PARTIAL_OUT);
      triggerRefresh(bus, false);
    }
  }

  // Sync DTM1 ("old" RAM) with the current frame for the next fast diff.
  bus.sendPlaneFlipped(CMD_DTM1, fb, _h, _wb);
  bus.cmd(CMD_DATA_STOP);
  // A 52 KB plane write leaves the controller busy, and until this drain nothing waited
  // for it. That is what handed a busy panel to the next refresh: its completion wait
  // latched the rising edge of THIS write finishing, returned after 133 ms where a warm
  // differential needs 566 ms, and its own waveform then ran while the host was already
  // writing the next frame's planes. The frame underneath survived, which is the ghost.
  waitPanelIdle(bus);
  // Both DTM planes now hold a BW frame, not grayscale planes, so the next
  // displayGrayscaleBase() can take the differential happy path. Without this
  // clear, lsbValid stays true after any grayscale page and pins
  // cleanBaseNeeded on forever.
  _grayState.lsbValid = false;
  _redRamSynced = true;

  // First differential after a full garbles on X3; spend a no-op fast settle of
  // the just-displayed frame so the caller's next diff is clean.
  if (doFullSync) {
    loadBankCdi(bus, 0x29, 0x07, _cfg.fast);
    bus.sendPlaneFlipped(CMD_DTM2, fb, _h, _wb);
    triggerRefresh(bus, turnOff);
    bus.sendPlaneFlipped(CMD_DTM1, fb, _h, _wb);
    bus.cmd(CMD_DATA_STOP);
  }

  if (doFullSync && _initialFullSyncsRemaining > 0) {
    _initialFullSyncsRemaining--;
  }
  _forceFullSyncNext = false;
  _forcedConditionPassesNext = 0;

  // Final probe: whatever state this pass hands to the next one.
  if (digitalRead(bus.pins().busy) == LOW) _lastDiagnostic |= kBusyLowAfterPost;
}

void Uc8253X3Driver::requestResync(uint8_t settlePasses) {
  _forceFullSyncNext = true;
  _forcedConditionPassesNext = settlePasses;
}

void Uc8253X3Driver::skipInitialResync() {
  _initialFullSyncsRemaining = 0;
  _redRamSynced = true;
}

// Drops the rails without DEEP_SLEEP (lector's 2 s idle powerOffPanel); the paint path
// powers back on when the flag is clear. An idle panel left powered drifts into speckle.
void Uc8253X3Driver::powerOff(EpdBus& bus) {
  if (!_isScreenOn) return;
  bus.cmd(CMD_POWER_OFF);
  bus.waitBusy(" X3 power-down");
  _isScreenOn = false;
}

void Uc8253X3Driver::deepSleep(EpdBus& bus) {
  powerOff(bus);
  bus.cmd(CMD_DEEP_SLEEP);
  bus.data(0xA5);
}

// Per-board waveform/LUT injection: a board that drives a different UC8253 panel
// (different LUTs) supplies its own config without editing this driver — define
// `const Uc8253X3Config& yourConfig();` in namespace freeink and build with
// -DFREEINK_UC8253_X3_CONFIG=yourConfig. Resolution is orthogonal: it always
// comes from that board's BoardProfile. A panel that also differs in init or
// rotation should be its own sibling driver instead.
#ifdef FREEINK_UC8253_X3_CONFIG
const Uc8253X3Config& FREEINK_UC8253_X3_CONFIG();
static const Uc8253X3Config& uc8253X3ActiveConfig() { return FREEINK_UC8253_X3_CONFIG(); }
#else
static const Uc8253X3Config& uc8253X3ActiveConfig() { return uc8253X3DefaultConfig(); }
#endif

PanelDriver& uc8253X3Driver() {
  static Uc8253X3Driver instance(uc8253X3ActiveConfig());
  return instance;
}

}  // namespace freeink
