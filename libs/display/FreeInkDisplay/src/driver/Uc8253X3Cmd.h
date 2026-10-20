#pragma once

// UC8253 command bytes shared by the X3 driver's two translation units.

#include <stdint.h>

namespace freeink {
namespace {
// UC8253 command set.
constexpr uint8_t CMD_PANEL_SETTING = 0x00;
constexpr uint8_t CMD_POWER_SETTING = 0x01;
constexpr uint8_t CMD_POWER_OFF = 0x02;
constexpr uint8_t CMD_POWER_OFF_SEQ = 0x03;
constexpr uint8_t CMD_POWER_ON = 0x04;
constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x06;
constexpr uint8_t CMD_DEEP_SLEEP = 0x07;
constexpr uint8_t CMD_DTM1 = 0x10;
constexpr uint8_t CMD_DATA_STOP = 0x11;
constexpr uint8_t CMD_DISPLAY_REFRESH = 0x12;
constexpr uint8_t CMD_DTM2 = 0x13;
constexpr uint8_t CMD_LUT_VCOM = 0x20;
constexpr uint8_t CMD_LUT_WW = 0x21;
constexpr uint8_t CMD_LUT_BW = 0x22;
constexpr uint8_t CMD_LUT_WB = 0x23;
constexpr uint8_t CMD_LUT_BB = 0x24;
constexpr uint8_t CMD_PLL_CONTROL = 0x30;
constexpr uint8_t CMD_VCOM_DATA_INTERVAL = 0x50;
constexpr uint8_t CMD_RESOLUTION = 0x61;
constexpr uint8_t CMD_GATE_SOURCE_START = 0x65;
constexpr uint8_t CMD_VCOM_DC = 0x82;
constexpr uint8_t CMD_LV_SELECTION = 0xE1;
constexpr uint8_t CMD_PARTIAL_WINDOW = 0x90;
constexpr uint8_t CMD_PARTIAL_IN = 0x91;
constexpr uint8_t CMD_PARTIAL_OUT = 0x92;
}  // namespace
}  // namespace freeink
