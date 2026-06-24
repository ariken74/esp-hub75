// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file fm6126a.cpp
// @brief FM6126A/ICN2038S shift driver initialization

// Based on https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA

#include "driver_init.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "panels/scan_patterns.h"
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <initializer_list>

namespace hub75 {

static const char *const TAG = "DriverInit";
static constexpr uint16_t FM63XX_CMD_VSYNC = 3;
static constexpr uint16_t FM63XX_CMD_EN_OP = 12;
static constexpr uint16_t FM63XX_CMD_PRE_ACT = 14;

#define CLK_PULSE \
  gpio_set_level((gpio_num_t) pins.clk, 1); \
  gpio_set_level((gpio_num_t) pins.clk, 0);

void DriverInit::fm6126a_init(const Hub75Pins &pins, uint16_t pixels_per_row) {
  ESP_LOGI(TAG, "Initializing FM6126A shift driver (pixels_per_row=%d)", pixels_per_row);
  ESP_LOGI(TAG, "Pins: R1=%d G1=%d B1=%d R2=%d G2=%d B2=%d CLK=%d LAT=%d OE=%d A=%d B=%d C=%d D=%d E=%d",
           pins.r1, pins.g1, pins.b1, pins.r2, pins.g2, pins.b2,
           pins.clk, pins.lat, pins.oe, pins.a, pins.b, pins.c, pins.d, pins.e);

  // Control register values
  static constexpr bool REG1[16] = {false, false, false, false, false, true,  true,  true,
                                    true,  true,  true,  false, false, false, false, false};  // Global brightness
  static constexpr bool REG2[16] = {false, false, false, false, false, false, false, false,
                                    false, true,  false, false, false, false, false, false};  // Enable output

  // 1. Configure all pins as GPIO output
  for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2, pins.clk, pins.lat, pins.oe}) {
    gpio_reset_pin((gpio_num_t) pin);
    gpio_set_direction((gpio_num_t) pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) pin, 0);
  }

  // 2. Disable display (OE high)
  gpio_set_level((gpio_num_t) pins.oe, 1);

  // 3. Send REG1 (latch at pixels_per_row - 12)
  for (int i = 0; i < pixels_per_row; i++) {
    for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
      gpio_set_level((gpio_num_t) pin, REG1[i % 16]);
    }
    if (i > pixels_per_row - 12) {
      gpio_set_level((gpio_num_t) pins.lat, 1);
    }
    CLK_PULSE;
  }
  gpio_set_level((gpio_num_t) pins.lat, 0);

  // 4. Send REG2 (latch at pixels_per_row - 13)
  for (int i = 0; i < pixels_per_row; i++) {
    for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
      gpio_set_level((gpio_num_t) pin, REG2[i % 16]);
    }
    if (i > pixels_per_row - 13) {
      gpio_set_level((gpio_num_t) pins.lat, 1);
    }
    CLK_PULSE;
  }
  gpio_set_level((gpio_num_t) pins.lat, 0);

  // 5. Blank display data
  for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
    gpio_set_level((gpio_num_t) pin, 0);
  }
  for (int i = 0; i < pixels_per_row; i++) {
    CLK_PULSE;
  }

  // 6. Latch and enable display
  gpio_set_level((gpio_num_t) pins.lat, 1);
  CLK_PULSE;
  gpio_set_level((gpio_num_t) pins.lat, 0);
  gpio_set_level((gpio_num_t) pins.oe, 0);
  CLK_PULSE;

  ESP_LOGI(TAG, "FM6126A initialized successfully");
}

static void set_all_rgb(const Hub75Pins &pins, uint8_t level) {
  for (int8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
    gpio_set_level((gpio_num_t) pin, level);
  }
}

static void configure_shift_pins(const Hub75Pins &pins) {
  for (int8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2, pins.clk, pins.lat, pins.oe,
                     pins.a, pins.b, pins.c, pins.d, pins.e}) {
    if (pin < 0) {
      continue;
    }

    gpio_reset_pin((gpio_num_t) pin);
    gpio_set_direction((gpio_num_t) pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) pin, 0);
  }
}

static void send_latches(const Hub75Pins &pins, uint16_t latches) {
  set_all_rgb(pins, 0);
  gpio_set_level((gpio_num_t) pins.clk, 0);
  gpio_set_level((gpio_num_t) pins.lat, 1);

  for (uint16_t i = 0; i < latches; i++) {
    CLK_PULSE;
  }

  gpio_set_level((gpio_num_t) pins.lat, 0);
}

static void fm63xx_vsync(const Hub75Pins &pins) {
  set_all_rgb(pins, 0);
  gpio_set_level((gpio_num_t) pins.clk, 0);
  send_latches(pins, FM63XX_CMD_VSYNC);
}

static uint16_t fm63xx_scan_lines_for_config(const Hub75Config &config) {
  return get_effective_num_rows(config.scan_wiring, config.panel_height);
}

static void send_to_all_rgb(const Hub75Pins &pins, uint16_t pixels_per_row, uint16_t data, uint16_t latches) {
  const uint16_t latch_start = (pixels_per_row > latches) ? (pixels_per_row - latches) : 0;

  for (uint16_t i = 0; i < pixels_per_row; i++) {
    if (i == latch_start) {
      gpio_set_level((gpio_num_t) pins.lat, 1);
    }

    const bool bit = (data << (i % 16)) & 0x8000;
    set_all_rgb(pins, bit ? 1 : 0);
    CLK_PULSE;
  }

  gpio_set_level((gpio_num_t) pins.lat, 0);
  set_all_rgb(pins, 0);
}

static void set_row_address(const Hub75Pins &pins, uint16_t row) {
  if (pins.a >= 0) gpio_set_level((gpio_num_t) pins.a, (row >> 0) & 1);
  if (pins.b >= 0) gpio_set_level((gpio_num_t) pins.b, (row >> 1) & 1);
  if (pins.c >= 0) gpio_set_level((gpio_num_t) pins.c, (row >> 2) & 1);
  if (pins.d >= 0) gpio_set_level((gpio_num_t) pins.d, (row >> 3) & 1);
  if (pins.e >= 0) gpio_set_level((gpio_num_t) pins.e, (row >> 4) & 1);
}

// ICN2053-family init (FM6565 ≈ ICN2065 ≈ ICN2053, per Colorlight chip table).
// Reference: SebiTimeWaster/ICN2053_ESP32_LedWall (config originally from LEDVISION).
// cfg[] slot order: [0]=DEBUG(LE2) [1]=CFG1(LE4) [2]=CFG2(LE6) [3]=CFG3(LE8) [4]=CFG4(LE10).
// Working direct init order: for each register, PRE_ACT, 16 blank clocks,
// register data with its command latch, then 16 blank clocks.
static void icn2053_init(const Hub75Pins &pins, uint16_t pixels_per_row, const uint16_t (&cfg)[5]) {
  auto send_blank_clocks = [&pins](uint16_t count) {
    set_all_rgb(pins, 0);
    while (count--) {
      CLK_PULSE;
    }
  };

  const struct {
    uint16_t data;
    uint16_t command;
  } regs[] = {
      {cfg[1], 4},   // CFG1
      {cfg[2], 6},   // CFG2
      {cfg[3], 8},   // CFG3
      {cfg[4], 10},  // CFG4
      {cfg[0], 2},   // DEBUG
  };

  for (const auto &reg : regs) {
    send_latches(pins, FM63XX_CMD_PRE_ACT);
    send_blank_clocks(16);
    send_to_all_rgb(pins, pixels_per_row, reg.data, reg.command);
    send_blank_clocks(16);
  }
}

struct Fm63xxDiagnosticState {
  Hub75Pins pins;
  uint16_t scan_lines;
  uint16_t chips_per_row;
};

static Fm63xxDiagnosticState g_fm63xx_diag_state = {};

// GCLK is EXTERNAL on the OE pin: exactly 138 pulses per data block drive the
// ICN2053/FM6565 internal PWM + SRAM address advance. This count must be exactly 138.
static constexpr uint16_t FM63XX_GCLK_PWM = 138;

static void send_pwm_clock(const Hub75Pins &pins, uint16_t n) {
  while (n--) {
    gpio_set_level((gpio_num_t) pins.oe, 1);
    gpio_set_level((gpio_num_t) pins.oe, 0);
  }
}

// One full-white frame using the same row-address/OE cadence as the working
// ICN2053 suffix: for each row address, send 138 OE/GCLK pulses, two idle
// GCLK words, then shift one full-white 16-bit grayscale payload per chip.
static void icn2053_refresh_white(const Hub75Pins &pins, uint16_t scan_lines, uint16_t chips) {
  send_latches(pins, FM63XX_CMD_VSYNC);

  for (uint16_t row = 0; row < scan_lines; row++) {
    set_row_address(pins, row);
    send_pwm_clock(pins, FM63XX_GCLK_PWM);
    gpio_set_level((gpio_num_t) pins.oe, 0);
    CLK_PULSE;
    CLK_PULSE;

    for (uint16_t chip = 0; chip < chips; chip++) {
      for (uint8_t bit = 0; bit < 16; bit++) {
        set_all_rgb(pins, 1);
        if (chip == chips - 1 && bit == 15) {
          gpio_set_level((gpio_num_t) pins.lat, 1);
        }
        CLK_PULSE;
      }
      gpio_set_level((gpio_num_t) pins.lat, 0);
    }
  }
}

static void fm63xx_white_diagnostic_task(void *arg) {
  auto *state = static_cast<Fm63xxDiagnosticState *>(arg);
  ESP_LOGW(TAG, "ICN2053/FM6565 refresh: %u scan lines, %u chips/row, 138 GCLK on OE, full-white",
           state->scan_lines, state->chips_per_row);

  while (true) {
    icn2053_refresh_white(state->pins, state->scan_lines, state->chips_per_row);
    vTaskDelay(1);  // yield to avoid watchdog reset
  }
}

void DriverInit::fm63xx_spwm_init(const Hub75Config &config, const char *driver_name,
                                   const uint16_t (&base_config)[5]) {
  const Hub75Pins &pins = config.pins;
  const uint16_t pixels_per_row = config.panel_width * config.layout_cols;
  const uint16_t scan_lines = get_effective_num_rows(config.scan_wiring, config.panel_height);

  // CFG1 upper byte encodes scan_lines-1 (e.g. 32 scan -> 0x1F70).
  uint16_t cfg[5] = {
      base_config[0],
      static_cast<uint16_t>(((scan_lines - 1) << 8) | (base_config[1] & 0x00ff)),
      base_config[2],
      base_config[3],
      base_config[4],
  };

  ESP_LOGI(TAG, "Initializing %s (ICN2053-family) S-PWM driver (pixels_per_row=%d, scan_lines=%d)", driver_name,
           pixels_per_row, scan_lines);
  ESP_LOGI(TAG, "%s pins: R1=%d G1=%d B1=%d R2=%d G2=%d B2=%d CLK/DCLK=%d LAT/LE=%d OE/GCLK=%d A=%d B=%d C=%d D=%d E=%d",
           driver_name, pins.r1, pins.g1, pins.b1, pins.r2, pins.g2, pins.b2, pins.clk, pins.lat, pins.oe, pins.a,
           pins.b, pins.c, pins.d, pins.e);
  ESP_LOGI(TAG, "%s config: CFG1=0x%04x CFG2=0x%04x CFG3=0x%04x CFG4=0x%04x DEBUG=0x%04x", driver_name,
           cfg[1], cfg[2], cfg[3], cfg[4], cfg[0]);

  configure_shift_pins(pins);
  gpio_set_level((gpio_num_t) pins.oe, 0);

  icn2053_init(pins, pixels_per_row, cfg);

  gpio_set_level((gpio_num_t) pins.oe, 0);
  ESP_LOGI(TAG, "%s S-PWM initialization complete", driver_name);
}

bool DriverInit::start_fm63xx_white_diagnostic(const Hub75Config &config) {
  const uint16_t pixels_per_row = config.panel_width * config.layout_cols;

  g_fm63xx_diag_state.pins = config.pins;
  g_fm63xx_diag_state.scan_lines = fm63xx_scan_lines_for_config(config);
  g_fm63xx_diag_state.chips_per_row = std::max<uint16_t>(1, (pixels_per_row + 15) / 16);

  BaseType_t ok = xTaskCreatePinnedToCore(fm63xx_white_diagnostic_task, "fm63xx_diag", 4096, &g_fm63xx_diag_state,
                                          tskIDLE_PRIORITY + 1, nullptr, tskNO_AFFINITY);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "FM63xx diagnostic: failed to create refresh task");
    return false;
  }

  ESP_LOGW(TAG, "FM63xx diagnostic: ICN2053 refresh task running (%u scan lines, %u chips/row)",
           g_fm63xx_diag_state.scan_lines, g_fm63xx_diag_state.chips_per_row);
  return true;
}

void DriverInit::dp3246_init(const Hub75Pins &pins, uint16_t pixels_per_row) {
  // TODO: Port from reference library when hardware available
  ESP_LOGW(TAG, "DP3246 initialization not yet implemented");
}

esp_err_t DriverInit::initialize(const Hub75Config &config) {
  uint16_t pixels_per_row = config.panel_width * config.layout_cols;
  // Config slot order follows command LE lengths: [0]=DEBUG/2, [1]=CFG1/4, [2]=CFG2/6, [3]=CFG3/8, [4]=CFG4/10.
  static constexpr uint16_t FM6353_CONFIG[5] = {0x0008, 0x1f70, 0x6707, 0x40f7, 0x0040};
  static constexpr uint16_t FM6363_CONFIG[5] = {0x7e08, 0x0fb0, 0xe79d, 0x60b6, 0x5a70};
  // ICN2053-family config (FM6565 ≈ ICN2065), matching the LEDVISION-derived
  // defaults in LAutour/ESP32-HUB75-MatrixPanel-I2S-DMA-icn2053.
  // CFG1 upper byte is patched with scan_lines-1 below.
  static constexpr uint16_t ICN2053_CONFIG[5] = {0x0008, 0x0070, 0x7ddb, 0x4047, 0x0e40};

  switch (config.shift_driver) {
    case Hub75ShiftDriver::GENERIC:
      return ESP_OK;

    case Hub75ShiftDriver::FM6126A:
    case Hub75ShiftDriver::ICN2038S:
      fm6126a_init(config.pins, pixels_per_row);
      return ESP_OK;

    case Hub75ShiftDriver::DP3246:
      dp3246_init(config.pins, pixels_per_row);
      return ESP_OK;

    case Hub75ShiftDriver::FM6353:
      fm63xx_spwm_init(config, "FM6353", FM6353_CONFIG);
      return ESP_OK;

    case Hub75ShiftDriver::FM6363:
      fm63xx_spwm_init(config, "FM6363", FM6363_CONFIG);
      return ESP_OK;

    case Hub75ShiftDriver::FM6565C:
      // FM6565 ≈ ICN2065 ≈ ICN2053 family (per Colorlight chip table) — use ICN2053 config + protocol.
      fm63xx_spwm_init(config, "FM6565C", ICN2053_CONFIG);
      return ESP_OK;

    case Hub75ShiftDriver::MBI5124:
      ESP_LOGW(TAG, "MBI5124: Ensure clk_phase_inverted is set to true");
      return ESP_OK;

    case Hub75ShiftDriver::FM6124:
      ESP_LOGW(TAG, "FM6124 initialization not yet implemented");
      return ESP_ERR_NOT_SUPPORTED;

    default:
      ESP_LOGW(TAG, "Unknown shift driver: %d", (int) config.shift_driver);
      return ESP_ERR_NOT_SUPPORTED;
  }
}

}  // namespace hub75
