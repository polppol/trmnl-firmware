#include <Arduino.h>
#include <JPEGDEC.h>
#include <PNGdec.h>
#include <Preferences.h>
#include <WiFi.h>
#include <board_config.h>
#include <color_reduce.h>
#include <display.h>
#include <display_internal.h>
#include <display_widgets.h>
#include <epd_panel_rev.h>
#include <jpeg_render.h>
#include <png_render.h>
#include <power.h>
#include <preferences_persistence.h>
#include <refresh_interval.h>

#include "battery.h"
#include "config.h"
#include "messages.h"
#ifdef PARALLEL_EPD
FASTEPD bbep;
#else
BBEPAPER bbep;
#endif // !PARALLEL_EPD

#ifdef PARALLEL_EPD
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

// clang-format off
const uint8_t u8_graytable[] = {
/* 0 */  0, 0, 0, 0, 0, 0, 1, 1, 1, 
/* 1 */  0, 0, 1, 1, 1, 2, 2, 1, 1, 
/* 2 */  0, 0, 0, 0, 1, 2, 2, 1, 1,
/* 3 */  1, 1, 2, 2, 1, 1, 1, 1, 2, 
/* 4 */  0, 0, 0, 1, 2, 1, 1, 1, 2, 
/* 5 */  1, 2, 2, 2, 2, 1, 1, 1, 2, 
/* 6 */  0, 0, 1, 1, 2, 2, 1, 1, 2, 
/* 7 */  0, 1, 1, 2, 1, 1, 2, 1, 2, 
/* 8 */  0, 1, 1, 1, 2, 1, 2, 1, 2, 
/* 9 */  0, 1, 1, 1, 1, 2, 2, 1, 2, 
/* 10 */  1, 1, 1, 2, 1, 1, 1, 2, 2, 
/* 11 */  0, 0, 1, 2, 1, 1, 1, 2, 2, 
/* 12 */  0, 0, 0, 1, 2, 1, 1, 2, 2, 
/* 13 */  0, 0, 0, 0, 1, 2, 1, 2, 2, 
/* 14 */  0, 1, 1, 1, 2, 2, 2, 2, 2, 
/* 15 */  0, 0, 0, 0, 0, 0, 0, 0, 2
};
// clang-format on
#endif

#include <api-client/display.h>
#include <config.h>
#include <ctype.h> //iscntrl()
#include <globals.h>
#include <inttypes.h>
#include <trmnl_log.h>

#include "Group5.h"
#include "bmp.h"
#include "png_flip.h"

#ifdef BOARD_SEEED_RETERMINAL_E1002
#include "driver/gpio.h"
#include "driver/spi_master.h"

//
// E1002 Spectra 6 display update using ESP-IDF SPI driver.
// Arduino SPI (bb_epaper) does NOT work on this hardware — requires
// SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY flags.
// Reads bb_epaper's 4bpp framebuffer and sends via ESP-IDF SPI.
//
static spi_device_handle_t s_epd_spi = NULL;
static bool s_spectra6_buf_is_1bpp = false;

static void spectra6_spi_init(void) {
  if (s_epd_spi) return; // already initialized

  // GPIO setup
  gpio_config_t out_conf = {};
  out_conf.pin_bit_mask =
      (1ULL << pDevice->epd_rst_pin) | (1ULL << pDevice->epd_dc_pin) | (1ULL << pDevice->epd_cs_pin);
  out_conf.mode = GPIO_MODE_OUTPUT;
  out_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&out_conf);
  gpio_config_t in_conf = {};
  in_conf.pin_bit_mask = (1ULL << pDevice->epd_busy_pin);
  in_conf.mode = GPIO_MODE_INPUT;
  in_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&in_conf);
  gpio_set_level((gpio_num_t)pDevice->epd_cs_pin, 1);
  gpio_set_level((gpio_num_t)pDevice->epd_dc_pin, 0);

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = pDevice->epd_mosi_pin;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = pDevice->epd_sck_pin;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 4096;
  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Log_info("spectra6: spi_bus_init failed: %s", esp_err_to_name(ret));
    return;
  }

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 20 * 1000 * 1000;
  devcfg.mode = 0;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 1;
  devcfg.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
  spi_bus_add_device(SPI2_HOST, &devcfg, &s_epd_spi);
  Log_info("spectra6: SPI initialized");
}

static void spectra6_wait_busy(const char *label) {
  vTaskDelay(pdMS_TO_TICKS(10));
  int wc = 0;
  while (gpio_get_level((gpio_num_t)pDevice->epd_busy_pin) == 0) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (++wc > 6000) {
      Log_info("spectra6 [%s] BUSY timeout", label);
      return;
    }
  }
}

void spectra6_update(void) {
  Log_info("spectra6_update: start");
  spectra6_spi_init();
  if (!s_epd_spi) {
    Log_info("spectra6_update: SPI not available");
    return;
  }
  spi_device_handle_t spi_dev = s_epd_spi;

  // SPI helpers (matching esp32-photoframe)
  auto spi_wr = [&spi_dev](const uint8_t *data, int len) {
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(spi_dev, &t);
  };

  auto cmd_data = [&spi_dev, &spi_wr](uint8_t cmd, const uint8_t *data, size_t len) {
    gpio_set_level((gpio_num_t)pDevice->epd_dc_pin, 0);
    spi_device_acquire_bus(spi_dev, portMAX_DELAY);
    gpio_set_level((gpio_num_t)pDevice->epd_cs_pin, 0);
    spi_transaction_ext_t cmd_t = {};
    cmd_t.command_bits = 8;
    cmd_t.base.flags = SPI_TRANS_VARIABLE_CMD;
    cmd_t.base.cmd = cmd;
    spi_device_polling_transmit(spi_dev, &cmd_t.base);
    if (len > 0 && data) {
      gpio_set_level((gpio_num_t)pDevice->epd_dc_pin, 1);
      uint8_t buf[16];
      memcpy(buf, data, len);
      spi_wr(buf, len);
    }
    gpio_set_level((gpio_num_t)pDevice->epd_cs_pin, 1);
    spi_device_release_bus(spi_dev);
  };

  auto send_cmd = [&cmd_data](uint8_t cmd) { cmd_data(cmd, NULL, 0); };

  auto send_buffer = [&spi_dev, &spi_wr](const uint8_t *data, int len) {
    const uint8_t *ptr = data;
    int rem = len;
    while (rem > 0) {
      int chunk = (rem > 128) ? 128 : rem;
      gpio_set_level((gpio_num_t)pDevice->epd_dc_pin, 1);
      spi_device_acquire_bus(spi_dev, portMAX_DELAY);
      gpio_set_level((gpio_num_t)pDevice->epd_cs_pin, 0);
      spi_wr(ptr, chunk);
      gpio_set_level((gpio_num_t)pDevice->epd_cs_pin, 1);
      spi_device_release_bus(spi_dev);
      ptr += chunk;
      rem -= chunk;
    }
  };

  // 1. Hardware reset
  gpio_set_level((gpio_num_t)pDevice->epd_rst_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(50));
  gpio_set_level((gpio_num_t)pDevice->epd_rst_pin, 0);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level((gpio_num_t)pDevice->epd_rst_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(50));
  spectra6_wait_busy("reset");

  // 2. Init sequence (esp32-photoframe ED2208-GCA values)
  const uint8_t d_aa[] = {0x49, 0x55, 0x20, 0x08, 0x09, 0x18};
  const uint8_t d_01[] = {0x3F};
  const uint8_t d_00[] = {0x5F, 0x69};
  const uint8_t d_03[] = {0x00, 0x54, 0x00, 0x44};
  const uint8_t d_05[] = {0x40, 0x1F, 0x1F, 0x2C};
  const uint8_t d_06[] = {0x6F, 0x1F, 0x17, 0x49};
  const uint8_t d_08[] = {0x6F, 0x1F, 0x1F, 0x22};
  const uint8_t d_30[] = {0x03};
  const uint8_t d_50[] = {0x3F};
  const uint8_t d_60[] = {0x02, 0x00};
  const uint8_t d_61[] = {0x03, 0x20, 0x01, 0xE0};
  const uint8_t d_84[] = {0x01};
  const uint8_t d_e3[] = {0x2F};
  cmd_data(0xAA, d_aa, 6);
  cmd_data(0x01, d_01, 1);
  cmd_data(0x00, d_00, 2);
  cmd_data(0x03, d_03, 4);
  cmd_data(0x05, d_05, 4);
  cmd_data(0x06, d_06, 4);
  cmd_data(0x08, d_08, 4);
  cmd_data(0x30, d_30, 1);
  cmd_data(0x50, d_50, 1);
  cmd_data(0x60, d_60, 2);
  cmd_data(0x61, d_61, 4);
  cmd_data(0x84, d_84, 1);
  cmd_data(0xE3, d_e3, 1);
  spectra6_wait_busy("init");

  // 3. Send framebuffer data from bb_epaper (4bpp, 192000 bytes)
  send_cmd(0x10);
  uint8_t *fb = (uint8_t *)bbep.getBuffer();
  if (fb) {
    if (s_spectra6_buf_is_1bpp) {
      // Expand 48000-byte 1bpp buffer (1=white 0x1, 0=black 0x0) into 400-byte 4bpp rows
      uint8_t row[400];
      for (int r = 0; r < 480; r++) {
        const uint8_t *src = fb + r * 100;
        for (int i = 0; i < 100; i++) {
          uint8_t b = src[i];
          row[i * 4 + 0] = (((b >> 7) & 1) << 4) | ((b >> 6) & 1);
          row[i * 4 + 1] = (((b >> 5) & 1) << 4) | ((b >> 4) & 1);
          row[i * 4 + 2] = (((b >> 3) & 1) << 4) | ((b >> 2) & 1);
          row[i * 4 + 3] = (((b >> 1) & 1) << 4) | ((b >> 0) & 1);
        }
        send_buffer(row, 400);
      }
    } else {
      // bb_epaper 4bpp buffer: 800/2 * 480 = 192000 bytes
      send_buffer(fb, 192000);
    }
  } else {
    // No buffer — send all white
    uint8_t row[400];
    memset(row, 0x11, 400);
    for (int r = 0; r < 480; r++) send_buffer(row, 400);
  }
  s_spectra6_buf_is_1bpp = false;
  bbep.setMemoryMode(BB_MODE_NATIVE);
  spectra6_wait_busy("data");

  // 4. Power on
  send_cmd(0x04);
  spectra6_wait_busy("PON");

  // 5. Display refresh (15-30 seconds)
  const uint8_t d_00_single[] = {0x00};
  const uint8_t d_a5[] = {0xA5};
  cmd_data(0x12, d_00_single, 1);
  Log_info("spectra6_update: refreshing...");
  spectra6_wait_busy("refresh");

  // 6. Power off + deep sleep
  cmd_data(0x02, d_00_single, 1);
  spectra6_wait_busy("POFF");
  cmd_data(0x07, d_a5, 1);

  // SPI bus kept open for reuse across refreshes
  Log_info("spectra6_update: done");
}

// Draw 6-color indicator boxes on E1002 embedded screens (pure colors, 2-color blends, 3-color blends)
void spectra6_draw_color_boxes(void) {
  // bb_epaper color indices (NOT raw nibble values)
  const uint8_t colors[] = {BBEP_BLACK, BBEP_WHITE, BBEP_YELLOW, BBEP_RED, BBEP_BLUE, BBEP_GREEN};
  int box_w = 36, box_h = 28, gap = 6;
  int totalW = 6 * box_w + 5 * gap;
  int startX = (bbep.width() - totalW) / 2; // centered
  int startY = 10;

  // Row 1: Pure colors
  for (int c = 0; c < 6; c++) {
    int x = startX + c * (box_w + gap);
    bbep.fillRect(x, startY, box_w, box_h, colors[c]);
    if (c == 1) bbep.drawRect(x, startY, box_w, box_h, BBEP_BLACK);
  }

  // Row 2: Blended/dithered colors (checkerboard pattern)
  // Orange, Purple, Pink, LightBlue, LightGreen, Teal
  const uint8_t blendA[] = {BBEP_RED, BBEP_RED, BBEP_RED, BBEP_BLUE, BBEP_GREEN, BBEP_BLUE};
  const uint8_t blendB[] = {BBEP_YELLOW, BBEP_BLUE, BBEP_WHITE, BBEP_WHITE, BBEP_YELLOW, BBEP_GREEN};
  int row2Y = startY + box_h + gap;
  for (int c = 0; c < 6; c++) {
    int bx = startX + c * (box_w + gap);
    for (int py = 0; py < box_h; py++) {
      for (int px = 0; px < box_w; px++) {
        uint8_t col = ((px + py) & 1) ? blendB[c] : blendA[c];
        bbep.drawPixel(bx + px, row2Y + py, col);
      }
    }
  }

  // Row 3: 3-color blends (dithered, cycling through 3 colors)
  const uint8_t tri_a[] = {BBEP_RED, BBEP_RED, BBEP_YELLOW, BBEP_BLACK, BBEP_BLACK, BBEP_WHITE};
  const uint8_t tri_b[] = {BBEP_YELLOW, BBEP_BLUE, BBEP_BLUE, BBEP_WHITE, BBEP_GREEN, BBEP_GREEN};
  const uint8_t tri_c[] = {BBEP_BLUE, BBEP_GREEN, BBEP_GREEN, BBEP_RED, BBEP_YELLOW, BBEP_RED};
  int row3Y = row2Y + box_h + gap;
  for (int c = 0; c < 6; c++) {
    int bx = startX + c * (box_w + gap);
    for (int py = 0; py < box_h; py++) {
      for (int px = 0; px < box_w; px++) {
        uint8_t col;
        int idx = (px + py) % 3;
        if (idx == 0)
          col = tri_a[c];
        else if (idx == 1)
          col = tri_b[c];
        else
          col = tri_c[c];
        bbep.drawPixel(bx + px, row3Y + py, col);
      }
    }
  }
}
#endif // BOARD_SEEED_RETERMINAL_E1002

#ifdef BB_EPAPER
static uint8_t bLightSleepWanted = 0;

static void display_apply_light_sleep(uint8_t enabled) {
#ifdef CMD_CS1_CS2
  bbep.setLightSleep(enabled, pDevice->epd_en_pin);
#else
  bbep.setLightSleep(enabled);
#endif
}

bool display_update_epaper(int refreshMode, bool wait, bool writePlane, uint8_t plane) {
#ifdef BOARD_SEEED_RETERMINAL_E1002
  (void)refreshMode;
  (void)wait;
  (void)writePlane;
  (void)plane;
  spectra6_update();
  return true;
#else
#if defined(BOARD_SEEED_RETERMINAL_E1002) || defined(BOARD_SEEED_RETERMINAL_E1004)
    // Light sleep with the WiFi radio running is unsupported
  if (bLightSleepWanted && WiFi.getMode() != WIFI_MODE_NULL) {
    display_apply_light_sleep(false);
  } else {
    display_apply_light_sleep(bLightSleepWanted);
  }
#endif
  if (writePlane) {
    bbep.writePlane(plane);
  }
  if (refreshMode == REFRESH_PARTIAL && !bCanDoPartial) {
    refreshMode = REFRESH_FAST;
    Log_info("Can't do partial refresh, no previous image");
  } else {
    Log_info("Have valid previous image in EPD memory, doing partial refresh");
  }
  bbep.refresh(refreshMode, wait);
    // The next update can be a partial update because the current is 1-bpp and stays in the EPD RAM
  bCanDoPartial = (bbep.getPanelType() == dpList[pDevice->panel_set][iTempProfile].OneBit);
  return true;
#endif
}

static void display_select_one_bit_panel(bool only_if_changed = false) {
  bool custom_pins = pDevice->epd_mosi_pin != 0 || pDevice->epd_sck_pin != 0;
  int one_bit = dpList[pDevice->panel_set][custom_pins ? iTempProfile : 0].OneBit;
  if (only_if_changed && bbep.getPanelType() == one_bit) {
    return;
  }
  if (custom_pins) {
    bbep.setPanelType(one_bit);
#ifndef BOARD_SEEED_RETERMINAL_E1002
    bbep.initIO(pDevice->epd_dc_pin, pDevice->epd_rst_pin, pDevice->epd_busy_pin, pDevice->epd_cs_pin,
                pDevice->epd_mosi_pin, pDevice->epd_sck_pin, 8000000);
#endif
  } else {
    bbep.begin(one_bit);
  }
}

// Message screens are 1-bit (4-bpp native on E1002 so 6-color swatch boxes and spectra6_update work directly)
void display_prepare_message(void) {
  display_select_one_bit_panel(true);
#ifndef BOARD_SEEED_RETERMINAL_E1002
  bbep.setMemoryMode(BB_MODE_1BPP); // use 1-bit mode to save memory
#else
  s_spectra6_buf_is_1bpp = false;
  bbep.setMemoryMode(BB_MODE_NATIVE);
#endif
  bbep.allocBuffer(false);
}

#endif // BB_EPAPER

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void) {
  if (pDevice == NULL) {
    hw_config_init();
  }
  Log_info("dev module start");
  iTempProfile = preferences.getUInt(PREFERENCES_TEMP_PROFILE, TEMP_PROFILE_DEFAULT);
  Log_info("Saved temperature profile: %" PRIu32, iTempProfile);
#ifdef BB_EPAPER
  Log_info("BB e-Paper init");
#ifdef BOARD_SEEED_STICKY
// Special case for the Sticky - it shares the SPI bus with the uSD card, so the EPD CS line
// must be the only one active at a time
  pinMode(8, OUTPUT); // SD card CS (shared SPI with EPD)
  digitalWrite(8, 1); // SD CS disabled
  pinMode(47, OUTPUT);
  digitalWrite(47, 1); // enable EPD power
  pinMode(10, OUTPUT); // SD card enable (if it's powered down, the SPI bus may be blocked)
  digitalWrite(10, 1);
#endif // BOARD_SEEED_STICKY
#ifdef BOARD_SEEED_RETERMINAL_E1002
  if (pDevice->epd_en_pin != 0xff) {
    pinMode(pDevice->epd_en_pin, OUTPUT);
    digitalWrite(pDevice->epd_en_pin, HIGH);
    delay(20);
  }
#endif
#ifndef BOARD_SEEED_RETERMINAL_E1002
    // Read the panel ID after any board-specific power/CS setup, but before bb_epaper takes over the SPI pins.
    // Only on the first call: the bit-banged read detaches SCK/MOSI from the SPI peripheral, and
    // SPI.begin() is a no-op once the bus is running, so a second read would leave the panel unreachable.
  static bool panel_rev_read = false;
  if (!panel_rev_read) {
    panel_rev = get_panel_rev();
    panel_rev_read = true;
    Log_info("Panel ID = 0x%08x\n", panel_rev);
  }
#endif
  if (pDevice->epd_mosi_pin != 0 || pDevice->epd_sck_pin != 0) {
    bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].OneBit); // must be set BEFORE calling initio
#ifdef BOARD_SEEED_RETERMINAL_E1002
    spectra6_spi_init();
#else
    bbep.initIO(pDevice->epd_dc_pin, pDevice->epd_rst_pin, pDevice->epd_busy_pin, pDevice->epd_cs_pin,
                pDevice->epd_mosi_pin, pDevice->epd_sck_pin, 8000000);
#endif
  } else { // it's a pre-defined PCB+display in bb_epaper
    bbep.begin(dpList[pDevice->panel_set][0].OneBit);
  }
#else // Parallel eink devices
#if defined(BOARD_SEEED_RETERMINAL_E1003)
  bbep.initIT8951(EPD_MOSI_PIN, EPD_MISO_PIN, EPD_SCK_PIN, EPD_CS_PIN, EPD_BUSY_PIN, EPD_RST_PIN, EPD_EN_PIN,
                  EPD_VCC_EN);
  bbep.setPanelSize(BBEP_DISPLAY_ED103TC2);
#else // normal parallel eink devices
  bbep.initPanel(pDevice->iBoardType);
  if (pDevice->iPanelSize != BB_PANEL_NONE) {
    bbep.setPanelSize(pDevice->iPanelSize);
  }
#endif
#ifdef BOARD_TRMNL_X
//    bbep.initPanel(BB_PANEL_TRMNL_X);
  bbep.setPasses(3, 3); // The 10.3" panel needs fewer pushes
#endif
// #elif defined( BOARD_TRMNL_X_SENSORIAS3 )
//     bbep.initPanel(BB_PANEL_V7_RAW);
//     bbep.setPanelSize(1280, 720, BB_PANEL_FLAG_MIRROR_X, -1600);
// #elif defined( BOARD_TRMNL_X_SENSORIAC5 )
//     bbep.initPanel(BB_PANEL_SENSORIA_C5);
// #elif defined(BOARD_TRMNL_X_PAPERS3)
//     bbep.initPanel(BB_PANEL_M5PAPERS3);
// #elif defined(BOARD_TRMNL_X_LILYGO)
//     bbep.initPanel(BB_PANEL_EPDIY_V7);
//     bbep.setPanelSize(960, 540);
#endif // bb_epaper
  Log_info("dev module end");
}

/**
 * @brief Enable or disable light sleep at runtime
 * @param enabled true to enable light sleep, false to disable
 * @return none
 */
void display_set_light_sleep(uint8_t enabled) {
#ifdef BB_EPAPER
  bLightSleepWanted = enabled;
  display_apply_light_sleep(enabled);
#endif
}

// Clear any ghosting on the X display by wiping it black/white many times
void display_wipe(void) {
#ifdef BB_EPAPER

#ifdef BOARD_TRMNL_4CLR
  int refreshCount = 2;
#elif defined(BOARD_SEEED_RETERMINAL_E1002)
  int refreshCount = 2;
#else
  int refreshCount = 60;
#endif

  bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].OneBit);
  bbep.fillScreen(BBEP_WHITE);
  for (int i = 0; i < refreshCount; i++) {
#ifdef BOARD_SEEED_RETERMINAL_E1002
    spectra6_update();
#else
    bbep.refresh(REFRESH_FULL); // 2 to 3 minutes of Black/White clearing of the display
#endif
  }
#ifndef BOARD_SEEED_RETERMINAL_E1002
  bbep.sleep(LIGHT_SLEEP);
#endif
#else
  bbep.setMode(BB_MODE_1BPP);
  bbep.fillScreen(BBEP_WHITE);
  for (int i = 0; i < 100; i++) { // 200 black/white cycles should remove any ghosting
    bbep.fullUpdate(CLEAR_SLOW, true);
  }
  bbep.einkPower(0); // power off the display
#endif
} /* display_wipe() */

/**
 * @brief Function to sleep the ESP32 while saving power
 * @param u32Millis represents the sleep time in milliseconds
 * @return none
 */
void display_sleep(uint32_t u32Millis) {
#ifdef DO_NOT_LIGHT_SLEEP
  delay(u32Millis);
#else
    // Runtime control for light sleep (true = enabled, false = disabled)
  static bool light_sleep_enabled = true;
  if (!light_sleep_enabled) {
    delay(u32Millis);
  } else {
    esp_sleep_enable_timer_wakeup(u32Millis * 1000L);
    esp_light_sleep_start();
  }
#endif
}

/**
 * @brief Function to reset the display
 * @param none
 * @return none
 */
void display_reset(void) {
  Log_info("e-Paper Clear start");
  bbep.fillScreen(BBEP_WHITE);
#ifdef BB_EPAPER
  display_set_light_sleep(true);
  if (!display_update_epaper(apiDisplayResult.response.maximum_compatibility ? REFRESH_FULL : REFRESH_FAST, true)) {
    Log_error("display_reset: e-paper update failed");
  }
#else
  bbep.fullUpdate();
#endif
  Log_info("e-Paper Clear end");
    // DEV_Delay_ms(500);
}

/**
 * @brief Function to read the display height
 * @return uint16_t - height of display in pixels
 */
uint16_t display_height() { return bbep.height(); }

/**
 * @brief Function to read the display width
 * @return uint16_t - width of display in pixels
 */
uint16_t display_width() { return bbep.width(); }

// Draws a PNG, JPEG, Group5 or BMP image into the panel driver and picks the refresh mode it needs;
// false when there was no memory to draw it
static bool draw_image(uint8_t *image_buffer, int data_size, int *pRefreshMode, bool *pAlloc) {
  bool isPNG = data_size >= 4 && MOTOLONG(image_buffer) == (int32_t)0x89504e47;
  auto width = display_width();
  auto height = display_height();
  bool bAlloc = false;
#ifdef BB_EPAPER
  int iRefreshMode = REFRESH_FULL; // assume full (slow) refresh
#else
  int iRefreshMode = 0;
#endif

#ifdef BB_EPAPER
  display_select_one_bit_panel();
#endif // BB_EPAPER
  if (isPNG == true && data_size < MAX_IMAGE_SIZE) {
    Log_info("Drawing PNG");
    iRefreshMode = png_to_epd(image_buffer, data_size, false);
  } else if (MOTOSHORT(image_buffer) == 0xffd8) {
    Log_info("Drawing JPEG");
    iRefreshMode = jpeg_to_epd(image_buffer, data_size);
  } else // uncompressed BMP or Group5 compressed image
  {
      if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER) {
            // G5 compressed image
      BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
      bbep.setMemoryMode(BB_MODE_1BPP); // use 1-bit mode to save memory
#else
      s_spectra6_buf_is_1bpp = false;
      bbep.setMemoryMode(BB_MODE_NATIVE);
#endif
      if (bbep.allocBuffer(false) != BBEP_SUCCESS) {
        Log_info("Error allocating bb_epaper frame buffer");
        return false;
      }
      bAlloc = true;
#endif
        //    int x = (width - pBBB->width)/2;
        //    int y = (height - pBBB->height)/2; // center it
        // place it in the lower right corner
      int x = (width - pBBB->width);
      int y = (height - pBBB->height);
      if (x > 0 || y > 0) // only clear if the image is smaller than the display
      {
        bbep.fillScreen(BBEP_WHITE);
      }
      bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
#ifdef BOARD_TRMNL_X
      display_draw_battery_icon(bbep.width() - DISPLAY_BATTERY_ICON_W - DISPLAY_BATTERY_ICON_MARGIN,
                                DISPLAY_BATTERY_ICON_MARGIN);
#endif // BOARD_TRMNL_X
    } else {
         // This work-around is due to a lack of RAM; the correct method would be to use loadBMP()
      const int iBmpWidth = image_buffer[18] | (image_buffer[19] << 8);
      const int iBmpHeight = image_buffer[22] | (image_buffer[23] << 8);
      bmpNormalizePolarity(image_buffer, image_buffer + 62,
                           (iBmpWidth / 8) * iBmpHeight); // palette may be [white, black]
      flip_image(image_buffer + 62, iBmpWidth, iBmpHeight, false); // fix bottom-up bitmap images
#ifdef BB_EPAPER
      bbep.setMemoryMode(BB_MODE_1BPP);
#ifdef BOARD_SEEED_RETERMINAL_E1002
      s_spectra6_buf_is_1bpp = true;
#endif
      bbep.setBuffer(image_buffer + 62); // uncompressed 1-bpp bitmap
#endif // BB_EPAPER
    }
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
    bbep.writePlane(); // send image data to the EPD (converting it if needed to 4/6-clr)
#endif
    iRefreshMode = REFRESH_PARTIAL;
#endif // BB_EPAPER
    iUpdateCount = 1; // use partial update
  }
  *pRefreshMode = iRefreshMode;
  *pAlloc = bAlloc;
  return true;
}

// Saves the temperature profile the server asked for, when it changed
static void save_temp_profile_from_api(void) {
  if (iTempProfile != apiDisplayResult.response.temp_profile) {
    iTempProfile = apiDisplayResult.response.temp_profile;
    Log_info("Saving new temperature profile (%" PRIu32 ") to FLASH", iTempProfile);
    preferences.putUInt(PREFERENCES_TEMP_PROFILE, iTempProfile);
  }
}

#ifdef BB_EPAPER
// Adjusts the image's refresh mode for the refresh count, the server's settings and the panel
static int choose_refresh_mode(int iRefreshMode, bool *pWait) {
  bool bWait = *pWait;
  if ((iUpdateCount & 7) == 0 || apiDisplayResult.response.maximum_compatibility == true) {
    Log_info("%s [%d]: Forcing full refresh; desired refresh mode was: %d\r\n", __FILE__, __LINE__, iRefreshMode);
    iRefreshMode = REFRESH_FULL; // force full refresh every 8 partials
  }
  int refresh_seconds = refreshInterval.seconds();
  if (refresh_seconds >= 30 * 60 && iRefreshMode == REFRESH_PARTIAL) {
        // For users who set updates 30 minutes or longer, use the "fast" update to prevent ghosting
    Log_info("%s [%d]: Forcing fast refresh (not partial) since the TRMNL refresh_rate is set to > 30 min\n", __FILE__,
             __LINE__);
    iRefreshMode = REFRESH_FAST;
  }
  if (bbep.capabilities() & (BBEP_4GRAY | BBEP_4COLOR | BBEP_3COLOR | BBEP_7COLOR)) bWait = 1;
  if (!bWait) {
    iRefreshMode = REFRESH_FAST; // fast update when showing loading screen
  }
  Log_info("%s [%d]: EPD refresh mode: %d\r\n", __FILE__, __LINE__, iRefreshMode);
  *pWait = bWait;
  return iRefreshMode;
}

// Refreshes the panel with the drawn image; false when the update failed
static bool refresh_epaper(int iRefreshMode, bool bWait, bool bAlloc) {
  static int i426Workaround = 0;
#ifdef DO_NOT_LIGHT_SLEEP
  display_set_light_sleep(false);
#else
  display_set_light_sleep(true);
#endif // DO_NOT_LIGHT_SLEEP
  if (bbep.getPanelType() == EP397_800x480 && iRefreshMode == REFRESH_FAST) {
        // Seeed Sticky: fast refresh on this panel isn't working and full refresh = fast
    iRefreshMode = REFRESH_FULL;
  }
  if (!display_update_epaper(iRefreshMode, bWait, bAlloc)) {
    Log_error("display_show_image: e-paper update failed");
    if (bAlloc) {
      bbep.freeBuffer();
    }
    return false;
  }

  if ((bbep.getPanelType() == EP426_800x480 || bbep.getPanelType() == EP426_800x480_4GRAY ||
       bbep.getPanelType() == EP397_800x480 ||
       bbep.getPanelType() == EP397_800x480_4GRAY) /* && iRefreshMode == REFRESH_PARTIAL*/) {
    i426Workaround = 1; // need to re-initialize the controller for another update before sleeping
  }
  if (bAlloc) {
    bbep.freeBuffer();
  }
  return true;
}
#else
// Pushes the drawn image to the panel with the right clearing passes
static void refresh_fastepd(bool bSkipClear) {
  {
    int rc = bbep.setCustomMatrix(u8_graytable, sizeof(u8_graytable));
    Log_info("%s [%d]: setCustomMatrix returned %d\r\n", __FILE__, __LINE__, rc);

 //   if (bbep.getPreviousMode() != BB_MODE_NONE && (bbep.getMode() == BB_MODE_1BPP || bbep.getMode() ==
 //   BB_MODE_2BPP)) {
    //       Log_info("%s [%d]: Using partial update since we have a copy of the previous image\n", __FILE__, __LINE__);
    //       bbep.setPasses(6,6);
    //       bbep.partialUpdate(false); // we have a previous image to diff against; use a non-flickering update
    //   } else {
    // bWait=false means loading screen: skip clearing passes so it appears
    // faster. Ghosting from the previous image is acceptable since the
    // real content refresh (bWait=true) immediately follows.
    int iClearMode = bSkipClear                                        ? CLEAR_NONE
                     : ((iUpdateCount & 7) == 0 || (iTempProfile > 0)) ? CLEAR_SLOW
                                                                       : CLEAR_FAST;
    Log_info("fullUpdate clear mode = %d\n", iClearMode);
    bbep.fullUpdate(iClearMode, false);
    //   }
  }
}
#endif // BB_EPAPER

/**
 * @brief Function to show the image on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param reverse shows if the color scheme is reverse
 * @return none
 */
void display_show_image(uint8_t *image_buffer, int data_size, bool bWait, bool bSkipClear) {
  int iRefreshMode;
  bool bAlloc;

  Log_info("display_show_image start");
  Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
  if (!draw_image(image_buffer, data_size, &iRefreshMode, &bAlloc)) {
    return;
  }
  Log_info("Display refresh start");
  save_temp_profile_from_api();
#ifdef BB_EPAPER
  iRefreshMode = choose_refresh_mode(iRefreshMode, &bWait);
  if (!refresh_epaper(iRefreshMode, bWait, bAlloc)) {
    return;
  }
#else
  refresh_fastepd(bSkipClear);
#endif
  iUpdateCount++;
  Log_info("display_show_image end");
}
/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void) {
  Log_info("Goto Sleep...");
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
  bbep.sleep(LIGHT_SLEEP);
#endif
#else
  bbep.einkPower(0);
  bbep.deInit();
#endif
}
