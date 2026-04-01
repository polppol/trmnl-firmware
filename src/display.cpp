#include <Arduino.h>
#include <SPI.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <display.h>
#include <PNGdec.h>
#include <JPEGDEC.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <preferences_persistence.h>
#include "DEV_Config.h"
#define MAX_BIT_DEPTH 8
#ifndef BOARD_TRMNL_X
#define BB_EPAPER
#include "bb_epaper.h"
const DISPLAY_PROFILE dpList[4] = { // 1-bit and 2-bit display types for each profile
#ifdef BOARD_XTEINK_X4
    {EP426_800x480, EP426_800x480_4GRAY}, // default (for original EPD)
    {EP426_800x480, EP426_800x480_4GRAY}, // a = uses built-in fast + 4-gray
    {EP426_800x480, EP426_800x480_4GRAY}, // b = darker grays
};
BBEPAPER bbep(EP426_800x480);
#elif defined(BOARD_WAVESHARE_397)
    {EP397_800x480, EP397_800x480_4GRAY}, // default (for original EPD)
    {EP397_800x480, EP397_800x480_4GRAY}, // a = uses built-in fast + 4-gray
    {EP397_800x480, EP397_800x480_4GRAY}, // b = darker grays
};
BBEPAPER bbep(EP397_800x480);
#elif defined(BOARD_XIAO_EPAPER_DISPLAY_3CLR)
    {EP75R_800x480, EP75R_800x480}, // default (for original EPD)
    {EP75R_800x480, EP75R_800x480}, // a = uses built-in fast + 4-gray
    {EP75R_800x480, EP75R_800x480}, // b = darker grays
};
BBEPAPER bbep(EP75R_800x480);
#elif defined(BOARD_TRMNL_4CLR)
    {EP75YR_800x480, EP75YR_800x480}, // default (for original EPD)
    {EP75YR_800x480, EP75YR_800x480}, // a = uses built-in fast + 4-gray
    {EP75YR_800x480, EP75YR_800x480}, // b = darker grays
};
BBEPAPER bbep(EP75YR_800x480);
#elif defined(BOARD_SEEED_RETERMINAL_E1002)
    // Spectra 6 — bb_epaper init patched in bb_ep.inl to match esp32-photoframe
    {EP73_SPECTRA_800x480, EP73_SPECTRA_800x480}, // default (Spectra 6)
    {EP73_SPECTRA_800x480, EP73_SPECTRA_800x480}, // a
    {EP73_SPECTRA_800x480, EP73_SPECTRA_800x480}, // b
};
BBEPAPER bbep(EP73_SPECTRA_800x480);
#else
    {EP75_800x480, EP75_800x480_4GRAY}, // default (for original EPD)
    {EP75_800x480_GEN2, EP75_800x480_4GRAY_GEN2}, // a = uses built-in fast + 4-gray
    {EP75_800x480, EP75_800x480_4GRAY_V2}, // b = darker grays
};
BBEPAPER bbep(EP75_800x480);
#endif
#ifdef BOARD_SEEED_RETERMINAL_E1002
uint8_t u8SpectraPal[512]; // RGB333 mapped to closest Spectra6 color
#endif // E1002
#else
#include "FastEPD.h"
FASTEPD bbep;
const uint8_t u8_graytable[] = {
/* 0 */  2, 2, 1, 1, 1, 1, 1, 1,
/* 1 */  2, 2, 2, 2, 1, 1, 2, 1,
/* 2 */  2, 2, 2, 1, 1, 1, 1, 2,
/* 3 */  2, 2, 2, 1, 1, 1, 1, 2,
/* 4 */  2, 2, 2, 2, 1, 1, 1, 2,
/* 5 */  2, 2, 2, 2, 1, 2, 2, 1,
/* 6 */  2, 2, 1, 1, 1, 2, 1, 2,
/* 7 */  2, 2, 2, 1, 1, 2, 1, 2,
/* 8 */  1, 1, 1, 1, 1, 1, 2, 2,
/* 9 */  2, 1, 1, 1, 1, 1, 2, 2,
/* 10 */  2, 2, 1, 1, 1, 1, 2, 2,
/* 11 */  2, 2, 2, 1, 1, 1, 2, 2,
/* 12 */  2, 1, 1, 2, 1, 1, 2, 2,
/* 13 */  2, 2, 2, 2, 1, 1, 2, 2,
/* 14 */  2, 2, 2, 2, 2, 1, 2, 2,
/* 15 */  2, 2, 2, 2, 2, 2, 2, 2
};
#endif
// Counts the number of partial updates to know when to do a full update
RTC_DATA_ATTR int iUpdateCount = 0;
#include "Group5.h"
#include <config.h>
#include "wifi_connect_qr.h"
#include "wifi_failed_qr.h"
#include <ctype.h> //iscntrl()
#include <api-client/display.h>
#include <trmnl_log.h>
#include "png_flip.h"
#include "nicoclean_8.h"
#include "Inter_18.h"
#include "Roboto_Black_24.h"
extern char filename[];
extern Preferences preferences;
extern ApiDisplayResult apiDisplayResult;
uint32_t iTempProfile;
static int i426Workaround = 0;
static uint8_t *pDither;

// Runtime control for light sleep (true = enabled, false = disabled)
static bool g_light_sleep_enabled = true;

#ifdef BOARD_SEEED_RETERMINAL_E1002
//
// E1002 Spectra 6 display update using ESP-IDF SPI driver.
// Arduino SPI (bb_epaper) does NOT work on this hardware — requires
// SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY flags.
// Reads bb_epaper's 4bpp framebuffer and sends via ESP-IDF SPI.
//
static spi_device_handle_t s_epd_spi = NULL;

static void spectra6_spi_init(void)
{
    if (s_epd_spi) return; // already initialized

    // GPIO setup
    gpio_config_t out_conf = {};
    out_conf.pin_bit_mask = (1ULL << EPD_RST_PIN) | (1ULL << EPD_DC_PIN) | (1ULL << EPD_CS_PIN);
    out_conf.mode = GPIO_MODE_OUTPUT;
    out_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&out_conf);
    gpio_config_t in_conf = {};
    in_conf.pin_bit_mask = (1ULL << EPD_BUSY_PIN);
    in_conf.mode = GPIO_MODE_INPUT;
    in_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&in_conf);
    gpio_set_level((gpio_num_t)EPD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)EPD_DC_PIN, 0);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = EPD_MOSI_PIN;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = EPD_SCK_PIN;
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

static void spectra6_wait_busy(const char *label)
{
    vTaskDelay(pdMS_TO_TICKS(10));
    int wc = 0;
    while (gpio_get_level((gpio_num_t)EPD_BUSY_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (++wc > 6000) {
            Log_info("spectra6 [%s] BUSY timeout", label);
            return;
        }
    }
}

void spectra6_update(void)
{
    Log_info("spectra6_update: start");
    spectra6_spi_init();
    if (!s_epd_spi) { Log_info("spectra6_update: SPI not available"); return; }
    spi_device_handle_t spi_dev = s_epd_spi;

    // SPI helpers (matching esp32-photoframe)
    auto spi_wr = [&spi_dev](const uint8_t *data, int len) {
        spi_transaction_t t = {};
        t.length = len * 8;
        t.tx_buffer = data;
        spi_device_polling_transmit(spi_dev, &t);
    };

    auto cmd_data = [&spi_dev, &spi_wr](uint8_t cmd, const uint8_t *data, size_t len) {
        gpio_set_level((gpio_num_t)EPD_DC_PIN, 0);
        spi_device_acquire_bus(spi_dev, portMAX_DELAY);
        gpio_set_level((gpio_num_t)EPD_CS_PIN, 0);
        spi_transaction_ext_t cmd_t = {};
        cmd_t.command_bits = 8;
        cmd_t.base.flags = SPI_TRANS_VARIABLE_CMD;
        cmd_t.base.cmd = cmd;
        spi_device_polling_transmit(spi_dev, &cmd_t.base);
        if (len > 0 && data) {
            gpio_set_level((gpio_num_t)EPD_DC_PIN, 1);
            uint8_t buf[16];
            memcpy(buf, data, len);
            spi_wr(buf, len);
        }
        gpio_set_level((gpio_num_t)EPD_CS_PIN, 1);
        spi_device_release_bus(spi_dev);
    };

    auto send_cmd = [&cmd_data](uint8_t cmd) { cmd_data(cmd, NULL, 0); };

    auto send_buffer = [&spi_dev, &spi_wr](const uint8_t *data, int len) {
        const uint8_t *ptr = data;
        int rem = len;
        while (rem > 0) {
            int chunk = (rem > 128) ? 128 : rem;
            gpio_set_level((gpio_num_t)EPD_DC_PIN, 1);
            spi_device_acquire_bus(spi_dev, portMAX_DELAY);
            gpio_set_level((gpio_num_t)EPD_CS_PIN, 0);
            spi_wr(ptr, chunk);
            gpio_set_level((gpio_num_t)EPD_CS_PIN, 1);
            spi_device_release_bus(spi_dev);
            ptr += chunk;
            rem -= chunk;
        }
    };

    // 1. Hardware reset
    gpio_set_level((gpio_num_t)EPD_RST_PIN, 1); vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)EPD_RST_PIN, 0); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level((gpio_num_t)EPD_RST_PIN, 1); vTaskDelay(pdMS_TO_TICKS(50));
    spectra6_wait_busy("reset");

    // 2. Init sequence (esp32-photoframe ED2208-GCA values)
    cmd_data(0xAA, (const uint8_t[]){0x49, 0x55, 0x20, 0x08, 0x09, 0x18}, 6);
    cmd_data(0x01, (const uint8_t[]){0x3F}, 1);
    cmd_data(0x00, (const uint8_t[]){0x5F, 0x69}, 2);
    cmd_data(0x03, (const uint8_t[]){0x00, 0x54, 0x00, 0x44}, 4);
    cmd_data(0x05, (const uint8_t[]){0x40, 0x1F, 0x1F, 0x2C}, 4);
    cmd_data(0x06, (const uint8_t[]){0x6F, 0x1F, 0x17, 0x49}, 4);
    cmd_data(0x08, (const uint8_t[]){0x6F, 0x1F, 0x1F, 0x22}, 4);
    cmd_data(0x30, (const uint8_t[]){0x03}, 1);
    cmd_data(0x50, (const uint8_t[]){0x3F}, 1);
    cmd_data(0x60, (const uint8_t[]){0x02, 0x00}, 2);
    cmd_data(0x61, (const uint8_t[]){0x03, 0x20, 0x01, 0xE0}, 4);
    cmd_data(0x84, (const uint8_t[]){0x01}, 1);
    cmd_data(0xE3, (const uint8_t[]){0x2F}, 1);
    spectra6_wait_busy("init");

    // 3. Send framebuffer data from bb_epaper (4bpp, 192000 bytes)
    send_cmd(0x10);
    uint8_t *fb = (uint8_t *)bbep.getBuffer();
    if (fb) {
        // bb_epaper 4bpp buffer: 800/2 * 480 = 192000 bytes
        send_buffer(fb, 192000);
    } else {
        // No buffer — send all white
        uint8_t row[400];
        memset(row, 0x11, 400);
        for (int r = 0; r < 480; r++) send_buffer(row, 400);
    }
    spectra6_wait_busy("data");

    // 4. Power on
    send_cmd(0x04);
    spectra6_wait_busy("PON");

    // 5. Display refresh (15-30 seconds)
    cmd_data(0x12, (const uint8_t[]){0x00}, 1);
    Log_info("spectra6_update: refreshing...");
    spectra6_wait_busy("refresh");

    // 6. Power off + deep sleep
    cmd_data(0x02, (const uint8_t[]){0x00}, 1);
    spectra6_wait_busy("POFF");
    cmd_data(0x07, (const uint8_t[]){0xA5}, 1);

    // SPI bus kept open for reuse across refreshes
    Log_info("spectra6_update: done");
}

// Draw 6-color indicator boxes on E1002 embedded screens
static void spectra6_draw_color_boxes(void)
{
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
    const uint8_t blendA[] = {BBEP_RED,    BBEP_RED,  BBEP_RED,    BBEP_BLUE,  BBEP_GREEN,  BBEP_BLUE};
    const uint8_t blendB[] = {BBEP_YELLOW, BBEP_BLUE, BBEP_WHITE,  BBEP_WHITE, BBEP_YELLOW, BBEP_GREEN};
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
    const uint8_t tri_a[] = {BBEP_RED,    BBEP_RED,   BBEP_YELLOW, BBEP_BLACK, BBEP_BLACK,  BBEP_WHITE};
    const uint8_t tri_b[] = {BBEP_YELLOW, BBEP_BLUE,  BBEP_BLUE,   BBEP_WHITE, BBEP_GREEN,  BBEP_GREEN};
    const uint8_t tri_c[] = {BBEP_BLUE,   BBEP_GREEN, BBEP_GREEN,  BBEP_RED,   BBEP_YELLOW, BBEP_RED};
    int row3Y = row2Y + box_h + gap;
    for (int c = 0; c < 6; c++) {
        int bx = startX + c * (box_w + gap);
        for (int py = 0; py < box_h; py++) {
            for (int px = 0; px < box_w; px++) {
                uint8_t col;
                int idx = (px + py) % 3;
                if (idx == 0) col = tri_a[c];
                else if (idx == 1) col = tri_b[c];
                else col = tri_c[c];
                bbep.drawPixel(bx + px, row3Y + py, col);
            }
        }
    }
}
#endif // BOARD_SEEED_RETERMINAL_E1002

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void)
{
    Log_info("dev module start");
    iTempProfile = preferences.getUInt(PREFERENCES_TEMP_PROFILE, TEMP_PROFILE_DEFAULT);
    Log_info("Saved temperature profile: %d", iTempProfile);
#ifdef BB_EPAPER
    bbep.setPanelType(dpList[iTempProfile].OneBit); // must be set BEFORE calling initio
#ifdef BOARD_SEEED_RETERMINAL_E1002
    // E1002: skip initIO — Arduino SPI doesn't work on this hardware.
    // We only need setPanelType for buffer dimensions/format.
    // spectra6_update() uses ESP-IDF SPI directly.
#else
    bbep.initIO(EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN, EPD_CS_PIN, EPD_MOSI_PIN, EPD_SCK_PIN, 8000000);
#endif
#else
    bbep.initPanel(BB_PANEL_EPDIY_V7_16); //, 26000000);
    bbep.setPanelSize(1872, 1404, BB_PANEL_FLAG_MIRROR_X);
#endif
    Log_info("dev module end");
}

/**
 * @brief Enable or disable light sleep at runtime
 * @param enabled true to enable light sleep, false to disable
 * @return none
 */
void display_set_light_sleep(uint8_t enabled)
{
    bbep.setLightSleep(enabled);
}

/**
 * @brief Function to sleep the ESP32 while saving power
 * @param u32Millis represents the sleep time in milliseconds
 * @return none
 */
void display_sleep(uint32_t u32Millis)
{
#ifdef DO_NOT_LIGHT_SLEEP
    delay(u32Millis);
#else
    if (!g_light_sleep_enabled) {
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
void display_reset(void)
{
    Log_info("e-Paper Clear start");
    bbep.fillScreen(BBEP_WHITE);
    bbep.setLightSleep(true);
#ifdef BB_EPAPER
#ifdef BOARD_SEEED_RETERMINAL_E1002
    spectra6_update();
#else
    if (!apiDisplayResult.response.maximum_compatibility) {
        bbep.refresh(REFRESH_FAST, true);
    } else {
        bbep.refresh(REFRESH_FULL, true); // incompatible panel
    }
#endif
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
uint16_t display_height()
{
    return bbep.height();
}

/**
 * @brief Function to read the display width
 * @return uint16_t - width of display in pixels
 */
uint16_t display_width()
{
    return bbep.width();
}

/**
 * @brief Function to draw multi-line text onto the display
 * @param x_start X coordinate to start drawing
 * @param y_start Y coordinate to start drawing
 * @param message Text message to draw
 * @param max_width Maximum width in pixels for each line
 * @param font_width Width of a single character in pixels
 * @param color_fg Foreground color
 * @param color_bg Background color
 * @param font Font to use
 * @param is_center_aligned If true, center the text; if false, left-align
 * @return none
 */
void Paint_DrawMultilineText(UWORD x_start, UWORD y_start, const char *message,
                             uint16_t max_width, uint16_t font_width,
                             UWORD color_fg, UWORD color_bg, const void *font,
                             bool is_center_aligned)
{
    BB_FONT_SMALL *pFont = (BB_FONT_SMALL *)font;
    uint16_t display_width_pixels = max_width;
    int max_chars_per_line = display_width_pixels / font_width;
    const int font_height = pFont->height;
    uint8_t MAX_LINES = 4;

    char lines[MAX_LINES][max_chars_per_line + 1] = {0};
    uint16_t line_count = 0;

    int text_len = strlen(message);
    int current_width = 0;
    int line_index = 0;
    int line_pos = 0;
    int word_start = 0;
    int i = 0;
    char word_buffer[max_chars_per_line + 1] = {0};
    int word_length = 0;

    bbep.setFont(font);
    bbep.setTextColor(color_fg, color_bg);

    bbep.setFont(font);
    bbep.setTextColor(color_fg, color_bg);

    while (i <= text_len && line_index < MAX_LINES)
    {
        word_length = 0;
        word_start = i;

        // Skip leading spaces
        while (i < text_len && message[i] == ' ')
        {
            i++;
        }
        word_start = i;

        // Find end of word or end of text
        while (i < text_len && message[i] != ' ')
        {
            i++;
        }

        word_length = i - word_start;
        if (word_length > max_chars_per_line)
        {
            word_length = max_chars_per_line; // Truncate if word is too long
        }

        if (word_length > 0)
        {
            strncpy(word_buffer, message + word_start, word_length);
            word_buffer[word_length] = '\0';
        }
        else
        {
            i++;
            continue;
        }

        int word_width = word_length * font_width;

        // Check if adding the word exceeds max_width
        if (current_width + word_width + (current_width > 0 ? font_width : 0) <= display_width_pixels)
        {
            // Add space before word if not the first word in the line
            if (current_width > 0 && line_pos < max_chars_per_line - 1)
            {
                lines[line_index][line_pos++] = ' ';
                current_width += font_width;
            }

            // Add word to current line
            if (line_pos + word_length <= max_chars_per_line)
            {
                strcpy(&lines[line_index][line_pos], word_buffer);
                line_pos += word_length;
                current_width += word_width;
            }
        }
        else
        {
            // Current line is full, draw it
            if (line_pos > 0)
            {
                lines[line_index][line_pos] = '\0'; // Null-terminate the current line
                line_index++;
                line_count++;

                if (line_index >= MAX_LINES)
                {
                    break;
                }

                // Start new line with this word
                strncpy(lines[line_index], word_buffer, word_length);
                line_pos = word_length;
                current_width = word_width;
            }
            else
            {
                // Single long word case
                strncpy(lines[line_index], word_buffer, max_chars_per_line);
                lines[line_index][max_chars_per_line] = '\0';
                line_index++;
                line_count++;
                line_pos = 0;
                current_width = 0;
            }
        }

        // Move to next word
        if (message[i] == ' ')
        {
            i++;
        }
    }

    // Store the last line if any
    if (line_pos > 0 && line_index < MAX_LINES)
    {
        lines[line_index][line_pos] = '\0';
        line_count++;
    }

    // Draw the lines
    for (int j = 0; j < line_count; j++)
    {
        uint16_t line_width = strlen(lines[j]) * font_width;
        uint16_t draw_x = x_start;

        if (is_center_aligned)
        {
            if (line_width < max_width)
            {
                draw_x = x_start + (max_width - line_width) / 2;
            }
        }
        bbep.setCursor(draw_x, y_start + j * (font_height + 5));
        bbep.print(lines[j]);
    }
}
/**
 * @brief Reduce the bit depth of line of pixels using thresholding (aka simple color mapping)
 * @param Destination bit count (1 or 2)
 * @param Pointer to a PNG palette (3 bytes per entry)
 * @param Pointer to the source pixels
 * @param Pointer to the destination pixels
 * @param Pixel count
 * @param Original bit depth
 * @return none
 */
void ReduceBpp(int iDestBpp, int iPixelType, uint8_t *pPalette, uint8_t *pSrc, uint8_t *pDest, int w, int iSrcBpp)
{
    int g = 0, x, iDelta;
    uint8_t *s, *d, *pPal, u8, count;
    const uint8_t u8G2ToG8[4] = {0x00, 0x55, 0xaa, 0xff}; // 2-bit to 8-bit gray

    if (iPixelType == PNG_PIXEL_TRUECOLOR) iSrcBpp = 24;
    else if (iPixelType == PNG_PIXEL_TRUECOLOR_ALPHA) iSrcBpp = 32;
    iDelta = iSrcBpp/8; // bytes per pixel
    count = 8; // bits in a byte
    u8 = 0; // start with all black
    d = pDest;
    s = pSrc;
    for (x=0; x<w; x++) {
        u8 <<= iDestBpp;
        switch (iSrcBpp) {
            case 24:
            case 32:
                g = (s[0] + s[1]*2 + s[2])/4; // convert color to gray value
                s += iDelta;
                break;
            case 8:
                if (iPixelType == PNG_PIXEL_INDEXED) {
                    pPal = &pPalette[s[0] * 3];
                    g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                } else { // must be grayscale
                    g = s[0];
                }
                s++;
                break;
            case 4:
                if (x & 1) {
                    if (iPixelType == PNG_PIXEL_INDEXED) {
                        pPal = &pPalette[(s[0] & 0xf) * 3];
                        g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                    } else {
                        g = (s[0] & 0xf) | (s[0] << 4);
                    }
                    s++;
                } else {
                    if (iPixelType == PNG_PIXEL_INDEXED) {
                        pPal = &pPalette[(s[0]>>4) * 3];
                        g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                    } else {
                        g = (s[0] & 0xf0) | (s[0] >> 4);
                    }
                }
                break;
            case 2: // We need to handle this case for 2-bit images with (random) palettes
                g = s[0] >> (6-((x & 3) * 2));
                if (iPixelType == PNG_PIXEL_INDEXED) {
                    pPal = &pPalette[(g & 3)*3];
                    g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                } else {
                    g = u8G2ToG8[g & 3];
                }
                if ((x & 3) == 3) {
                    s++;
                }
                break;
        } // switch on bpp
        if (iDestBpp == 1) {
            u8 |= (g >> 7); // B/W
        } else { // generate 4 gray levels (2 bits)
            u8 |= (3 ^ (g >> 6)); // 4 gray levels (inverted relative to 1-bit)
        }
        count -= iDestBpp;
        if (count == 0) { // byte is full, move on
            *d++ = u8;
            u8 = 0;
            count = 8;
        }
    } // for x
    if (count != 8) { // partial byte remaining
        u8 <<= count;
        *d++ = u8;
    }
} /* ReduceBpp() */
enum {
    PNG_1_BIT = 0,
    PNG_1_BIT_INVERTED,
    PNG_2_BIT_0,
    PNG_2_BIT_1,
    PNG_2_BIT_BOTH,
    PNG_2_BIT_INVERTED,
};
//
// Match the given pixel to black (00), white (01), or red (1x)
//
unsigned char GetBWRPixel(int r, int g, int b)
{
    uint8_t ucOut=BBEP_BLACK;
    int gr;

    gr = (b + r + g*2)>>2; // gray
    // match the color to closest of black/white/red
    if (r > g && r > b) { // red is dominant
        if (gr < 100 && r < 80) {
            // black
        } else {
            if (r-b > 32 && r-g > 32) {
                // is red really dominant?
                ucOut = BBEP_RED; // red (can be 2 or 3, but 3 is compatible w/BWYR)
            } else { // yellowish should be white
                // no, use white instead of pink/yellow
                ucOut = BBEP_WHITE;
            }
        }
    } else { // check for white/black
        if (gr >= 128) {
            ucOut = BBEP_WHITE; // white
        } else {
            // black
        }
    }
    return ucOut;
} /* GetBWRPixel() */
//
// Match the given pixel to black (00), white (01), yellow (10), or red (11)
// returns 2 bit value of closest matching color
//
unsigned char GetBWYRPixel(int r, int g, int b)
{
    uint8_t ucOut=BBEP_BLACK;
    int gr;

    gr = (b + r + g*2)>>2; // gray
    // match the color to closest of black/white/yellow/red
    if (r > b || g > b) { // red or yellow is dominant
        if (gr < 90 && r < 80 && g < 80) {
            // black
        } else {
            if (r-b > 32 && r-g > r/2) {
                // is red really dominant?
                ucOut = BBEP_RED; // red
            } else if (r-b > 32 && g-b > 32) {
                // yes, yellow
                ucOut = BBEP_YELLOW;
            } else {
                ucOut = BBEP_WHITE; // gray/white
            }
        }
    } else { // check for white/black
        if (gr >= 100) {
            ucOut = BBEP_WHITE; // white
        } else {
            // black
        }
    }
    return ucOut;
} /* GetBWYRPixel() */
#ifdef BOARD_SEEED_RETERMINAL_E1002
//
// Spectra 6 measured palette (from esp32-photoframe actual display measurements)
//
static const int16_t spectra6_palette[6][3] = {
    {  2,   2,   2},   // 0 = Black
    {179, 182, 171},   // 1 = White
    {205, 202,   0},   // 2 = Yellow
    {117,  10,   0},   // 3 = Red
    {  0,  47, 107},   // 4 = Blue
    { 33,  69,  40},   // 5 = Green
};

// Find closest Spectra 6 color (Euclidean distance in RGB)
static uint8_t spectra6_find_closest(int16_t r, int16_t g, int16_t b)
{
    int min_dist = 0x7FFFFFFF;
    uint8_t best = 0;
    for (int i = 0; i < 6; i++) {
        int dr = r - spectra6_palette[i][0];
        int dg = g - spectra6_palette[i][1];
        int db = b - spectra6_palette[i][2];
        int dist = dr*dr + dg*dg + db*db;
        if (dist < min_dist) { min_dist = dist; best = i; }
    }
    return best;
}

// Clamp value to 0-255
static inline int16_t clamp8(int16_t v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

// Floyd-Steinberg error diffusion buffer for PNG line-by-line decode
static int16_t *fs_err_cur = NULL;  // current row error [width * 3] (R,G,B)
static int16_t *fs_err_next = NULL; // next row error [width * 3]
static int fs_width = 0;

static void spectra6_fs_init(int width)
{
    fs_width = width;
    fs_err_cur = (int16_t *)calloc(width * 3, sizeof(int16_t));
    fs_err_next = (int16_t *)calloc(width * 3, sizeof(int16_t));
}

static void spectra6_fs_free(void)
{
    if (fs_err_cur) { free(fs_err_cur); fs_err_cur = NULL; }
    if (fs_err_next) { free(fs_err_next); fs_err_next = NULL; }
    fs_width = 0;
}

// Process one row with Floyd-Steinberg dithering, write to framebuffer
static void spectra6_fs_dither_row(int y, const uint8_t *rgb_row, int width)
{
    if (!fs_err_cur || !fs_err_next) return;

    for (int x = 0; x < width; x++) {
        // Original pixel + accumulated error
        int16_t r = clamp8(rgb_row[x*3 + 0] + fs_err_cur[x*3 + 0]);
        int16_t g = clamp8(rgb_row[x*3 + 1] + fs_err_cur[x*3 + 1]);
        int16_t b = clamp8(rgb_row[x*3 + 2] + fs_err_cur[x*3 + 2]);

        // Find nearest palette color
        uint8_t ci = spectra6_find_closest(r, g, b);
        bbep.drawPixel(x, y, ci);

        // Quantization error
        int16_t er = r - spectra6_palette[ci][0];
        int16_t eg = g - spectra6_palette[ci][1];
        int16_t eb = b - spectra6_palette[ci][2];

        // Floyd-Steinberg diffusion: right (7/16), below-left (3/16), below (5/16), below-right (1/16)
        if (x + 1 < width) {
            fs_err_cur[(x+1)*3 + 0] += er * 7 / 16;
            fs_err_cur[(x+1)*3 + 1] += eg * 7 / 16;
            fs_err_cur[(x+1)*3 + 2] += eb * 7 / 16;
        }
        if (x > 0) {
            fs_err_next[(x-1)*3 + 0] += er * 3 / 16;
            fs_err_next[(x-1)*3 + 1] += eg * 3 / 16;
            fs_err_next[(x-1)*3 + 2] += eb * 3 / 16;
        }
        fs_err_next[x*3 + 0] += er * 5 / 16;
        fs_err_next[x*3 + 1] += eg * 5 / 16;
        fs_err_next[x*3 + 2] += eb * 5 / 16;
        if (x + 1 < width) {
            fs_err_next[(x+1)*3 + 0] += er * 1 / 16;
            fs_err_next[(x+1)*3 + 1] += eg * 1 / 16;
            fs_err_next[(x+1)*3 + 2] += eb * 1 / 16;
        }
    }

    // Swap buffers: next becomes current, clear next
    int16_t *tmp = fs_err_cur;
    fs_err_cur = fs_err_next;
    fs_err_next = tmp;
    memset(fs_err_next, 0, width * 3 * sizeof(int16_t));
}

// Legacy simple nearest-neighbor (kept for reference / non-dithered use)
uint8_t GetSpectraPixel(int r, int g, int b)
{
    return spectra6_find_closest(r, g, b);
}
void CreateSpectra6Pal(void) { /* no-op — using direct palette lookup now */ }
#endif // E1002
/**
 * @brief Callback function for each line of PNG decoded
 * @param PNGDRAW structure containing the current line and relevant info
 * @return none
 */
#ifdef BB_EPAPER
#ifdef BOARD_SEEED_RETERMINAL_E1002
//
// PNG line callback with Floyd-Steinberg dithering for Spectra 6.
// Extracts RGB from any PNG format, then dithers each row.
//
static uint8_t *fs_rgb_row = NULL; // temp RGB row buffer

int png_draw_6clr(PNGDRAW *pDraw)
{
    uint8_t r=0, g=0, b=0, *s, *pPal, *pPalette = pDraw->pPalette;
    int x, iDelta, iBpp = pDraw->iBpp;
    int y = pDraw->y;
    int w = pDraw->iWidth;

    switch (pDraw->iPixelType) {
        case PNG_PIXEL_INDEXED: break;
        case PNG_PIXEL_TRUECOLOR:
            if (iBpp <= 8) iBpp *= 3;
            pPalette = NULL; break;
        case PNG_PIXEL_TRUECOLOR_ALPHA:
            if (iBpp <= 8) iBpp *= 4;
            pPalette = NULL; break;
        case PNG_PIXEL_GRAYSCALE:
            pPalette = NULL; break;
    }
    iDelta = iBpp/8;
    s = pDraw->pPixels;

    // Extract RGB into row buffer
    if (!fs_rgb_row) fs_rgb_row = (uint8_t *)malloc(w * 3);
    if (!fs_rgb_row) return 0;

    for (x = 0; x < w; x++) {
        switch (iBpp) {
            case 24: case 32:
                r = s[0]; g = s[1]; b = s[2]; s += iDelta; break;
            case 16:
                r = s[1] & 0xf8; g = ((s[0] | s[1] << 8) >> 3) & 0xfc; b = s[0] << 3;
                s += 2; break;
            case 8:
                if (pPalette) { pPal = &pPalette[s[0]*3]; r=pPal[0]; g=pPal[1]; b=pPal[2]; }
                else { r = g = b = s[0]; }
                s++; break;
            case 4:
                if (pPalette) {
                    if (x&1) { pPal=&pPalette[(s[0]&0xf)*3]; s++; }
                    else { pPal=&pPalette[(s[0]>>4)*3]; }
                    r=pPal[0]; g=pPal[1]; b=pPal[2];
                } else {
                    if (x&1) { r=g=b=(s[0]&0xf)|(s[0]<<4); s++; }
                    else { r=g=b=(s[0]>>4)|(s[0]&0xf0); }
                }
                break;
            case 2:
                if (pPalette) { pPal=&pPalette[((s[0]>>((3-(x&3))*2))&3)*3]; r=pPal[0]; g=pPal[1]; b=pPal[2]; }
                else { r=g=b=(s[0]<<((x&3)*2))&0xc0; }
                if ((x&3)==3) s++;
                break;
            case 1:
                if (pPalette) { pPal=&pPalette[((s[0]>>(7-(x&7)))&1)*3]; r=pPal[0]; g=pPal[1]; b=pPal[2]; }
                else { r=g=b=((s[0]<<(x&7))&0x80); }
                if ((x&7)==7) s++;
                break;
        }
        fs_rgb_row[x*3+0] = r;
        fs_rgb_row[x*3+1] = g;
        fs_rgb_row[x*3+2] = b;
    }

    // Apply Floyd-Steinberg dithering to this row
    spectra6_fs_dither_row(y, fs_rgb_row, w);
    return 1;
} /* png_draw_6clr() */
#endif // E1002 (Spectra6 only)

#ifdef BOARD_TRMNL_4CLR
//
// Draw the PNG image into the local framebuffer memory using the drawPixel() method
// to do color translation and to properly format the memory layout
//
int png_draw_4clr(PNGDRAW *pDraw)
{
    uint8_t r=0, g=0, b=0, *s, *pPal, *pPalette = pDraw->pPalette;
    int x, iDelta, iBpp = pDraw->iBpp;
    uint8_t uc=0, *d, *pTemp = bbep.getCache(); // get some scratch memory (not from the stack)

    d = pTemp;
    switch (pDraw->iPixelType) {
        case PNG_PIXEL_INDEXED:
            break;
        case PNG_PIXEL_TRUECOLOR:
	        if (iBpp <= 8) {
                iBpp *= 3;
	        }
            pPalette = NULL;
            break;
        case PNG_PIXEL_TRUECOLOR_ALPHA:
	        if (iBpp <= 8) {
                iBpp *= 4;
	        }
            pPalette = NULL;
            break;
        case PNG_PIXEL_GRAYSCALE:
            pPalette = NULL;
            break;
    } // switch on pixel type
    iDelta = iBpp/8;
    s = pDraw->pPixels;
    for (x=0; x<pDraw->iWidth; x++) { // slower code, but less code :)
        switch (iBpp) {
            case 24:
            case 32:
                r = s[0];
                g = s[1];
                b = s[2];
                s += iDelta;
                break;
            case 16:
                r = s[1] & 0xf8; // red
                g = ((s[0] | s[1] << 8) >> 3) & 0xfc; // green
                b = s[0] << 3;
                s += 2;
                break;
                case 8:
                    if (pPalette) {
                        pPal = &pPalette[s[0] * 3];
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        r = g = b = s[0];
                    }
                    s++;
                    break;
                case 4:
                    if (pPalette) {
                        if (x & 1) {
                            pPal = &pPalette[(s[0] & 0xf) * 3];
                            s++;
                        } else {
                            pPal = &pPalette[(s[0]>>4) * 3];
                        }
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        if (x & 1) {
                            r = g = b = (s[0] & 0xf) | (s[0] << 4);
                            s++;
                        } else {
                            r = g = b = (s[0] >> 4) | (s[0] & 0xf0);
                        }
                    }
                    break;
                case 2:
                    if (pPalette) {
                    pPal = &pPalette[((s[0] >> ((3-(x&3))*2)) & 3) * 3];
                    r = pPal[0]; g = pPal[1]; b = pPal[2];
                    } else {
                    r = g = b = (s[0] << ((x&3)*2)) & 0xc0;
                    }
                    if ((x & 3) == 3) s++;
                    break;
                case 1:
                    if (pPalette) {
                        pPal = &pPalette[((s[0] >> (7-(x&7))) & 1) * 3];
                        r = pPal[0]; g = pPal[1]; b = pPal[2];
                    } else {
                        r = g = b = ((s[0] << (x&7)) & 0x80);
                    }
                    if ((x & 7) == 7) s++;
                    break;
            } // switch on bpp
            uc <<= 2;
            uc |= GetBWYRPixel(r, g, b); // get the best matching 2-bit color
            if ((x & 3) == 3) { // 4 pixels packed into each byte
                *d++ = uc;
            }
        } // for x
    bbep.writeData(pTemp, (pDraw->iWidth+3)/4);
    return 1; // continue decoding
} /* png_draw4clr() */
#endif // BOARD_TRMNL_4CLR (4 color only)

int png_draw(PNGDRAW *pDraw)
{
    int x;
    uint8_t ucBppChanged = 0, ucInvert = 0;
    uint8_t uc, ucMask, src, *s, *d, *pTemp = bbep.getCache(); // get some scratch memory (not from the stack)
    int iPlane = *(int *)pDraw->pUser;
    int iWidth;

    iWidth = pDraw->iWidth;
    if (pDraw->y >= bbep.height()) return 0; // stop decoding if we'll go past the bottom
    if (iWidth > bbep.width()) iWidth = bbep.width(); // crop image width to display size if it's larger

    if (pDraw->iPixelType == PNG_PIXEL_INDEXED || pDraw->iBpp > 2) {
        if (pDraw->iBpp == 1) { // 1-bit output, just see which color is brighter
            uint32_t u32Gray0, u32Gray1;
            u32Gray0 = pDraw->pPalette[0] + (pDraw->pPalette[1]<<2) + pDraw->pPalette[2];
            u32Gray1 = pDraw->pPalette[3] + (pDraw->pPalette[4]<<2) + pDraw->pPalette[5];
          if (u32Gray0 < u32Gray1) {
            ucInvert = 0xff;
          }
        } else {
            // Reduce the source image to 1-bpp or 2-bpp
            ReduceBpp((pDraw->pUser) ? 2:1, pDraw->iPixelType, pDraw->pPalette, pDraw->pPixels, pTemp, iWidth, pDraw->iBpp);
            ucBppChanged = 1;
        }
    } else if (pDraw->iBpp == 2) {
        ucInvert = 0xff; // 2-bit non-palette images need to be inverted colors for 4-gray mode
    }
    s = (ucBppChanged) ? pTemp : (uint8_t *)pDraw->pPixels;
    d = pTemp;
    if (iPlane == PNG_1_BIT || iPlane == PNG_1_BIT_INVERTED) {
        // 1-bit output, decode the single plane and write it
        if (iPlane == PNG_1_BIT_INVERTED) ucInvert = ~ucInvert; // to do PLANE_FALSE_DIFF
        if (iPlane == PNG_1_BIT_INVERTED && (bbep.capabilities() & BBEP_3COLOR)) { // write the red plane as 0's for this case
            memset(d, 0, iWidth/8);
        } else {
            for (x=0; x<iWidth; x+= 8) {
                d[0] = s[0] ^ ucInvert;
                d++; s++;
            }
        }
    } else { // we need to split the 2-bit data into plane 0 and 1
        src = *s++;
        src ^= ucInvert;
        uc = 0; // suppress warning/error
        if (iPlane == PNG_2_BIT_BOTH || iPlane == PNG_2_BIT_INVERTED) { // draw 2bpp data as 1-bit to use for partial update
            if (iPlane == PNG_2_BIT_BOTH) {
                ucInvert = ~ucInvert; // the invert rule is backwards for grayscale data
            }
            src = ~src;
            for (x=0; x<iWidth; x++) {
                uc <<= 1;
                if (src & 0xc0) { // non-white -> black
                    uc |= 1; // high bit of source pair
                }
                src <<= 2;
                if ((x & 3) == 3) { // new input byte
                    src = *s++;
                    src ^= ucInvert;
                }
                if ((x & 7) == 7) { // new output byte
                    *d++ = uc;
                }
            } // for x
        } else { // normal 0/1 split plane
            ucMask = (iPlane == PNG_2_BIT_0) ? 0x40 : 0x80; // lower or upper source bit
            for (x=0; x<iWidth; x++) {
                uc <<= 1;
                if (src & ucMask) {
                    uc |= 1; // high bit of source pair
                }
                src <<= 2;
                if ((x & 3) == 3) { // new input byte
                    src = *s++;
                    src ^= ucInvert;
                }
                if ((x & 7) == 7) { // new output byte
                    *d++ = uc;
                }
            } // for x
        }
    }
    bbep.writeData(pTemp, (iWidth+7)/8);
    return 1;
} /* png_draw() */
#else // TRMNL_X version
int png_draw(PNGDRAW *pDraw)
{
    int x;
    uint8_t uc = 0;
    uint8_t ucMask, src, *s, *d;
    int iPitch;

    s = (uint8_t *)pDraw->pPixels;
    d = bbep.currentBuffer();
    iPitch = bbep.width()/2;
    if (pDraw->iBpp == 1) {
        if (bbep.width() == pDraw->iWidth) { // normal orientation
            iPitch = (bbep.width() + 7)/8;
            d += pDraw->y * iPitch; // point to the correct line
            memcpy(d, s, (pDraw->iWidth+7)/8);
        } else { // rotated
            uint8_t ucPixel, ucMask, j;
            d += (bbep.height() - 1) * iPitch;
            d += (pDraw->y / 8);
            ucMask = 0x80 >> (pDraw->y & 7); // destination mask
            for (x=0; x<pDraw->iWidth; x++) {
                if ((x & 7) == 0) uc = *s++;
                ucPixel = d[0] & ~ucMask; // unset old pixel
                if (uc & 0x80) ucPixel |= ucMask;
                d[0] = ucPixel;
                uc <<= 1;
                d -= iPitch;
            }
        }
    } else if (pDraw->iBpp == 2) { // we need to convert the 2-bit data into 4-bits
        iPitch = bbep.width()/2;
        if (bbep.width() == pDraw->iWidth) { // normal orientation
            for (x=0; x<pDraw->iWidth; x+=4) {
                src = *s++;
                uc = (src & 0xc0); // first pixel
                uc |= ((src & 0x30) >> 2);
                *d++ = uc;
                uc = (src & 0xc) << 4;
                uc |= ((src & 0x3) << 2);
                *d++ = uc;
            } // for x
        } else { // rotated
            d += (bbep.height() - 1) * iPitch;
            d += (pDraw->y / 2);
            if (pDraw->y & 1) { // odd line (column)
                for (x=0; x<pDraw->iWidth; x+=4) {
                    uc = (d[0] & 0xf0) | ((s[0] >> 4) & 0x0c);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf0) | ((s[0] >> 2) & 0x0c);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf0) | (s[0] & 0xc);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf0) | ((s[0] << 2) & 0x0c);
                    *d = uc;
                    d -= iPitch;
                    s++;
                } // for x
            } else {
                for (x=0; x<pDraw->iWidth; x+=4) {
                    uc = (d[0] & 0xf) | (s[0] & 0xc0);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf) | ((s[0] << 2) & 0xc0);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf) | ((s[0] << 4) & 0xc0);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf) | ((s[0] << 6) & 0xc0);
                    *d = uc;
                    d -= iPitch;
                    s++;
                } // for x
            }
        }
    } else if (pDraw->iBpp == 4) { // 4-bit is the native format
        if (bbep.width() == pDraw->iWidth) { // normal orientation
            d += pDraw->y * iPitch; // point to the correct line
            memcpy(d, s, (pDraw->iWidth+1)/2);
        } else { // rotated
            d += (bbep.height() - 1) * iPitch;
            d += (pDraw->y / 2);
            if (pDraw->y & 1) { // odd line (column)
                for (x=0; x<pDraw->iWidth; x+=2) {
                    uc = (d[0] & 0xf0) | (s[0] >> 4);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf0) | (s[0] & 0xf);
                    *d = uc;
                    d -= iPitch;
                    s++;
                } // for x
            } else {
                for (x=0; x<pDraw->iWidth; x+=2) {
                    uc = (d[0] & 0xf) | (s[0] & 0xf0);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf) | (s[0] << 4);
                    *d = uc;
                    d -= iPitch;
                    s++;
                } // for x
            }
        }
    } else { // must be 8-bit grayscale
        if (bbep.width() == pDraw->iWidth) { // normal orientation
            d += pDraw->y * iPitch; // point to the correct line
            for (x=0; x<pDraw->iWidth; x+=2) {
                uc = (s[0] & 0xf0) | (s[1] >> 4);
                *d++ = uc;
                s += 2;
            } // for x
        } else { // rotated
            d += (bbep.height() - 1) * iPitch;
            d += (pDraw->y / 2);
            if (pDraw->y & 1) { // odd line (column)
                for (x=0; x<pDraw->iWidth; x++) {
                    uc = (d[0] & 0xf0) | (s[0] >> 4);
                    *d = uc;
                    s++;
                    d -= iPitch;
                } // for x
            } else {
                for (x=0; x<pDraw->iWidth; x++) {
                    uc = (d[0] & 0xf) | (s[0] & 0xf0);
                    *d = uc;
                    s++;
                    d -= iPitch;
                } // for x
            }
        }
    }
    return 1;
} /* png_draw() */
#endif
//
// A table to accelerate the testing of 2-bit images for the number
// of unique colors. Each entry sets bits 0-3 depending on the presence
// of colors 0-3 in each 2-bit pixel
//
const uint8_t ucTwoBitFlags[256] = {
0x01,0x03,0x05,0x09,0x03,0x03,0x07,0x0b,0x05,0x07,0x05,0x0d,0x09,0x0b,0x0d,0x09,
0x03,0x03,0x07,0x0b,0x03,0x03,0x07,0x0b,0x07,0x07,0x07,0x0f,0x0b,0x0b,0x0f,0x0b,
0x05,0x07,0x05,0x0d,0x07,0x07,0x07,0x0f,0x05,0x07,0x05,0x0d,0x0d,0x0f,0x0d,0x0d,
0x09,0x0b,0x0d,0x09,0x0b,0x0b,0x0f,0x0b,0x0d,0x0f,0x0d,0x0d,0x09,0x0b,0x0d,0x09,
0x03,0x03,0x07,0x0b,0x03,0x03,0x07,0x0b,0x07,0x07,0x07,0x0f,0x0b,0x0b,0x0f,0x0b,
0x03,0x03,0x07,0x0b,0x03,0x02,0x06,0x0a,0x07,0x06,0x06,0x0e,0x0b,0x0a,0x0e,0x0a,
0x07,0x07,0x07,0x0f,0x07,0x06,0x06,0x0e,0x07,0x06,0x06,0x0e,0x0f,0x0e,0x0e,0x0e,
0x0b,0x0b,0x0f,0x0b,0x0b,0x0a,0x0e,0x0a,0x0f,0x0e,0x0e,0x0e,0x0b,0x0a,0x0e,0x0a,
0x05,0x07,0x05,0x0d,0x07,0x07,0x07,0x0f,0x05,0x07,0x05,0x0d,0x0d,0x0f,0x0d,0x0d,
0x07,0x07,0x07,0x0f,0x07,0x06,0x06,0x0e,0x07,0x06,0x06,0x0e,0x0f,0x0e,0x0e,0x0e,
0x05,0x07,0x05,0x0d,0x07,0x06,0x06,0x0e,0x05,0x06,0x04,0x0c,0x0d,0x0e,0x0c,0x0c,
0x0d,0x0f,0x0d,0x0d,0x0f,0x0e,0x0e,0x0e,0x0d,0x0e,0x0c,0x0c,0x0d,0x0e,0x0c,0x0c,
0x09,0x0b,0x0d,0x09,0x0b,0x0b,0x0f,0x0b,0x0d,0x0f,0x0d,0x0d,0x09,0x0b,0x0d,0x09,
0x0b,0x0b,0x0f,0x0b,0x0b,0x0a,0x0e,0x0a,0x0f,0x0e,0x0e,0x0e,0x0b,0x0a,0x0e,0x0a,
0x0d,0x0f,0x0d,0x0d,0x0f,0x0e,0x0e,0x0e,0x0d,0x0e,0x0c,0x0c,0x0d,0x0e,0x0c,0x0c,
0x09,0x0b,0x0d,0x09,0x0b,0x0a,0x0e,0x0a,0x0d,0x0e,0x0c,0x0c,0x09,0x0a,0x0c,0x08
};

int png_draw_count(PNGDRAW *pDraw)
{
    int x, *pFlags = (int *)pDraw->pUser;
    uint8_t *s, set_bits;

    if (pDraw->y > 430) return 0; // Workaround to ignore the icon in the lower left corner

    set_bits = pFlags[0]; // use a local var
    s = (uint8_t *)pDraw->pPixels;
    for (x=0; x<pDraw->iWidth; x+=4) {
        set_bits |= ucTwoBitFlags[*s++]; // do 4 pixels at a time
    } // for x
    pFlags[0] = set_bits; // put it back in the flags array
    return 1;
} /* png_draw_count() */
/**
 * @brief Function to decode a PNG and count the number of unique colors
 *        This is needed because 2-bit (4gray) images can sometimes contain
 *        only 2 unique colors. This will allow us to use partial (non-flickering)
 *        updates on these images.
 * @param pointer to the PNG class instance
 * @param pointer to the buffer holding the PNG file
 * @param size of the PNG file
 * @return the number of unique colors in the image (2 to 4)
 */
int png_count_colors(PNG *png, const uint8_t *pData, int iDataSize)
{
int i, iColors;
    png->openRAM((uint8_t *)pData, iDataSize, png_draw_count);
    i = 0;
    png->decode(&i, 0);
    png->close();
    iColors = 0;
    if (i & 1) iColors++;
    if (i & 2) iColors++;
    if (i & 4) iColors++;
    if (i & 8) iColors++;
    Log_info("%s [%d]: png_count_colors: %d\r\n", __FILE__, __LINE__, iColors);
    return iColors;
} /* png_count_colors() */

/**
 * @brief JPEGDEC callback function passed blocks of MCUs (minimum coded units)
 * @param pointer to the JPEGDRAW structure
 * @return 1 to continue decoding or 0 to abort
 */
int jpeg_draw(JPEGDRAW *pDraw)
{
#ifdef BB_EPAPER
int x, y;
int iPlane = *(int *)pDraw->pUser;
uint8_t src=0, uc=0, ucMask, *s, *d, *pTemp = bbep.getCache();

    bbep.setAddrWindow(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
    if (iPlane == 0) { // 1-bit mode
        bbep.startWrite(PLANE_0); // start writing image data to plane 0
        for (y=0; y<pDraw->iHeight; y++) { // this is 8 or 16 depending on the color subsampling
            s = (uint8_t *)pDraw->pPixels;
            s += (y * (pDraw->iWidth >> 3));
            // The pixel format of the display is the same as JPEGDEC, so just copy it
            bbep.writeData(s, (pDraw->iWidth+7)/8);
        } // for y
    } else {
        bbep.startWrite((iPlane == 1) ? PLANE_0 : PLANE_1); // start writing image data to plane 0
        for (y=0; y<pDraw->iHeight; y++) { // this is 8 or 16 depending on the color subsampling
            d = pTemp;
            s = (uint8_t *)pDither;
            s += (y * (pDraw->iWidth >> 2));
            ucMask = (iPlane == 1) ? 0x40 : 0x80; // lower or upper source bit
            for (x=0; x<pDraw->iWidth; x++) {
                if ((x & 3) == 0) { // new input byte
                    src = *s++;
                }
                uc <<= 1;
                if (src & ucMask) {
                    uc |= 1; // high bit of source pair
                }
                src <<= 2;
                if ((x & 7) == 7) { // new output byte
                    *d++ = uc;
                }
            } // for x
            bbep.writeData(pTemp, (pDraw->iWidth+7)/8);
        } // for y
    }
#else // FastEPD
  int x, y, iPitch = bbep.width()/2; // assume 4-bpp drawing mode
  uint8_t *s, *d, *pBuffer = bbep.currentBuffer();
  for (y=0; y<pDraw->iHeight; y++) {
    d = &pBuffer[((pDraw->y + y)*iPitch) + (pDraw->x/2)];
    s = (uint8_t *)pDraw->pPixels;
    s += (y * (pDraw->iWidth/2));
    memcpy(d, s, pDraw->iWidth/2); // source & dest format are the same
  } // for y
#endif
    return 1; // continue decoding
} /* jpeg_draw() */
/**
 * @brief Function to decode and display a JPEG image from memory
 *        The decoded lines are written directly into the EPD framebuffer
 *        due to insufficient RAM to hold the fully decoded image
 * @param pointer to the buffer holding the JPEG file
 * @param size of the JPEG file
 * @return refresh mode based on image type and presence of old image
 */
int jpeg_to_epd(const uint8_t *pJPEG, int iDataSize)
{
JPEGDEC *jpg = new JPEGDEC();
int rc = -1; // invalid mode
int iPlane = 0;

    if (!jpg) {
        Log_error("%s [%d]: Not enough memory for the JPEG decoder instance", __FILE__, __LINE__);
        return JPEG_ERROR_MEMORY; // not enough memory for the decoder instance
    }
    rc = jpg->openRAM((uint8_t *)pJPEG, iDataSize, jpeg_draw);
    if (rc) {
        if (jpg->getWidth() != bbep.width() || jpg->getHeight() != bbep.height()) {
            Log_error("JPEG image size doesn't match display size");
            rc = -1;
        } else { // okay to decode
#ifdef BB_EPAPER
            //bbep.setPanelType(TWO_BIT_PANEL);
            Log_info("%s [%d]: Decoding jpeg as 1-bpp dithered\r\n", __FILE__, __LINE__);
            jpg->setPixelType(ONE_BIT_DITHERED); // request 1-bit dithered output
#else
            bbep.setMode(BB_MODE_4BPP);
            Log_info("%s [%d]: Decoding jpeg as 4-bpp dithered\r\n", __FILE__, __LINE__);
            jpg->setPixelType(FOUR_BIT_DITHERED); // request 4-bit dithered output
#endif
            pDither = (uint8_t *)malloc(jpg->getWidth() * 16);
            iPlane = 0;//1; // Decode first plane
            Log_info("%s [%d]: Decoding plane 0\r\n", __FILE__, __LINE__);
            jpg->setUserPointer((void *)&iPlane);
            jpg->decodeDither(pDither, 0);
            jpg->close();
            // Decode the second plane
//            iPlane = 2;
//            Log_info("%s [%d]: Decoding plane 1\r\n", __FILE__, __LINE__);
//            jpg->openRAM((uint8_t *)pJPEG, iDataSize, jpeg_draw);
//            jpg->setPixelType(TWO_BIT_DITHERED); // request 1-bit dithered output
//            jpg->setUserPointer((void *)&iPlane);
//            jpg->decodeDither(pDither, 0);
            free(pDither);
#ifdef BB_EPAPER
            rc = REFRESH_FULL;
#endif
        }
    }
    jpg->close();
    free(jpg);
    return rc;
} /* jpeg_to_epd() */
/**
 * @brief Function to decode and display a PNG image from memory
 *        The decoded lines are written directly into the EPD framebuffer
 *        due to insufficient RAM to hold the fully decoded image
 * @param pointer to the buffer holding the PNG file
 * @param size of the PNG file
 * @return refresh mode based on image type and presence of old image
 */
int png_to_epd(const uint8_t *pPNG, int iDataSize)
{
int iPlane = PNG_1_BIT, rc = -1;
PNG *png = new PNG();

    if (!png) {
        Log_error("%s [%d]: Not enough memory for the PNG decoder instance", __FILE__, __LINE__);
        return PNG_MEM_ERROR; // not enough memory for the decoder instance
    }
    rc = png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
    png->close();
    if (rc == PNG_SUCCESS) {
        Log_info("Decoding %d x %d PNG", png->getWidth(), png->getHeight());
        if (png->getWidth() == bbep.height() && png->getHeight() == bbep.width()) {
            Log_info("Rotating canvas to portrait orientation");
        } else if (png->getWidth() > bbep.width() || png->getHeight() > bbep.height()) {
            Log_info("PNG image is larger than the display (%dx%d), it will be cropped", png->getWidth(), png->getHeight());
        }
        if (rc == PNG_SUCCESS) { // okay to decode
            Log_info("%s [%d]: Decoding %d-bpp png (current)\r\n", __FILE__, __LINE__, png->getBpp());
            // Prepare target memory window (entire display)
#ifdef BB_EPAPER
#ifdef BOARD_SEEED_RETERMINAL_E1002
            if (bbep.allocBuffer() != BBEP_SUCCESS) {
                Log_error("%s [%d]: bbep.AllocBuffer failed!\n\r", __FILE__, __LINE__);
                return -1;
            }
            Log_info("%s [%d]: decoding for 6-color EPD with Floyd-Steinberg dithering\r\n", __FILE__, __LINE__);
            spectra6_fs_init(png->getWidth());
            png->openRAM((uint8_t *)pPNG, iDataSize, png_draw_6clr);
            png->decode(NULL, 0);
            png->close();
            spectra6_fs_free();
            if (fs_rgb_row) { free(fs_rgb_row); fs_rgb_row = NULL; }
            free(png);
            return REFRESH_FULL;
#endif // E1002
#ifdef BOARD_TRMNL_4CLR
            Log_info("%s [%d]: decoding for 4-color EPD\r\n", __FILE__, __LINE__);
            png->openRAM((uint8_t *)pPNG, iDataSize, png_draw_4clr);
            bbep.startWrite(PLANE_1); // start writing image data
            png->decode(NULL, 0);
            png->close();
            free(png); // free the decoder instance
            return REFRESH_FULL;
#endif // BOARD_TRMNL_4CLR
            bbep.setAddrWindow(0, 0, bbep.width(), bbep.height());
            if (png->getBpp() == 1 || (png->getBpp() == 2 && png_count_colors(png, pPNG, iDataSize) == 2)) { // 1-bit image (single plane)
                png->close(); // use a different PNGDraw callback for color matching
                bbep.setPanelType(dpList[iTempProfile].OneBit);
                rc = REFRESH_PARTIAL; // the new image is 1bpp - try a partial update
                bbep.startWrite(PLANE_0); // start writing image data to plane 0
                png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                if (png->getBpp() == 1 || png->getBpp() > 2) {
                    iPlane = PNG_1_BIT;
                    png->decode(&iPlane, 0);
                } else { // convert the 2-bit image to 1-bit output
                    Log_info("%s [%d]: Current png only has 2 unique colors!\n", __FILE__, __LINE__);
                    iPlane = PNG_2_BIT_BOTH;
                    if (png->decode(&iPlane, 0) != PNG_SUCCESS) {
                        Log_info("%s [%d]: Error decoding image = %d\n", __FILE__, __LINE__, png->getLastError());
                    }
                }
                png->close();
                if (bbep.getPanelType() != EP75_800x480) { // need to write the inverted plane to do PLANE_FALSE_DIFF
                    bbep.startWrite(PLANE_1); // start writing image data to plane 1
                    png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                    if (iPlane == PNG_1_BIT) {
                        iPlane = PNG_1_BIT_INVERTED; // inverted 1-bit to second memory plane
                    } else { // convert the 2-bit image to 1-bit output
                        iPlane = PNG_2_BIT_INVERTED; // inverted 2-bit -> 1-bit to second plane
                    }
                    png->decode(&iPlane, 0);
                } // temp profile needs the second plane written
            } else { // 2-bpp (or greater, but reduced to 2-bpp)
                bbep.setPanelType(dpList[iTempProfile].TwoBit);
                rc = REFRESH_FULL; // 4gray mode must be full refresh
                iUpdateCount = 0; // grayscale mode resets the partial update counter
                bbep.startWrite(PLANE_0); // start writing image data to plane 0
                iPlane = PNG_2_BIT_0;
                Log_info("%s [%d]: decoding 4-gray plane 0\r\n", __FILE__, __LINE__);
                png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                png->decode(&iPlane, 0); // tell PNGDraw to use bits for plane 0
                png->close(); // start over for plane 1
                iPlane = PNG_2_BIT_1;
                Log_info("%s [%d]: decoding 4-gray plane 1\r\n", __FILE__, __LINE__);
                png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                bbep.startWrite(PLANE_1); // start writing image data to plane 1
                png->decode(&iPlane, 0); // decode it again to get plane 1 data
            }
#else // FastEPD
            bbep.setMode((png->getBpp() == 1) ? BB_MODE_1BPP : BB_MODE_4BPP);
            png->decode(NULL, 0);
            png->close();
#endif
        }
    } else {
        Log_error("%s [%d]: png->openRAM() returned %d", __FILE__, __LINE__, rc);
    }
    free(png); // free the decoder instance
    return rc;
} /* png_to_epd() */
/**
 * @brief Function to show the image on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param reverse shows if the color scheme is reverse
 * @return none
 */
void display_show_image(uint8_t *image_buffer, int data_size, bool bWait)

{
    bool isPNG = data_size >= 4 && MOTOLONG(image_buffer) == (int32_t)0x89504e47;
    auto width = display_width();
    auto height = display_height();
//    uint32_t *d32;
    bool bAlloc = false;
#ifdef BB_EPAPER
    int iRefreshMode = REFRESH_FULL; // assume full (slow) refresh
#else
    int iRefreshMode = 0;
#endif

   // Log_info("Paint_NewImage %d", reverse);
    Log_info("display_show_image start");
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef FUTURE
    if (reverse)
    {
        d32 = (uint32_t *)image_buffer; // get framebuffer as a 32-bit pointer
        d32 = (uint32_t *)image_buffer; // get framebuffer as a 32-bit pointer
        Log_info("inverse the image");
        for (size_t i = 0; i < buf_size; i+=sizeof(uint32_t))
        for (size_t i = 0; i < buf_size; i+=sizeof(uint32_t))
        {
            d32[0] = ~d32[0];
            d32++;
            d32[0] = ~d32[0];
            d32++;
        }
    }
#endif
#ifdef BB_EPAPER
    if (i426Workaround) {
        // After a partial update, the 4.26" 800x480 needs to be 'reset' to accept writes
        // This is only needed if the user pressed the WAKE button and there will be 2 updates
        // while the power is on
        bbep.initIO(EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN, EPD_CS_PIN, EPD_MOSI_PIN, EPD_SCK_PIN, 8000000);
    }
#endif // BB_EPAPER
    if (isPNG == true && data_size < MAX_IMAGE_SIZE)
    {
        Log_info("Drawing PNG");
        iRefreshMode = png_to_epd(image_buffer, data_size);
    }
    else if (MOTOSHORT(image_buffer) == 0xffd8) {
        Log_info("Drawing JPEG");
        iRefreshMode = jpeg_to_epd(image_buffer, data_size);
    }
    else // uncompressed BMP or Group5 compressed image
    {
        if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER)
        {
            // G5 compressed image
            BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
#ifdef BB_EPAPER
            bbep.allocBuffer(false);
            bAlloc = true;
#endif
            int x = (width - pBBB->width)/2;
            int y = (height - pBBB->height)/2; // center it
            if (x > 0 || y > 0) // only clear if the image is smaller than the display
            {
                bbep.fillScreen(BBEP_WHITE);
            }
            bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
        }
        else
        {
#ifdef BOARD_SEEED_RETERMINAL_E1002
         // Spectra 6 uses 4-bpp buffer; raw 1-bpp BMP can't be set directly.
            bbep.allocBuffer(false);
            bAlloc = true;
            bbep.fillScreen(BBEP_WHITE);
#else
         // This work-around is due to a lack of RAM; the correct method would be to use loadBMP()
            flip_image(image_buffer+62, bbep.width(), bbep.height(), false); // fix bottom-up bitmap images
#ifdef BB_EPAPER
            bbep.setBuffer(image_buffer+62); // uncompressed 1-bpp bitmap
#endif
#endif
        }
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
#ifdef BOARD_XTEINK_X4
        bbep.writePlane(PLANE_FALSE_DIFF);
#else
        bbep.writePlane(); // send image data to the EPD
#endif
#endif // !E1002 — spectra6_update reads buffer directly
        iRefreshMode = REFRESH_PARTIAL;
#endif
        iUpdateCount = 1; // use partial update
    }
    Log_info("Display refresh start");
#ifdef BB_EPAPER
    if (iTempProfile != apiDisplayResult.response.temp_profile) {
        iTempProfile = apiDisplayResult.response.temp_profile;
        Log_info("Saving new temperature profile (%d) to FLASH", iTempProfile);
        preferences.putUInt(PREFERENCES_TEMP_PROFILE, iTempProfile);
    }
    if ((iUpdateCount & 7) == 0 || apiDisplayResult.response.maximum_compatibility == true) {
        Log_info("%s [%d]: Forcing full refresh; desired refresh mode was: %d\r\n", __FILE__, __LINE__, iRefreshMode);
        iRefreshMode = REFRESH_FULL; // force full refresh every 8 partials
    }
    int refresh_seconds = preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP);
    if (refresh_seconds >= 30*60 && iRefreshMode == REFRESH_PARTIAL) {
        // For users who set updates 30 minutes or longer, use the "fast" update to prevent ghosting
        Log_info("%s [%d]: Forcing fast refresh (not partial) since the TRMNL refresh_rate is set to > 30 min\n", __FILE__, __LINE__);
        iRefreshMode = REFRESH_FAST;
    }
    if (bbep.capabilities() & (BBEP_4COLOR | BBEP_3COLOR | BBEP_7COLOR)) bWait = 1;
    if (!bWait) iRefreshMode = REFRESH_PARTIAL; // fast update when showing loading screen
    Log_info("%s [%d]: EPD refresh mode: %d\r\n", __FILE__, __LINE__, iRefreshMode);
    bbep.setLightSleep(true);
#ifdef BOARD_SEEED_RETERMINAL_E1002
    spectra6_update();
#else
    bbep.refresh(iRefreshMode, bWait);
#endif
    if ((bbep.getPanelType() == EP426_800x480 || bbep.getPanelType() == EP397_800x480) && iRefreshMode == REFRESH_PARTIAL) {
        i426Workaround = 1; // need to re-initialize the controller for another update before sleeping
    }
    if (bAlloc) {
        bbep.freeBuffer();
    }
    iUpdateCount++;
#else
    bbep.setCustomMatrix(u8_graytable, sizeof(u8_graytable));
    bbep.fullUpdate();
#endif
    Log_info("display_show_image end");
}
/**
 * @brief Function to read an image from the file system
 * @param filename
 * @param pointer to file size returned
 * @return pointer to allocated buffer
 */
uint8_t * display_read_file(const char *filename, int *file_size)
{
File f = SPIFFS.open(filename, "r");
uint8_t *buffer;

  if (!f) {
    Serial.println("Failed to open file!");
    *file_size = 0;
    return nullptr;
  }
  *file_size = f.size();
  buffer = (uint8_t *)malloc(*file_size);
  if (!buffer) {
    Serial.println("Memory allocation filed!");
    *file_size = 0;
    return nullptr;
  }
  f.read(buffer, *file_size);
  f.close();
  return buffer;
} /* display_read_file() */

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type)
{
    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    BB_RECT rect;

    Log_info("display_show_msg start");
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef BB_EPAPER
    bbep.allocBuffer(false);
#endif
    if (image_buffer && *(uint16_t *)image_buffer == BB_BITMAP_MARKER)
    {
        // G5 compressed image
        BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
        int x = (width - pBBB->width)/2;
        int y = (height - pBBB->height)/2; // center it
        if (x > 0 || y > 0) // only clear if the image is smaller than the display
        {
            bbep.fillScreen(BBEP_WHITE);
        }
        bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
    }
    else
    {
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
        if (image_buffer) memcpy(bbep.getBuffer(), image_buffer+62, Imagesize); // uncompressed 1-bpp bitmap
#else
        bbep.fillScreen(BBEP_WHITE); // Spectra 6 uses 4-bpp buffer; can't memcpy 1-bpp BMP
#endif
#endif
    }

#ifdef BOARD_TRMNL_X
    bbep.setFont(Inter_18);
#else
    bbep.setFont(nicoclean_8);
#endif
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);

    switch (message_type)
    {
    case WIFI_CONNECT:
    {
        const char string1[] = "Connect to TRMNL WiFi";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);
        const char string2[] = "on your phone or computer";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, -1);
        bbep.print(string2);
    }
    break;
    case WIFI_FAILED:
    {
        String string0 = "TRMNL firmware ";
        string0 += FW_VERSION_STRING;
#ifdef __BB_EPAPER__
        bbep.setCursor(40, 48); // place in upper left corner
#else
        bbep.setCursor(80, 104); // place in upper left corner
#endif
        bbep.println(string0);
        const char string1[] = "Can't establish WiFi connection.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, bbep.height() - (rect.h*2)-140);
        bbep.println(string1);
        const char string2[] = "Hold button on the back to reset WiFi, or scan QR Code for help.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - 66 - 40, 40, BBEP_WHITE, BBEP_BLACK);
#else // bigger for X
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
    }
    break;
    case WIFI_INTERNAL_ERROR:
    {
        const char string1[] = "WiFi connected, but";
#ifdef __BB_EPAPER__
        int x = 132;
#else
        int x = 0;
#endif
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, 340);
#else
        bbep.setCursor((bbep.width() - rect.w)/2, bbep.height() - (rect.h*2)-140);
#endif
        bbep.println(string1);
        const char string2[] = "API connection cannot be";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - x - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "established. Try to refresh,";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - x - rect.w) / 2, -1);
        bbep.println(string3);
        const char string4[] = "or scan QR Code for help.";
        bbep.getStringBox(string4, &rect);
        bbep.setCursor((bbep.width() - x - rect.w) / 2, -1);
        bbep.print(string4);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_failed_qr, 639, 336, BBEP_WHITE, BBEP_BLACK);
#else // bigger for X
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
    }
    break;
    case WIFI_WEAK:
    {
        const char string1[] = "WiFi connected but signal is weak";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case API_REQUEST_FAILED:
    {
        const char string1[] = "WiFi connected, request to API failed.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
        bbep.println(string1);
        const char string2[] = "Short click the button on back,";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "otherwise check your internet.";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case API_UNABLE_TO_CONNECT:
    {
        const char string1[] = "WiFi connected, unable connect to API.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
        bbep.println(string1);
        const char string2[] = "Short click the button on back,";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "otherwise check your internet.";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case API_SETUP_FAILED:
    {
        const char string1[] = "WiFi connected, /api/setup returned error.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - (rect.h*3));
#endif
        bbep.println(string1);
        const char string2[] = "Short click the button on back,";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "otherwise check your internet.";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case API_SIZE_ERROR:
    {
        const char string1[] = "WiFi connected, TRMNL content malformed.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - (rect.h*2));
#endif
        bbep.println(string1);
        const char string2[] = "Wait or reset by holding button on back.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string2);
    }
    break;
    case API_FIRMWARE_UPDATE_ERROR:
    {
        const char string1[] = "WiFi connected, could not get firmware update from api.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.println(string1);
        const char string2[] = "Wait or reset by holding button on back.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string2);
    }
    break;
    case API_IMAGE_DOWNLOAD_ERROR:
    {
        const char string1[] = "WiFi connected, API could not deliver image to device.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.println(string1);
        const char string2[] = "Wait or reset by holding button on back.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string2);
    }
    break;
    case FW_UPDATE:
    {
        const char string1[] = "Firmware update available! Starting now...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case FW_UPDATE_FAILED:
    {
        const char string1[] = "Firmware update failed. Device will restart...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case FW_UPDATE_SUCCESS:
    {
        const char string1[] = "Firmware update success. Device will restart...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case QA_START:
    {
        const char string1[] = "Starting QA test";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.print(string1);
    }
    break;
    case MSG_TOO_BIG:
    {
        const char string1[] = "The image file from this URL is too large.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 360);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h*4);
#endif
        bbep.println(string1);
        if (strlen(filename) > 40) {
            filename[40] = 0; // truncate and add elipses
            strcat(filename, "...");
        }
        bbep.getStringBox(filename, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(filename);

        const char string2[] = "PNG images can be a maximum of";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
#ifdef __BB_EPAPER__
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each and 1 or 2-bpp");
#else
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each");
#endif
        bbep.getStringBox(string3.c_str(), &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case MSG_FORMAT_ERROR:
    {
        const char string1[] = "The image format is incorrect";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case TEST:
    {
        bbep.setCursor(0, 40);
        bbep.println("ABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIY");
        bbep.println("abcdefghiyabcdefghiyabcdefghiyabcdefghiyabcdefghiy");
        bbep.println("A B C D E F G H I Y A B C D E F G H I Y A B C D E");
        bbep.println("a b c d e f g h i y a b c d e f g h i y a b c d e");
    }
    break;
    case FILL_WHITE:
    {
        Log_info("Display set to white");
        bbep.fillScreen(BBEP_WHITE);
    }
    default:
        break;
    }
#ifdef BB_EPAPER
#ifdef BOARD_SEEED_RETERMINAL_E1002
    spectra6_update();
#else
    bbep.writePlane(PLANE_0);
    bbep.refresh(REFRESH_FULL, true);
#endif
    bbep.freeBuffer();
#else
    bbep.fullUpdate();
#endif
    Log_info("display_show_msg end");
}


void display_show_msg_qa(uint8_t *image_buffer, const float *voltage, const float *temperature, bool qa_result)
{
    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    BB_RECT rect;

    Log_info("display_show_msg start");
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef BB_EPAPER
    bbep.allocBuffer(false);
#endif
    if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER)
    {
        // G5 compressed image
        BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
        int x = (width - pBBB->width)/2;
        int y = (height - pBBB->height)/2; // center it
        if (x > 0 || y > 0) // only clear if the image is smaller than the display
        {
            bbep.fillScreen(BBEP_WHITE);
        }
        bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
    }
    else
    {
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
        memcpy(bbep.getBuffer(), image_buffer+62, Imagesize); // uncompressed 1-bpp bitmap
#else
        bbep.fillScreen(BBEP_WHITE); // Spectra 6 uses 4-bpp buffer; can't memcpy 1-bpp BMP
#endif
#endif
    }

    bbep.setFont(nicoclean_8); //Roboto_20);
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);

    String voltageString = String("Initial voltage: ")
    + String(voltage[0], 4)
    + String(" V, ")
    + String("  Final voltage: ")
    + String(voltage[1], 4)
    + String(" V, ")
    + String("  Diff: ")
    + String(voltage[2], 4)
    + String(" V");

    String temperatureString = String("Initial temperature: ")
    + String(temperature[0], 4)
    + String(" C, ")
    + String("  Final temperature: ")
    + String(temperature[1], 4)
    + String(" C")
    + String("  Diff: ")
    + String(temperature[2], 4)
    + String(" C");


    bbep.getStringBox(voltageString.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 340);
    bbep.print(voltageString);

    bbep.getStringBox(temperatureString.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 370);
    bbep.print(temperatureString);

    String qaResultInstruction = (qa_result)
    ? "QA passed, press button to clear screen"
    : "QA failed, please use another board and put in failure pile for investigation";

    bbep.getStringBox(qaResultInstruction.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 400);
    bbep.println(qaResultInstruction);

    String qaResultString = (qa_result) ? "PASS" : "FAIL";
    bbep.setFont(Roboto_Black_24);
    bbep.getStringBox(qaResultString.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 250);
    bbep.print(qaResultString);

    #ifdef BB_EPAPER
    #ifdef BOARD_SEEED_RETERMINAL_E1002
        spectra6_update();
    #else
        bbep.writePlane(PLANE_0);
        bbep.refresh(REFRESH_FULL, true);
    #endif
        bbep.freeBuffer();
    #else
        bbep.fullUpdate();
    #endif
        Log_info("display_show_msg end");
    /*
     const char string2[] = "PNG images can be a maximum of";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each and 1 or 2-bpp");
        bbep.getStringBox(string3.c_str(), &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    */
}

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @param friendly_id device friendly ID
 * @param id shows if ID exists
 * @param fw_version version of the firmware
 * @param message additional message
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type, String friendly_id, bool id, const char *fw_version, String message)
{
    Log_info("Free heap in display_show_msg - %d", ESP.getMaxAllocHeap());
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef BB_EPAPER
    bbep.allocBuffer(false);
    Log_info("Free heap after bbep.allocBuffer() - %d", ESP.getMaxAllocHeap());
#endif

    if (message_type == WIFI_CONNECT)
    {
        Log_info("Display set to white");
        bbep.fillScreen(BBEP_WHITE);
#ifdef BB_EPAPER
#ifdef BOARD_SEEED_RETERMINAL_E1002
        spectra6_update();
#else
        bbep.writePlane(PLANE_0);
        if (!apiDisplayResult.response.maximum_compatibility) {
            bbep.refresh(REFRESH_FAST, true);
        } else {
            bbep.refresh(REFRESH_FULL, true);
        }
#endif
#else
        bbep.fullUpdate();
#endif
        display_sleep(1000);
    }

    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    BB_RECT rect;

    Log_info("display_show_msg2 start");

    // Load the image into the bb_epaper framebuffer
    if (image_buffer && *(uint16_t *)image_buffer == BB_BITMAP_MARKER)
    {
        // G5 compressed image
        BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
        int x = (width - pBBB->width)/2;
        int y = (height - pBBB->height)/2; // center it
        if (x > 0 || y > 0) // only clear if the image is smaller than the display
        {
            bbep.fillScreen(BBEP_WHITE);
        }
        bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
    }
    else
    {
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
        if (image_buffer) memcpy(bbep.getBuffer(), image_buffer+62, Imagesize); // uncompressed 1-bpp bitmap
#else
        bbep.fillScreen(BBEP_WHITE); // Spectra 6 uses 4-bpp buffer; can't memcpy 1-bpp BMP
#endif
#endif
    }

#ifdef BOARD_TRMNL_X
    bbep.setFont(Inter_18);
#else
    bbep.setFont(nicoclean_8);
#endif
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
    switch (message_type)
    {
    case FRIENDLY_ID:
    {
        Log_info("friendly id case");
        const char string1[] = "Please sign up at trmnl.com/start";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w)/2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w)/2, bbep.height() - 140 - rect.h*2);
#endif
        bbep.println(string1);

        String string2 = "with Friendly ID ";
        if (id)
        {
            string2 += friendly_id;
        }
        string2 += " to finish setup";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, -1);
        bbep.print(string2);
    }
    break;
    case WIFI_CONNECT:
    {
        Log_info("wifi connect case");

        String string1 = "TRMNL firmware ";
        string1 += fw_version;
        bbep.setCursor(40, 48); // place in upper left corner
        bbep.println(string1);
        const char string2[] = "Connect your phone or computer to TRMNL WiFi network";
        bbep.getStringBox(string2, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 386);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h*2);
#endif
        bbep.println(string2);
        const char string3[] = "or scan the QR code for help";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_connect_qr, bbep.width() - 40 - 66, 40, BBEP_WHITE, BBEP_BLACK); // 66x66 QR code
#else // bigger for X
        bbep.loadG5Image(wifi_connect_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
    }
    break;
    case MAC_NOT_REGISTERED:
    {
        UWORD y_start = 340;
        UWORD font_width = 18; // DEBUG
        Paint_DrawMultilineText(0, y_start, message.c_str(), width, font_width, BBEP_BLACK, BBEP_WHITE,
#ifdef BOARD_TRMNL_X
        Inter_18, true);
#else
        nicoclean_8, true);
#endif
    }
    break;
    default:
        break;
    }
    Log_info("Start drawing...");
#ifdef BB_EPAPER
#ifdef BOARD_SEEED_RETERMINAL_E1002
    spectra6_draw_color_boxes();
    spectra6_update();
#else
    bbep.writePlane(PLANE_0);
    bbep.refresh(REFRESH_FULL, true);
#endif
    bbep.freeBuffer();
#else
    bbep.fullUpdate();
#endif
    Log_info("display_show_msg2 end");
}

/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void)
{
    Log_info("Goto Sleep...");
#ifdef BB_EPAPER
#ifndef BOARD_SEEED_RETERMINAL_E1002
    bbep.sleep(DEEP_SLEEP);
#endif
    // E1002: panel already in deep sleep after spectra6_update()
#else
    bbep.einkPower(0);
    bbep.deInit();
#endif
}
