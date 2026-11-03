#include <PNGdec.h>
#include <color_reduce.h>
#include <epd_driver.h>
#include <esp_heap_caps.h>
#include <globals.h>
#include <trmnl_log.h>

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
void ReduceBpp(int iDestBpp, int iPixelType, uint8_t *pPalette, uint8_t *pSrc, uint8_t *pDest, int w, int iSrcBpp) {
  int g = 0, x, iDelta;
  uint8_t *s, *d, *pPal, u8, count;
  const uint8_t u8G2ToG8[4] = {0x00, 0x55, 0xaa, 0xff}; // 2-bit to 8-bit gray

  if (iPixelType == PNG_PIXEL_TRUECOLOR)
    iSrcBpp = 24;
  else if (iPixelType == PNG_PIXEL_TRUECOLOR_ALPHA)
    iSrcBpp = 32;
  iDelta = iSrcBpp / 8; // bytes per pixel
  count = 8; // bits in a byte
  u8 = 0; // start with all black
  d = pDest;
  s = pSrc;
  for (x = 0; x < w; x++) {
    u8 <<= iDestBpp;
    switch (iSrcBpp) {
    case 24:
    case 32:
      g = (s[0] + s[1] * 2 + s[2]) / 4; // convert color to gray value
      s += iDelta;
      break;
    case 8:
      if (iPixelType == PNG_PIXEL_INDEXED) {
        pPal = &pPalette[s[0] * 3];
        g = (pPal[0] + pPal[1] * 2 + pPal[2]) / 4;
      } else { // must be grayscale
        g = s[0];
      }
      s++;
      break;
    case 4:
      if (x & 1) {
        if (iPixelType == PNG_PIXEL_INDEXED) {
          pPal = &pPalette[(s[0] & 0xf) * 3];
          g = (pPal[0] + pPal[1] * 2 + pPal[2]) / 4;
        } else {
          g = (s[0] & 0xf) | ((s[0] & 0xf) << 4); // the low nibble only: g is an int
        }
        s++;
      } else {
        if (iPixelType == PNG_PIXEL_INDEXED) {
          pPal = &pPalette[(s[0] >> 4) * 3];
          g = (pPal[0] + pPal[1] * 2 + pPal[2]) / 4;
        } else {
          g = (s[0] & 0xf0) | (s[0] >> 4);
        }
      }
      break;
    case 2: // We need to handle this case for 2-bit images with (random) palettes
      g = s[0] >> (6 - ((x & 3) * 2));
      if (iPixelType == PNG_PIXEL_INDEXED) {
        pPal = &pPalette[(g & 3) * 3];
        g = (pPal[0] + pPal[1] * 2 + pPal[2]) / 4;
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
    } else if (iDestBpp == 2) { // generate 4 gray levels (2 bits)
      u8 |= (3 ^ (g >> 6)); // 4 gray levels (inverted relative to 1-bit)
    } else { // must be 4-bpp output
      u8 |= (g >> 4);
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

#ifdef BB_EPAPER
uint8_t u8SpectraPal[512]; // RGB333 mapped to closest Spectra6 color

//
// Match the given pixel to black (00), white (01), or red (1x)
//
unsigned char GetBWRPixel(int r, int g, int b) {
  uint8_t ucOut = BBEP_BLACK;
  int gr;

  gr = (b + r + g * 2) >> 2; // gray
    // match the color to closest of black/white/red
  if (r > g && r > b) { // red is dominant
    if (gr < 100 && r < 80) {
            // black
    } else {
      if (r - b > 32 && r - g > 32) {
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
unsigned char GetBWYRPixel(int r, int g, int b) {
  uint8_t ucOut = BBEP_BLACK;
  int gr;

  gr = (b + r + g * 2) >> 2; // gray
    // match the color to closest of black/white/yellow/red
  if (r > b || g > b) { // red or yellow is dominant
    if (gr < 90 && r < 80 && g < 80) {
            // black
    } else {
      if (r - b > 32 && r - g > r / 2) {
                // is red really dominant?
        ucOut = BBEP_RED; // red
      } else if (r - b > 32 && g - b > 32) {
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

//
// Calibrated Spectra6 palette colors (measured panel reflectance)
// Indices match bb_epaper color constants:
// 0=Black, 1=White, 2=Yellow, 3=Red, 4=Blue, 5=Green
//
// clang-format off
const int iSpectraRGB[] = { // r, g, b
    0,   0,   0,   // black = 0
    255, 255, 255, // white = 1
    255, 243, 56,  // yellow = 2
    191, 0,   0,   // red = 3
    100, 64,  255, // blue = 4
    67,  138, 28,  // green = 5
};

static const int spectra6_palette[6][3] = {
    {  0,   0,   0}, // 0: Black  -> BBEP_BLACK  (0)
    {255, 255, 255}, // 1: White  -> BBEP_WHITE  (1)
    {255, 243,  56}, // 2: Yellow -> BBEP_YELLOW (2)
    {191,   0,   0}, // 3: Red    -> BBEP_RED    (3)
    {100,  64, 255}, // 4: Blue   -> BBEP_BLUE   (4)
    { 67, 138,  28}, // 5: Green  -> BBEP_GREEN  (5)
};
// clang-format on

uint8_t spectra6_find_closest(int r, int g, int b) {
  int min_dist = 0x7fffffff;
  uint8_t best = 0;
  for (int i = 0; i < 6; i++) {
    int dr = r - spectra6_palette[i][0];
    int dg = g - spectra6_palette[i][1];
    int db = b - spectra6_palette[i][2];
    int dist = dr * dr + dg * dg + db * db;
    if (dist < min_dist) {
      min_dist = dist;
      best = i;
    }
  }
  return best;
}

// Floyd-Steinberg error buffers for streaming PNG decode (2 rows of RGB int16_t errors)
static int16_t *s_fs_err_cur = nullptr;
static int16_t *s_fs_err_nxt = nullptr;
static int s_fs_width = 0;

bool spectra6_fs_init(int width) {
  spectra6_fs_free();
  s_fs_width = width;
  size_t sz = (width + 2) * 3 * sizeof(int16_t);
  s_fs_err_cur = (int16_t *)heap_caps_calloc(1, sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  s_fs_err_nxt = (int16_t *)heap_caps_calloc(1, sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_fs_err_cur || !s_fs_err_nxt) {
    if (!s_fs_err_cur) s_fs_err_cur = (int16_t *)calloc(1, sz);
    if (!s_fs_err_nxt) s_fs_err_nxt = (int16_t *)calloc(1, sz);
  }
  if (!s_fs_err_cur || !s_fs_err_nxt) {
    Log_error("spectra6_fs_init: failed to alloc error buffers");
    spectra6_fs_free();
    return false;
  }
  return true;
}

void spectra6_fs_free(void) {
  if (s_fs_err_cur) {
    free(s_fs_err_cur);
    s_fs_err_cur = nullptr;
  }
  if (s_fs_err_nxt) {
    free(s_fs_err_nxt);
    s_fs_err_nxt = nullptr;
  }
  s_fs_width = 0;
}

static inline int clamp255(int v) {
  return v < 0 ? 0 : (v > 255 ? 255 : v);
}

void spectra6_fs_dither_row(uint8_t *rgbRow, int y, int width, bool bRotated) {
  for (int x = 0; x < width; x++) {
    int idx = (x + 1) * 3;
    int r = clamp255(rgbRow[x * 3 + 0] + s_fs_err_cur[idx + 0]);
    int g = clamp255(rgbRow[x * 3 + 1] + s_fs_err_cur[idx + 1]);
    int b = clamp255(rgbRow[x * 3 + 2] + s_fs_err_cur[idx + 2]);

    uint8_t ci = spectra6_find_closest(r, g, b);

    if (bRotated) {
      bbep.drawPixel(bbep.width() - 1 - y, x, ci);
    } else {
      bbep.drawPixel(x, y, ci);
    }

    int er = r - spectra6_palette[ci][0];
    int eg = g - spectra6_palette[ci][1];
    int eb = b - spectra6_palette[ci][2];

    // Right (x+1, y): 7/16
    s_fs_err_cur[idx + 3 + 0] += (er * 7) >> 4;
    s_fs_err_cur[idx + 3 + 1] += (eg * 7) >> 4;
    s_fs_err_cur[idx + 3 + 2] += (eb * 7) >> 4;
    // Bottom-left (x-1, y+1): 3/16
    s_fs_err_nxt[idx - 3 + 0] += (er * 3) >> 4;
    s_fs_err_nxt[idx - 3 + 1] += (eg * 3) >> 4;
    s_fs_err_nxt[idx - 3 + 2] += (eb * 3) >> 4;
    // Bottom (x, y+1): 5/16
    s_fs_err_nxt[idx + 0] += (er * 5) >> 4;
    s_fs_err_nxt[idx + 1] += (eg * 5) >> 4;
    s_fs_err_nxt[idx + 2] += (eb * 5) >> 4;
    // Bottom-right (x+1, y+1): 1/16
    s_fs_err_nxt[idx + 3 + 0] += (er * 1) >> 4;
    s_fs_err_nxt[idx + 3 + 1] += (eg * 1) >> 4;
    s_fs_err_nxt[idx + 3 + 2] += (eb * 1) >> 4;
  }
  int16_t *tmp = s_fs_err_cur;
  s_fs_err_cur = s_fs_err_nxt;
  s_fs_err_nxt = tmp;
  memset(s_fs_err_nxt, 0, (s_fs_width + 2) * 3 * sizeof(int16_t));
}

// Map the Spectra6 palette to the closest RGB333 values
void CreateSpectra6Pal(void) {
  int i, j;
  int r, g, b, r1, g1, b1;
  int dist, min_dist, min_index;

  for (i = 0; i < 512; i++) { // RGB333
    r = (i & 7) * 36;
    g = ((i >> 3) & 7) * 36;
    b = (i >> 6) * 36;
    min_dist = 0x7fffffff;
    min_index = 0;
    for (j = 0; j < 6; j++) { // match to the closes Spectra6 color
      r1 = iSpectraRGB[j * 3];
      g1 = iSpectraRGB[j * 3 + 1];
      b1 = iSpectraRGB[j * 3 + 2];
      dist = (r - r1) * (r - r1); // delta red squared
      dist += (g - g1) * (g - g1); // delta green squared
      dist += (b - b1) * (b - b1); // delta blue squared
      if (dist < min_dist) {
        min_dist = dist;
        min_index = j;
      }
    } // for j
    u8SpectraPal[i] = min_index; // best match palette index for this RGB333 color
  } // for i
} /* CreateSpectra6Pal() */
//
// Convert the RGB value into one of 6 Spectra6 colors
//
uint8_t GetSpectraPixel(int r, int g, int b) {
  return spectra6_find_closest(r, g, b);
} /* GetSpectraPixel() */
#endif // BB_EPAPER
