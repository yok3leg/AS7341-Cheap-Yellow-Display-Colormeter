// TFT_eSPI User_Setup.h for ESP32-2432S028R "Cheap Yellow Display" (2.8", ILI9341, 240x320)
// Installed by the Colormeter project. Original file kept as User_Setup.h.orig
#define USER_SETUP_INFO "CYD ESP32-2432S028R"

#define ILI9341_2_DRIVER
// If colours look inverted (white shows as black) on a dual-USB board, uncomment ONE of:
// #define TFT_INVERSION_ON
// #define TFT_INVERSION_OFF
// If colours are wrong (red/blue swapped), try: #define TFT_RGB_ORDER TFT_BGR

#define TFT_WIDTH  240
#define TFT_HEIGHT 320

#define TFT_BL   21
#define TFT_BACKLIGHT_ON HIGH

#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

#define USE_HSPI_PORT
#define SPI_FREQUENCY       55000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000
