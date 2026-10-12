# DIY Colormeter — AS7341 + ESP32 Cheap Yellow Display

Live 8-channel visible spectrum (415–680 nm) plus Clear and NIR from an Adafruit AS7341,
shown on an ESP32-2432S028R 2.8" touch display, with background subtraction and a
phone web app over a WiFi access point.

## Hardware

| Part | Notes |
|---|---|
| ESP32-2432S028R "Cheap Yellow Display" | 2.8" 240x320 ILI9341, XPT2046 resistive touch, dual USB |
| Adafruit AS7341 breakout | I2C address 0x39, pull-ups on board, 3.3 V |

Wiring: sensor to connector **CN1** (4-pin JST 1.25 mm, next to the SD slot).

| CN1 pin | Sensor |
|---|---|
| GND | GND |
| GPIO22 | SCL |
| GPIO27 | SDA |
| 3V3 | VIN |

GPIO21 on the other connector is the display backlight and cannot be used for I2C.

## Build and flash (Arduino IDE)

1. Board: **ESP32 Dev Module** (esp32 core 3.x). Port: the CH340 COM port.
2. Libraries: `TFT_eSPI`, `XPT2046_Touchscreen`, `Adafruit AS7341` (+ `Adafruit BusIO`).
3. Copy `TFT_eSPI_User_Setup_CYD.h` over `Documents/Arduino/libraries/TFT_eSPI/User_Setup.h`.
4. Open `AS7341_Spectrum/AS7341_Spectrum.ino` and upload.

If the screen shows inverted colours (white background), uncomment `TFT_INVERSION_ON`
in the User_Setup file. Some dual-USB boards ship with an ST7789 panel.

## Using it

**MAIN screen** — live chart, bars coloured by wavelength, white spline through the 8 visible channels.
Header shows an estimated colour swatch, peak channel, gain, integration time and update rate.

| Button | Action |
|---|---|
| SET BG / CLR BG | Average 10 readings as background; subtract it from every later reading. Tap again to clear. |
| LED | Sensor white LED (reflectance measurements). Default off. |
| RUN / HOLD | Freeze the display. |
| WIFI | Access-point screen. |
| SETUP | Display options. |

**SETUP screen** — VIEW (bars / line / both), SCALE (NORM: tallest bar = 100 %, ABS: counts/s at unit gain),
GAIN (AUTO or 1x–512x), CLR/NIR (show the Clear and NIR bars, hidden by default). Saved to flash on BACK.

**WIFI screen** — `-` / `+` change the network name suffix (`Colormeter-01` … `-99`) so several
devices can run side by side. AP ON starts an open network. On the phone, connect to the
network and open **http://192.168.4.1**. The page shows the same chart, has SET BG / LED / HOLD /
GAIN buttons, and can record readings (Record, or one Snapshot at a time) and save them as a
CSV file to the phone's Downloads folder.

**Serial** — 115200 baud, one CSV line per reading: raw ADC counts for all 10 channels and the
background-subtracted normalised values.

## Values

Displayed values are *basic counts*: raw ADC counts divided by gain and integration time
(counts per second at unit gain). They stay comparable when auto-gain changes, and are the
quantity the background is subtracted in. Red outline on a bar = that channel saturated.

Integration: ATIME 29, ASTEP 599 → 50 ms per bank, two banks per reading, about 8 readings/s.
