# GliderFlapDisplay
Display the current flap setting of a glider

![](etc/TTGO T4.png)

## TFT_eSPI settings
- open `.pio/libdeps/TFT_eSPI/User_Setup_Select.h`
- comment `//#include <User_Setup.h`
- uncomment `#include <User_Setups/Setup22_TTGO_T4_v1.3.h>`

## T4 
| Name        | V13     |
| ----------- | ------- |
| TFT Driver  | ILI9341 |
| TFT_MISO    | 12      |
| TFT_MOSI    | 23      |
| TFT_SCLK    | 18      |
| TFT_CS      | 27      |
| TFT_DC      | 32      |
| TFT_RST     | 5       |
| TFT_BL      | 4       |
| SD_MISO     | 2       |
| SD_MOSI     | 15      |
| SD_SCLK     | 14      |
| SD_CS       | 13      |
| Button 0    | 14      |
| Button 1    | 5       |
| Button 2    | 27      |
| Button 3    | 25      |
| SPEAKER PWD | 19      |
| SPEAKER OUT | 25      |
| I2C_SDA     | 21      |
| I2C_SCL     | 22      |