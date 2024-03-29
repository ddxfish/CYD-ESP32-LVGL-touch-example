# CYD-ESP32-LVGL-touch-example
Example code for the ESP32 dev board known as the CYD or Cheap Yellow Display. AKA ESP32-2432S028, ESP32-2432S024R. This project is a beginner example. It is heavily commented code, and coded without classes or separate files. Every device on the board is used in this example, as well as LVGL for graphics.
![esp32-2432s028-demo-example](https://github.com/ddxfish/CYD-ESP32-LVGL-touch-example/assets/6764685/2292044e-8565-4174-9b1b-cc87066512c5)


## Usage
This is for platform.io. Load the project files, and adjust platformio.ini to your needs. Switch between ILI9341 and ST7796 by commenting in/out the code you need in platform.io.

## This Demo Tests For
*Backlight  
*TFT_eSPI display driver  
*LVGL graphics  
*LED toggle (random color)  
*Speaker buzzer  
*LDS/CdS/photoresistor light sensor  
*XPT2040 Touchscreen  
*SD/TF Card  

## Videos
### Touch, LED, Speaker
https://github.com/ddxfish/CYD-ESP32-LVGL-touch-example/assets/6764685/63705bf9-8ba6-4354-b953-73b7901e280d

### Light Sensor (LDR, CdS, photoresistor)
https://github.com/ddxfish/CYD-ESP32-LVGL-touch-example/assets/6764685/316df193-19b4-4409-bbd0-6bca2e2b871b

## Credits 
Code from @AllanOricil helped a lot: https://github.com/AllanOricil/esp32-lvgl-lcd-touch-sd-card




## Software and Libraries Used
LVGL: An open-source graphics library for creating embedded GUIs.  
TFT_eSPI: A graphics library for ESP8266 and ESP32 that supports different TFT displays.  
XPT2046_Bitbang: A library for interfacing with XPT2046 resistive touch controllers using bit-banging for SPI communication.  
PlatformIO: Development platforms used for writing, compiling, and uploading the code to ESP32.  
