#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include <SD.h>
#include "SPI.h"
#include "XPT2046_Bitbang.h"

//TF Card
#define SD_CS 5 // Adjust to your SD card CS pin

//TFT_eSPI used as LVGL display driver
TFT_eSPI tft = TFT_eSPI(); /* TFT instance */

//LVGL (not all of this has to be global, but it's easier for now)
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10]; // /* Declare a buffer for 10 lines */
static lv_disp_drv_t disp_drv;  // Display driver
lv_obj_t *atextlabel = nullptr;

//Our globals. dont use global variables like this. stop it, get some help.
int counter = 0;
String printthis = "";

//Touchscreen
#define MOSI_PIN 32
#define MISO_PIN 39
#define CLK_PIN  25
#define CS_PIN   33
#define RERUN_CALIBRATE true
XPT2046_Bitbang touchscreen(MOSI_PIN, MISO_PIN, CLK_PIN, CS_PIN);

// Flush display - this is the link between LVGL and TFT_eSPI for LVGL v8
void cyd_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    Serial.println("Flushing display...");
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);

    // Use the pushColors() method to write the pixel data to the display.
    tft.pushColors((uint16_t*)color_p, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp_drv); // Inform LVGL that flushing is done
    Serial.println("Display flushed.");
}


void initTouchscreen(){
    // Initialize the touchscreen
    touchscreen.begin();
    // Check for existing calibration data
    if (!touchscreen.loadCalibration()) {
        Serial.println("Failed to load calibration data from SPIFFS.");
    }
}

void initSPIFFS(){
    // Initialize SPIFFS for calibration data
    if (!SPIFFS.begin(true)) {
        Serial.println("An error has occurred while mounting SPIFFS");
        return;
    }
}

void calibrateTouchscreen(){
    Serial.println("Re-running calibration as requested...");
    delay(2000); //wait for user to see serial
    touchscreen.calibrate();
    touchscreen.saveCalibration();
}

void enableTouchscreen(){
    // Initialize SPIFFS for calibration data
    initSPIFFS();

    // Initialize the touchscreen
    touchscreen.begin();

    // Check for existing calibration data
    if ((!touchscreen.loadCalibration()) || (RERUN_CALIBRATE)) {
        Serial.println("Re-calibrating. Either RERUN_CALIBRATE is set or no calibration data found.");
        calibrateTouchscreen();
    }
}

void turnOnBacklight(){
    pinMode(21, OUTPUT); // Set pin 21 as an output for the backlight
    digitalWrite(21, HIGH); // Turn on the backlight
}

void configureLVGL(){    
    //LVGL - graphics library that interfaces with TFT_eSPI
    Serial.println("LVGL initializing.");
    lv_init();

     //LVGL configuration
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LV_HOR_RES_MAX * 10);
    lv_disp_drv_init(&disp_drv); // Basic initialization
    disp_drv.draw_buf = &draw_buf; // Assign the initialized buffer
    disp_drv.flush_cb = cyd_disp_flush; // Set your display's flush callback
    disp_drv.hor_res = 240;  // Set the horizontal resolution of your display
    disp_drv.ver_res = 320;  // Set the vertical resolution of your display
    lv_disp_drv_register(&disp_drv); // Register the driver

}

void SDCardInit(){
    //SD Card
    if(!SD.begin(SD_CS)) {
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }
    Serial.println("SD card initialized.");
}

void removeSDCardFile(const char* path){
    if(SD.exists(path)) {
        SD.remove(path);
        Serial.println("Removed existing file: " + String(path));
    }
}

void createNewSDCardFile(const char* path){
    File file = SD.open(path, FILE_WRITE);
    if(!file) {
        Serial.println("Failed to create file");
        return;
    }
    file.close();
    Serial.println("Created new file: " + String(path));
}

void writeToSDCardFile(String data){
    Serial.println("Written 'Hello, hi from SD card!' to /hello.txt");
    File file = SD.open("/hello.txt", FILE_WRITE);
    if(!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    file.println("Hello, fellow CYD user!");
    file.close();
}

String readLineFromSDCardFile(const char* path){
    File file = SD.open(path);
    String thereadline;
    if(!file) {
        Serial.println("Failed to open file for reading");
        return thereadline;
    }
    Serial.println("Reading from file: " + String(path));
    while(file.available()) {
        thereadline = file.readStringUntil('\n');
        Serial.println(thereadline);
    }
    file.close();
    Serial.println("First line read from file: " + thereadline);
    return thereadline;
}

void pushTextToScreen(String text) {
    if (!atextlabel) {
        atextlabel = lv_label_create(lv_scr_act());
        lv_obj_align(atextlabel, LV_ALIGN_CENTER, 0, 0);
    }
    Serial.println("Setting text on label object.");
    lv_label_set_text(atextlabel, text.c_str());
    Serial.println("Displayed text on screen.");
}


void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial port to connect
    delay(2000);

    //Backlight
    Serial.println("Turning on backlight...");
    turnOnBacklight();

    //Touchscreen XPT2046 Bitbang
    Serial.println("Enabling touchscreen...");
    enableTouchscreen();

    //TFT_eSPI as display driver
    Serial.println("TFT init and rotation.");
    tft.begin(); /* TFT init */
    tft.setRotation(0); /* Landscape orientation */
    tft.fillScreen(TFT_BLACK); // Clear the screen to black
    // Draw a red circle
    tft.drawCircle(120, 160, 50, TFT_RED); // Parameters: x, y, radius, color
    delay(2000);

    //Config LVGL
    Serial.println("Configuring LVGL...");
    configureLVGL();

    //SDCard
    Serial.println("SD card initization.");
    SDCardInit();
    // Remove existing file if any
    Serial.println("Removing existing file /hello.txt if any.");
    removeSDCardFile("/hello.txt");
    // Create a new file
    createNewSDCardFile("/hello.txt");
    // Write to the file
    writeToSDCardFile("Hello, fellow CYD user!");
    // Read from the file
    printthis = readLineFromSDCardFile("/hello.txt");
    Serial.println("Line read from file: " + printthis);

    // Push some text to the screen as a test
    Serial.println("Pushing text to screen.");
    pushTextToScreen("Hello, CYD user!");

}


void loop() { 
    Serial.println("Looping..." + String(counter));
    
    //printthis = "none";
    //printthis = readLineFromSDCardFile("/hello.txt"); //this reads from SD, high wear for a loop
    //Serial.println("Line read from file: " + printthis);
    Serial.println("Pushing text to screen.");
    pushTextToScreen("Hello, CYD user!");

    Point touchPoint = touchscreen.getTouch(); // Poll the touch controller
    if (touchPoint.x != 0) { // No touch should return 0,0
        Serial.println("touchscreen location: " + String(touchPoint.x) + ", " + String(touchPoint.y));
    }

    lv_task_handler(); /* Handle LVGL tasks */

    Serial.print("Available heap memory: " + String(esp_get_free_heap_size()) + " bytes");

    delay(500);
    counter += 1;
}