#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include <SD.h>
#include "SPI.h"
#include "XPT2046_Bitbang.h"
#include <Arduino.h>

//TF Card
#define SD_CS 5 // Adjust to your SD card CS pin

//TFT_eSPI used as LVGL display driver
TFT_eSPI tft = TFT_eSPI();

//LVGL (not all of this has to be global, but it's easier for now)
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10]; // /* Declare a buffer for 10 lines */
static lv_disp_drv_t disp_drv;  // Display driver
static lv_obj_t* circle = nullptr; //Circle object here so we dont stack overflow
static lv_obj_t *atextlabel = nullptr;

//Our globals. dont use global variables like this. stop it, get some help.
int counter = 0;
String printthis = "";

//Touchscreen
#define MOSI_PIN 32
#define MISO_PIN 39
#define CLK_PIN  25
#define CS_PIN   33
#define RERUN_CALIBRATE false //turn this on and watch serial at boot if you get touch errors
XPT2046_Bitbang touchscreen(MOSI_PIN, MISO_PIN, CLK_PIN, CS_PIN);
void calibrateTouchscreen();

// led and light sensor
#define LDR_PIN 34 // Photoresistor on GPIO34
#define LIGHT_SENSOR_RESOLUTION 12 

// LED pins
#define RED_LED_PIN 4
#define GREEN_LED_PIN 16
#define BLUE_LED_PIN 17
// PWM channels
#define RED_CHANNEL 0
#define GREEN_CHANNEL 1
#define BLUE_CHANNEL 2


// LVGL
static void cyd_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
static void cyd_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data);
#define LVGL_TICK_PERIOD_MS 1


#define SPEAKER_PIN 26  // ESP32's pin 26 corresponds to GPIO26 on ESP32 DevKitC V4 board 
#define FREQUENCY 200  // Low frequency (less than 30kHz) 
#define RESOLUTION 8    // 8-bit resolution (0-255) 
#define DUTY_CYCLE 1  // low duty cycle
#define BEEP_DURATION 100  // 250ms beep duration 
#define BEEP_COUNT 3  // 3 beeps 

// Flush display - this is the link between LVGL and TFT_eSPI for LVGL v8
static void cyd_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1); //calculate width and height of the area to be flushed
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h); //set the address window to the area to be flushed
    tft.pushColors((uint16_t*)color_p, w * h, true); //cast to 16 bit color for LVGL
    tft.endWrite();
    lv_disp_flush_ready(disp_drv); // Inform LVGL that flushing is done
    Serial.println("Display flushed.");
}

// Read touchpad - this is the link between LVGL and the XPT2046 driver for LVGL v8
static void cyd_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data) {
    Point touchPoint = touchscreen.getTouch(); // Poll the touch controller
    int16_t tmp_x = touchPoint.x;
    int16_t tmp_y = touchPoint.y;
    touchPoint.x = tmp_y;  // this x y was swapped for my display
    touchPoint.y = tmp_x;   //x y swapped, okay whatever
    Serial.println("Touchscreen location: " + String(touchPoint.x) + ", " + String(touchPoint.y));
    if (touchPoint.x > 0 && touchPoint.x < 240 && touchPoint.y > 0 && touchPoint.y < 320) {  
        // if the touch cnoordinates are within the display's coordinates
        data->state = LV_INDEV_STATE_PR; //update LVGL state to pressed
        data->point.x = touchPoint.x; //update LVGL point to the touch coordinates
        data->point.y = touchPoint.y;
        Serial.println("Valid touch point");
    } else {
        data->state = LV_INDEV_STATE_REL;  // if the touch screen is not touched (i.e., the touch coordinates are outside the display's coordinates)
        // Serial.println("Invalid touch point");
    }
    return; 
}

void initTouchscreen(){
    touchscreen.begin();
    // Check for existing calibration data
    if (!touchscreen.loadCalibration()) {
        Serial.println("Failed to load calibration data from SPIFFS.");
        calibrateTouchscreen();
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
    Serial.println("Calibration go! Get ready to touch the screen.");
    delay(2000); //wait for user to see serial
    touchscreen.calibrate();
    touchscreen.saveCalibration();
}

void enableTouchscreen(){
    initSPIFFS(); // Initialize SPIFFS for calibration data
    touchscreen.begin(); // Initialize the touchscreen
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
    //LVGL - high level graphics library that interfaces with TFT_eSPI driver
    Serial.println("LVGL initializing.");
    lv_init();

     //LVGL display configuration
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LV_HOR_RES_MAX * 10); // Initialize the display buffer
    lv_disp_drv_init(&disp_drv); // Basic initialization
    disp_drv.draw_buf = &draw_buf; // Assign the initialized buffer
    disp_drv.flush_cb = cyd_disp_flush; // Set your display's flush callback
    //disp_drv.rotated = LV_DISP_ROT_270;
    disp_drv.hor_res = 320;  // Set the horizontal resolution of your display
    disp_drv.ver_res = 240;  // Set the vertical resolution of your display
    lv_disp_drv_register(&disp_drv); // Register the driver

    // Initialize the touchpad driver with LVGL
    static lv_indev_drv_t indev_drv; //STATIC STATIC STATIC Descriptor of a touchpad/input device driver
    lv_indev_drv_init(&indev_drv); //Basic initialization
    indev_drv.type = LV_INDEV_TYPE_POINTER; // The touchpad is of type POINTER
    indev_drv.read_cb = cyd_touchpad_read; // Set your driver function
    lv_indev_drv_register(&indev_drv); // Finally register the touchpad driver
    Serial.println("Touch driver initialized and registered.");
}

void SDCardInit(){
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

int readLightLevel() {
    int lightLevel = analogRead(LDR_PIN);  // Read value from LIGHT_SENSOR_PIN 
    Serial.println("Light level: " + String(lightLevel));  // Print light level
    return lightLevel;  // Return light level 
}

#define MAX_PWM 255 // Maximum PWM value for 8-bit resolution
void toggleLEDs() {
    if (ledcRead(RED_CHANNEL) > 0) { // toggle the LED
        ledcWrite(RED_CHANNEL, 0); // Turn off the LED
    } else {
        ledcWrite(RED_CHANNEL, MAX_PWM); // Turn on the LED
    }
    if (ledcRead(GREEN_CHANNEL) > 0) {
        ledcWrite(GREEN_CHANNEL, 0); // Turn off the LED
    } else {
        ledcWrite(GREEN_CHANNEL, MAX_PWM); // Turn on the LED
    }
    if (ledcRead(BLUE_CHANNEL) > 0) {
        ledcWrite(BLUE_CHANNEL, 0); // Turn off the LED
    } else {
        ledcWrite(BLUE_CHANNEL, MAX_PWM); // Turn on the LED
    }
}
static void button_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("Button clicked!");
        // Call any function here
        toggleLEDs(); // Set the LED color to green (50% brightness
    }
}
void create_button(String buttontext) { //LED button
    lv_obj_t * btn = lv_btn_create(lv_scr_act()); // Create a button on the active screen
    lv_obj_set_size(btn, 200, 80); // Set the button's size
    lv_obj_set_align(btn, LV_ALIGN_TOP_MID);
    lv_obj_t * label = lv_label_create(btn); // Add a label to the button
    lv_label_set_text(label, buttontext.c_str()); // Set the label text
    lv_obj_center(label); // Center the label on the button
    lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_CLICKED, NULL); // Assign the event callback
}

void buzzSpeaker() {
    // Beep 3 times 
    for(int i = 0; i < BEEP_COUNT; i++) {  // Loop 3 times 
        ledcWrite(4, DUTY_CYCLE);  // Turn on speaker 
        delay(BEEP_DURATION);  // Wait 250ms
        ledcWrite(4, 0);  // Turn off speaker 
        delay(BEEP_DURATION);  // Wait 250ms 
    } 
}
static void button_event_cb2(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("Buzz Button clicked!");
        buzzSpeaker(); // buzzy buzzy
    }
}
void create_button2(String buttontext) { //Buzzer button
    lv_obj_t * btn = lv_btn_create(lv_scr_act()); // Create a button on the active screen
    lv_obj_set_size(btn, 200, 80); // Set the button's size
    lv_obj_set_align(btn, LV_ALIGN_BOTTOM_MID);
    lv_obj_t * label = lv_label_create(btn); // Add a label to the button
    lv_label_set_text(label, buttontext.c_str()); // Set the label text
    lv_obj_center(label); // Center the label on the button
    lv_obj_add_event_cb(btn, button_event_cb2, LV_EVENT_CLICKED, NULL); // Assign the event callback
}

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial (might cause issues when powered from battery)
    delay(1000);

    //Backlight
    Serial.println("Turning on backlight...");
    turnOnBacklight();

    // Initialize LED pins as output
    ledcSetup(RED_CHANNEL, 5000, 8); // 5 kHz PWM, 8-bit resolution
    ledcSetup(GREEN_CHANNEL, 5000, 8);
    ledcSetup(BLUE_CHANNEL, 5000, 8);

    // Attach the channel to the GPIO to be controlled
    ledcAttachPin(RED_LED_PIN, RED_CHANNEL);
    ledcAttachPin(GREEN_LED_PIN, GREEN_CHANNEL);
    ledcAttachPin(BLUE_LED_PIN, BLUE_CHANNEL);

    // Speaker
    pinMode(SPEAKER_PIN, OUTPUT);  // Set SPEAKER_PIN as output 
    // Configure PWM settings 
    ledcSetup(4, FREQUENCY, RESOLUTION);  // Setup PWM channel 0 
    // Attach SPEAKER_PIN to PWM channel 0 
    ledcAttachPin(SPEAKER_PIN, 4);  

    //LDR light sensor
    pinMode(LDR_PIN, INPUT);
    analogReadResolution(LIGHT_SENSOR_RESOLUTION);
    analogSetPinAttenuation(LDR_PIN, ADC_0db); 

    //Touchscreen XPT2046 Bitbang
    Serial.println("Enabling touchscreen...");
    enableTouchscreen();

    //TFT_eSPI as display driver
    Serial.println("TFT init and rotation.");
    tft.begin(); /* TFT init */
    tft.setRotation(1); //You may need to adjust this for your display
    tft.invertDisplay(0); //I had to invert colors: st7789 is 0, ili9341 is 1
    //Test TFT_eSPI with RED circle so you know it's working
    tft.fillScreen(TFT_BLACK); // Clear the screen to black
    Serial.println("Drawing a red circle using TFT_eSPI only.");
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
    createNewSDCardFile("/hello.txt");
    writeToSDCardFile("Hello, fellow CYD user, your SD works!");
    printthis = readLineFromSDCardFile("/hello.txt"); // Read from the file
    Serial.println("Line read from file: " + printthis);

    // Push some text to the screen as a test
    Serial.println("Pushing text to screen.");

    //if printthis is empty, push a default message
    if (printthis == "") {
        pushTextToScreen("Hello, fellow CYD user! SD didn't work :(");
    } else {
        pushTextToScreen(printthis);
    }

    // Create a button
    create_button("LED");
    create_button2("Buzzer");
}

void loop() { 
    static uint32_t prev_ms = millis(); // part of the LVGL timer workaround

    Serial.println("Looping..." + String(counter));
    
    //this wasn't working for me on multiple devices
    int lightLevel = readLightLevel(); //light sensor on pin 34

    //LVGL needs to be updated in the loop
    lv_task_handler(); // Throw LGVL a bone (this line and comment was written by copilot, I swear)
    lv_timer_handler(); // Throw LGVL another bone (again, this line and comment was written by copilot, I swear)

    delay(50);
    counter += 1;

    //Thanks to https://github.com/AllanOricil/esp32-lvgl-lcd-touch-sd-card 
    //this is a workaround to force LVGL to understand time
    uint32_t elapsed_ms = millis() - prev_ms;
    if(elapsed_ms >= LVGL_TICK_PERIOD_MS) {
        lv_tick_inc(elapsed_ms); //increment LVGL tick manually
        prev_ms += elapsed_ms;
    }
}