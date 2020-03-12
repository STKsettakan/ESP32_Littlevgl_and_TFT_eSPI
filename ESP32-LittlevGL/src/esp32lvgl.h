#ifndef ESP32_LVGL_h
#define ESP32_LVGL_h

#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "Arduino.h"
#include "lvgl/lvgl.h"
#include "lv_conf.h"





void lvglInit(TFT_eSPI *tft);
bool readTouchCalibarte(void);
void calibrateTouch();
void showCalibrateTouch(uint8_t timeout_sec=0);

void StartLvglHandle();
void StopLvglHandle();


#endif



















