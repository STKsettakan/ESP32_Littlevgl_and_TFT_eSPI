#include "TFT_eSPI.h"
#include "esp32lvgl.h"
TFT_eSPI tft = TFT_eSPI();
void setup()
{
  Serial.begin(115200);
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_WHITE);
  lvglInit(&tft);
  showCalibrateTouch(3);
  readTouchCalibarte();
  static lv_style_t style_screen;
  lv_style_copy(&style_screen, &lv_style_plain);
  style_screen.body.main_color = LV_COLOR_GRAY;
  style_screen.body.grad_color = LV_COLOR_GRAY;
  lv_obj_set_style(lv_scr_act(), &style_screen);
  lv_task_handler();

  lv_obj_t * label2 = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_long_mode(label2, LV_LABEL_LONG_SROLL_CIRC);     /*Circular scroll*/
  lv_obj_set_width(label2, 150);
  lv_label_set_text(label2, "It is a circularly scrolling text. ");
  lv_obj_align(label2, NULL, LV_ALIGN_CENTER, 0, 0);



}

void loop()
{

  lv_task_handler();
}
