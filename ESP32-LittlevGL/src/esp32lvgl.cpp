#include "esp32lvgl.h"
#include <Ticker.h>

static TFT_eSPI *_tft;
static lv_disp_buf_t disp_buf;
//static lv_color_t buf[LV_HOR_RES_MAX * 10];
lv_color_t *buf;
#define LVGL_TICK_PERIOD 20
Ticker tick; /* timer for interrupt handler */
static void lv_tick_handler(void);

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
bool read_encoder(lv_indev_drv_t * indev, lv_indev_data_t * data);
void lv_handler(void *);
#define CALIBRATION_FILE "/calibrationData"
uint16_t calibrationData[5];
TaskHandle_t LvglTask = NULL;

void lvglInit(TFT_eSPI *tft)
{
	buf = (lv_color_t*)ps_calloc(LV_HOR_RES_MAX * 10, sizeof(lv_color_t));
	_tft = tft;
	_tft->setSwapBytes(true);
	lv_init();
	lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);
	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = _tft->width();
	disp_drv.ver_res = _tft->height();
	disp_drv.flush_cb = my_disp_flush;
	disp_drv.buffer = &disp_buf;
	lv_disp_drv_register(&disp_drv);
	
	lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = read_encoder;
	lv_indev_drv_register(&indev_drv);

	if (!SPIFFS.begin())
	{
		Serial.println("formating file system");
		SPIFFS.format();
		SPIFFS.begin();
	}
	
	uint8_t calDataOK = 0;
	
	
 /*if (!readTouchCalibarte()) 
 {
   calibrateTouch();
 }
 */
	tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);
}
void showCalibrateTouch(uint8_t timeout_sec)
{
	_tft->fillScreen(TFT_BLACK);
	_tft->setTextColor(TFT_WHITE,TFT_BLACK);
	_tft->drawCentreString("Press Screen to Recalibrate", 240,10,2);
	uint8_t cnt_dwn = timeout_sec;
	unsigned long pv_calibrate = millis();
	unsigned long pv_cntdwn = pv_calibrate; 
	_tft->drawCentreString(String(cnt_dwn), 240,160,2);
	do
	{
		 uint16_t x, y;
		if (_tft->getTouch(&x, &y)) 
		{
			while(_tft->getTouch(&x, &y)){}
			Serial.println("Recalibrate...");
			calibrateTouch();
			return;
		}
		if(millis()-pv_cntdwn>1000)
		{
			cnt_dwn--;
			_tft->drawCentreString("     ", 240,160,2);
			_tft->drawCentreString(String(cnt_dwn), 240,160,2);
			pv_cntdwn = millis();
		}
	}
	while((millis()-pv_calibrate)<(timeout_sec*1000));
	Serial.println("Calibrate Time Out");
}
void calibrateTouch()
{
	_tft->calibrateTouch(calibrationData, TFT_WHITE, TFT_RED, 15);
	if (SPIFFS.exists(CALIBRATION_FILE))
	{
		SPIFFS.remove(CALIBRATION_FILE);
	}
	File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) 
	{
		f.write((const unsigned char *)calibrationData, 14);
		f.close();
    }
}
bool readTouchCalibarte(void)
{
	bool ret = false; 
	if (SPIFFS.exists(CALIBRATION_FILE)) {
    File f = SPIFFS.open(CALIBRATION_FILE, "r");
    if (f) 
	{
      if (f.readBytes((char *)calibrationData, 14) == 14)
			ret =  true;
      f.close();
	  Serial.println("touch set cal OK");
	  _tft->setTouch(calibrationData);
	
    }
  }
  return(ret);
}
void StartLvglHandle()
{
	 xTaskCreate( lv_handler,"lv_handler",10000,NULL,1,&LvglTask);      
}
void StopLvglHandle()
{
	vTaskDelete( LvglTask );
}
void lv_handler(void *)
{
	for(;;)
	{
		lv_task_handler(); /* let the GUI do its work */
		delay(5);
	}
	vTaskDelete( NULL );
}
static void lv_tick_handler(void)
{

  lv_tick_inc(LVGL_TICK_PERIOD);
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
	//_tft->setSwapBytes(true);
	_tft->pushImage(area->x1, area->y1,area->x2 - area->x1 + 1,area->y2 - area->y1 + 1,(uint16_t*) color_p);
	lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

bool read_encoder(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
 uint16_t x, y;
	static uint16_t prev_x, prev_y;
	if (_tft->getTouch(&x, &y)) 
	{
		data->point.x = x;
		data->point.y = y;
		data->state = LV_INDEV_STATE_PR;
		prev_x = data->point.x;
		prev_y = data->point.y;
		//Serial.printf("x:%d,Y:%d\r\n",x,y);
	}
	else 
	{
		data->point.x = prev_x;
		data->point.y = prev_y;
		data->state = LV_INDEV_STATE_REL;  
	}

  return false;
}

