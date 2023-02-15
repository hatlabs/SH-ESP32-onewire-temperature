#ifndef __SRC_EH_DISPLAY_H__
#define __SRC_EH_DISPLAY_H__

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "sensesp_base_app.h"

using namespace sensesp;

bool InitializeSSD1306(ReactESP* app, SensESPBaseApp* sensesp_app,
                       Adafruit_SSD1306** display, TwoWire* i2c);

void ClearRow(Adafruit_SSD1306* display, int row);

void PrintValue(Adafruit_SSD1306* display, int row, String title, float value);
void PrintValue(Adafruit_SSD1306* display, int row, String title, String value);

#endif
