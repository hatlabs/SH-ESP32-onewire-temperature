#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#include "sensors/onewire_temperature.h"
#include "signalk/signalk_output.h"

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 4

// SDA and SCL pins on SH-ESP32
#define SDA_PIN 16
#define SCL_PIN 17

// OLED display width and height, in pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// define temperature display units
#define TEMP_DISPLAY_FUNC KelvinToCelsius
//#define TEMP_DISPLAY_FUNC KelvinToFahrenheit

TwoWire* i2c;
Adafruit_SSD1306* display;

/// Clear a text row on an Adafruit graphics display
void ClearRow(int row) { display->fillRect(0, 8 * row, SCREEN_WIDTH, 8, 0); }

float KelvinToCelsius(float temp) {
  return temp - 273.15;
}

float KelvinToFahrenheit(float temp) {
  return (temp - 273.15) * 9./5. + 32.;
}

void PrintTemperature(int row, String title, float temperature) {
  ClearRow(row);
  display->setCursor(0, 8 * row);
  display->printf("%s: %.1f", title.c_str(), TEMP_DISPLAY_FUNC(temperature));
  display->display();
}

ReactESP app([]() {
// Some initialization boilerplate when in debug mode...
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  SensESPAppBuilder builder;

  sensesp_app = builder.set_hostname("temperatures")
                    ->set_wifi("Stairway to Heaven", "kanneluuri2406")
                    ->set_sk_server("hurma.lan", 80)
                    ->set_standard_sensors(NONE)
                    ->get_app();

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI paths

  auto main_engine_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineTemp/oneWire");
  auto main_engine_coolant_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineCoolantTemp/oneWire");
  auto main_engine_exhaust_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineExhaustTemp/oneWire");

  // connect the sensors to Signal K output paths

  main_engine_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.temperature", "/mainEngineTemp/skPath"));
  main_engine_coolant_temperature->connectTo(new SKOutput<float>(
      "propulsion.main.coolantTemperature", "/mainEngineCoolantTemp/skPath"));
  main_engine_exhaust_temperature->connectTo(new SKOutput<float>(
      "propulsion.main.exhaustTemperature", "/mainEngineExhaustTemp/skPath"));

  // initialize the display
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(100);
  display->setRotation(2);
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);  
  display->setCursor(0, 0);
  display->printf("Host: %s", sensesp_app->get_hostname().c_str());
  display->display();

  // Add display updaters for temperature values
  main_engine_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(1, "Engine", temperature); }));
  main_engine_coolant_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(2, "Coolant", temperature); }));
  main_engine_exhaust_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(3, "Exhaust", temperature); }));

  sensesp_app->enable();
});
