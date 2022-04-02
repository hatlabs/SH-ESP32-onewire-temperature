#include <Adafruit_GFX.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>
#include "eh_analog.h"
#include "rpm.h"
#include "eh_display.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"

using namespace sensesp;

// I2C pins on SH-ESP32
const int kSDAPin = 16;
const int kSCLPin = 17;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// Engine hat digital input pins
const int kDigitalInputPin1 = GPIO_NUM_15;
const int kDigitalInputPin2 = GPIO_NUM_13;
const int kDigitalInputPin3 = GPIO_NUM_14;
const int kDigitalInputPin4 = GPIO_NUM_12;

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 4

// SDA and SCL pins on SH-ESP32
#define SDA_PIN 16
#define SCL_PIN 17

// OLED display width and height, in pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// define temperature display units
//#define TEMP_DISPLAY_FUNC KelvinToCelsius
#define TEMP_DISPLAY_FUNC KelvinToFahrenheit

// Test output pin configuration
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_18;
// repetition interval in ms; corresponds to 1000/(2*5)=100 Hz
const int kTestOutputInterval = 5;
#endif

TwoWire* i2c;
Adafruit_SSD1306* display;

/// Clear a text row on an Adafruit graphics display
void ClearRow(int row) { display->fillRect(0, 8 * row, SCREEN_WIDTH, 8, 0); }

float KelvinToCelsius(float temp) { return temp - 273.15; }

float KelvinToFahrenheit(float temp) { return (temp - 273.15) * 9. / 5. + 32.; }

void PrintTemperature(int row, String title, float temperature) {
  ClearRow(row);
  display->setCursor(0, 8 * row);
  display->printf("%s: %.1f", title.c_str(), TEMP_DISPLAY_FUNC(temperature));
  display->display();
}

ReactESP app;

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  SensESPAppBuilder builder;

  sensesp_app = builder.set_hostname("SH-ESP32-Engine")
                       ->enable_uptime_sensor()
                       ->enable_system_info_sensors()
                       ->get_app();

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // Connect the tacho senders
  
  auto tacho_1_frequency = ConnectTachoSender(kDigitalInputPin1, "1");

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths

  auto main_engine_oil_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineOilTemp/oneWire");
  auto main_engine_coolant_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineCoolantTemp/oneWire");
  auto main_engine_exhaust_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineWetExhaustTemp/oneWire");

  // define metadata for sensors

  auto main_engine_oil_temperature_metadata =
      new SKMetadata("K",                       // units
                     "Engine Oil Temperature",  // display name
                     "Engine Oil Temperature",  // description
                     "Oil Temperature",         // short name
                     10.                        // timeout, in seconds
      );
  auto main_engine_coolant_temperature_metadata =
      new SKMetadata("K",                           // units
                     "Engine Coolant Temperature",  // display name
                     "Engine Coolant Temperature",  // description
                     "Coolant Temperature",         // short name
                     10.                            // timeout, in seconds
      );
  auto main_engine_temperature_metadata =
      new SKMetadata("K",                   // units
                     "Engine Temperature",  // display name
                     "Engine Temperature",  // description
                     "Temperature",         // short name
                     10.                    // timeout, in seconds
      );
  auto main_engine_exhaust_temperature_metadata =
      new SKMetadata("K",                        // units
                     "Wet Exhaust Temperature",  // display name
                     "Wet Exhaust Temperature",  // description
                     "Exhaust Temperature",      // short name
                     10.                         // timeout, in seconds
      );

  auto tacho_1_frequency_metadata =
      new SKMetadata("K",                       // units
                     "Engine RPM",  // display name
                     "Engine RPM",  // description
                     "RPM",         // short name
                     10.                        // timeout, in seconds
      );   

  // connect the sensors to Signal K output paths

  main_engine_oil_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.oilTemperature", "/mainEngineOilTemp/skPath",
      main_engine_oil_temperature_metadata));
  main_engine_coolant_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.coolantTemperature", "/mainEngineCoolantTemp/skPath",
      main_engine_coolant_temperature_metadata));
  // transmit coolant temperature as overall engine temperature as well
  main_engine_coolant_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.temperature", "/mainEngineTemp/skPath",
      main_engine_temperature_metadata));
  // propulsion.*.wetExhaustTemperature is a non-standard path
  main_engine_exhaust_temperature->connect_to(
      new SKOutput<float>("propulsion.main.wetExhaustTemperature",
                          "/mainEngineWetExhaustTemp/skPath",
                          main_engine_exhaust_temperature_metadata));

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
  app.onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString()); });
  main_engine_oil_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(2, "Oil", temperature); }));
  main_engine_coolant_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(3, "Coolant", temperature); }));
  main_engine_exhaust_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(4, "Exhaust", temperature); }));
  tacho_1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 5, "RPM", 60 * value); }));

  sensesp_app->start();
}

// main program loop
void loop() { app.tick(); }