#include <Arduino.h>

#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#include "sensors/onewire_temperature.h"
#include "signalk/signalk_output.h"

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 4

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


  sensesp_app->enable();
});
