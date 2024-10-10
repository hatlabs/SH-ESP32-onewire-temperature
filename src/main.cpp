#include "sensesp.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <memory>

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 4

// SDA and SCL pins on SH-ESP32
#define SDA_PIN 16
#define SCL_PIN 17

// CAN bus (NMEA 2000) pins on SH-ESP32
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

// OLED display width and height, in pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// define temperature display units
#define TEMP_DISPLAY_FUNC KelvinToCelsius
// #define TEMP_DISPLAY_FUNC KelvinToFahrenheit

using namespace sensesp;
using namespace sensesp::onewire;

TwoWire* i2c;
Adafruit_SSD1306* display;

tNMEA2000* nmea2000;

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

double oil_temperature = N2kDoubleNA;
double coolant_temperature = N2kDoubleNA;

/**
 * @brief Send Engine Dynamic Parameter data
 *
 * Send engine temperature data using the Engine Dynamic Parameter PGN.
 * All unused fields that are sent with undefined value except the status
 * bit fields are sent as zero. Hopefully we're not resetting anybody's engine
 * warnings...
 */
void SendEngineTemperatures() {
  tN2kMsg N2kMsg;
  SetN2kEngineDynamicParam(N2kMsg,
                           0,  // instance of a single engine is always 0
                           N2kDoubleNA,  // oil pressure
                           oil_temperature, coolant_temperature,
                           N2kDoubleNA,  // alternator voltage
                           N2kDoubleNA,  // fuel rate
                           N2kDoubleNA,  // engine hours
                           N2kDoubleNA,  // engine coolant pressure
                           N2kDoubleNA,  // engine fuel pressure
                           N2kInt8NA,    // engine load
                           N2kInt8NA,    // engine torque
                           (tN2kEngineDiscreteStatus1)0,
                           (tN2kEngineDiscreteStatus2)0);
  nmea2000->SendMsg(N2kMsg);
}

void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  SensESPAppBuilder builder;

  sensesp_app = builder.set_hostname("temperatures")->get_app();

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths

  auto main_engine_oil_temperature = std::make_shared<OneWireTemperature>(
      dts, 1000, "/mainEngineOilTemp/oneWire");

  ConfigItem(main_engine_oil_temperature)
      ->set_title("Oil Temperature")
      ->set_description("Entine Oil Temperature")
      ->set_sort_order(100);

  auto main_engine_coolant_temperature = std::make_shared<OneWireTemperature>(
      dts, 1000, "/mainEngineCoolantTemp/oneWire");

  ConfigItem(main_engine_coolant_temperature)
      ->set_title("Coolant Temperature")
      ->set_description("Engine Coolant Temperature")
      ->set_sort_order(200);

  auto main_engine_exhaust_temperature = std::make_shared<OneWireTemperature>(
      dts, 1000, "/mainEngineWetExhaustTemp/oneWire");

  ConfigItem(main_engine_exhaust_temperature)
      ->set_title("Exhaust Temperature")
      ->set_description("Wet Exhaust Temperature")
      ->set_sort_order(300);

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

  // connect the sensors to Signal K output paths

  auto sk_output_oil_temp = std::make_shared<SKOutput<float>>(
      "propulsion.main.oilTemperature", "/mainEngineOilTemp/skPath",
      main_engine_oil_temperature_metadata);
  auto sk_output_coolant_temp = std::make_shared<SKOutput<float>>(
      "propulsion.main.coolantTemperature", "/mainEngineCoolantTemp/skPath",
      main_engine_coolant_temperature_metadata);
  auto sk_output_engine_temp = std::make_shared<SKOutput<float>>(
      "propulsion.main.temperature", "/mainEngineTemp/skPath",
      main_engine_temperature_metadata);
  auto sk_output_exhaust_temp = std::make_shared<SKOutput<float>>(
      "propulsion.main.wetExhaustTemperature",
      "/mainEngineWetExhaustTemp/skPath",
      main_engine_exhaust_temperature_metadata);

  main_engine_oil_temperature->connect_to(sk_output_oil_temp);
  main_engine_coolant_temperature->connect_to(sk_output_coolant_temp);
  // transmit coolant temperature as overall engine temperature as well
  main_engine_coolant_temperature->connect_to(sk_output_engine_temp);
  // propulsion.*.wetExhaustTemperature is a non-standard path
  main_engine_exhaust_temperature->connect_to(sk_output_exhaust_temp);

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
  auto oil_temp_display_updater = new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(1, "Oil", temperature); });
  auto coolant_temp_display_updater = new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(2, "Coolant", temperature); });
  auto exhaust_temp_display_updater = new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(3, "Exhaust", temperature); });

  main_engine_oil_temperature->connect_to(oil_temp_display_updater);
  main_engine_coolant_temperature->connect_to(coolant_temp_display_updater);
  main_engine_exhaust_temperature->connect_to(exhaust_temp_display_updater);

  // initialize the NMEA 2000 subsystem

  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20210405",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 Temp Sensor",  // Manufacturer's Model ID (max 33 chars)
      "0.2.0.0 (2024-10-10)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      75,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 22);
  // Disable all msg forwarding to USB (=Serial)
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // Implement the N2K PGN sending. Engine (oil) temperature and coolant
  // temperature are a bit more complex because they're sent together
  // as part of a Engine Dynamic Parameter PGN.

  auto oil_temp_consumer = new LambdaConsumer<float>([](float temperature) {
    oil_temperature = temperature;
    SendEngineTemperatures();
  });

  auto coolant_temp_consumer = new LambdaConsumer<float>([](float temperature) {
    coolant_temperature = temperature;
    SendEngineTemperatures();
  });

  auto exhaust_temp_consumer = new LambdaConsumer<float>([](float temperature) {
    tN2kMsg N2kMsg;
    SetN2kTemperature(N2kMsg,
                      1,                            // SID
                      2,                            // TempInstance
                      N2kts_ExhaustGasTemperature,  // TempSource
                      temperature                   // actual temperature
    );
    nmea2000->SendMsg(N2kMsg);
  });

  main_engine_oil_temperature->connect_to(oil_temp_consumer);
  main_engine_coolant_temperature->connect_to(coolant_temp_consumer);
  main_engine_exhaust_temperature->connect_to(exhaust_temp_consumer);

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

// main program loop
void loop() { event_loop()->tick(); }
