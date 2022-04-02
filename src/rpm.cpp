#include "rpm.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"

using namespace sensesp;


// Default RPM count scale factor (corresponds to Yanmar 3GM30F RPM sender
// output)
const float kDefaultFrequencyScale = 1. / 97;


FloatProducer* ConnectTachoSender(int pin, String name) {
  char config_path[80];
  char sk_path[80];

  snprintf(config_path, sizeof(config_path), "", name.c_str());
  auto tacho_input = new DigitalInputCounter(pin, INPUT, RISING, 500, config_path);

  snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolution Multiplier", name.c_str());
  auto tacho_frequency = new Frequency(kDefaultFrequencyScale, config_path);

  snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolutions SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.revolutions", name.c_str());
  auto tacho_frequency_sk_output = new SKOutputFloat(sk_path, config_path);

  tacho_input
    ->connect_to(tacho_frequency)
    ->connect_to(tacho_frequency_sk_output);

  // tacho_input->attach([name, tacho_input]() {
  //   debugD("Input %s counter: %d", name.c_str(), tacho_input->get());
  // });

  return tacho_frequency;
}

//BoolProducer* ConnectAlarmSender(int pin, String name) {
//  char config_path[80];
//  char sk_path[80];
//
//  auto* alarm_input = new DigitalInputChange(pin, INPUT, CHANGE);
//
//  snprintf(config_path, sizeof(config_path), "/Alarm %s/SK Path", name.c_str());
//  snprintf(sk_path, sizeof(sk_path), "alarm.%s", name.c_str());
//  auto alarm_sk_output = new SKOutputBool(sk_path, config_path);
//
//  alarm_input
//    ->connect_to(alarm_sk_output);
//
//  return alarm_input;
//}
