//***************************************************************************
// Base project: https://bogza.ro/index.php/Temperature_relay_device_T19TSRAWV01
// Github: https://github.com/tinel-c/HomieTest
//***************************************************************************
// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Homie.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 5     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

// set up the time to send the distance
const int DEFAULT_DISTANCE_INTERVAL = 10;
//construct a global variable to handle the distance
unsigned long lastTemperatureSent = 0;

// relay pin
const byte outRelayPin = 4;

// data sent
float getHumidity;
float getTemperature;
unsigned long systemTime = -1; // get the system time

//calculated delay for the DHT sensor
int delayMS;


HomieNode ComplexSensorNode("T19TSRAWV01", "T19TSRAWV01","string");
HomieSetting<long> temperatureIntervalSetting("temperatureInterval", "The temperature interval in seconds"); 

bool globalInputHandler(const HomieNode& node, const HomieRange& range, const String& property, const String& value) {
  char nodeName[] = "T19TSRAWV01";
  String inputName = "on";
  String activationValue = "true";
  Homie.getLogger() << "Received on node " << node.getId() << ": " << property << " = " << value << endl;
  if(strcmp(nodeName,node.getId()) == 0) {
    Homie.getLogger() << "Node identified " << endl;
   if(property.equals(inputName)) { 
     Homie.getLogger() << "value " << inputName << " triggered" << endl;
     if(value.equals(activationValue)) { 
        Homie.getLogger() << "activationValue " << activationValue << " triggered output high on pin: "<<  outRelayPin << endl;
        digitalWrite(outRelayPin, HIGH); // sets the digital pin on
      }
      else
      {
        Homie.getLogger() << "activationValue " << activationValue << " triggered output low on pin: "<<  outRelayPin << endl;
        digitalWrite(outRelayPin, LOW); // sets the digital pin off
      }
      
    }
  }
  return true;
}

void setupHandler() {
  
}


void loopHandler() {
  if (millis() - lastTemperatureSent >= temperatureIntervalSetting.get() * 1000UL || lastTemperatureSent == 0) {
    systemTime = millis();
 // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Homie.getLogger() << "Error reading temperature!" << endl;
  }
  else {
    Homie.getLogger() << "Temperature: " << event.temperature << "°C" << endl;
    getTemperature = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Homie.getLogger() << "Error reading humidity!" << endl;
  }
  else {
    Homie.getLogger() << "Humidity: " << event.relative_humidity << "%" << endl;
    getHumidity = event.relative_humidity;
  }


    Homie.getLogger() << "Temperature: " << getTemperature << endl;
    Homie.getLogger() << "Humidity: " << getHumidity << endl;
    Homie.getLogger() << "Time since the system started: " << systemTime << endl; //prints time since program started

    // send data to Homie values on mqtt
    ComplexSensorNode.setProperty("temperatureRead").send(String(getTemperature));
    ComplexSensorNode.setProperty("humidityRead").send(String(getHumidity));
    ComplexSensorNode.setProperty("sensorUptime").send(String(systemTime / 1000));
    lastTemperatureSent = millis();
  }
}


void setup() {
  // start the serial communication
  Serial.begin(115200);
  Serial << endl << endl;

  // set the name of the device family
  Homie_setBrand("T19TSRAWV01");
  //set the name and version of the sensor
  Homie_setFirmware("Temperature_Relay_Software", "1.0.0");
  // launch the global input handler
  Homie.setGlobalInputHandler(globalInputHandler);
  // start advertise
  ComplexSensorNode.advertise("on").setName("On").setDatatype("boolean").settable();
  ComplexSensorNode.advertise("temperatureRead").setName("TemperatureRead").setDatatype("string");
  ComplexSensorNode.advertise("humidityRead").setName("HumidityRead").setDatatype("string");
  ComplexSensorNode.advertise("sensorUptime").setName("SensorUptime").setDatatype("integer").setUnit("s");

  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);

  temperatureIntervalSetting.setDefaultValue(DEFAULT_DISTANCE_INTERVAL).setValidator([] (long candidate) {
    return candidate > 0;
  });

  // Initialize dht device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  // relay pin setup
  Homie.getLogger() << " output low on pin: "<<  outRelayPin << endl;
  pinMode(outRelayPin, OUTPUT);           // set pin to input
  digitalWrite(outRelayPin, LOW); // sets the digital pin off
  // stup homie
  Homie.setup();
}

void loop() {
  Homie.loop();
  if((systemTime < 0) || (millis() - systemTime >=2000))
  {
    // once every 2000 ms do nothing
  }
}