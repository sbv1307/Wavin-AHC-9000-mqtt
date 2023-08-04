#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "WavinController.h"
#include "PrivateConfig.h"

#define SKTECH_VERSION "Esp8266 MQTT interface - V0.0.8"

#define ALT_LED_BUILTIN 16


/*
 * ISSUES V0.0.6
 * Changing target temeratur:
 * After changing Target temperature in HA, the Target temperature in HA did not get updated.
 * Investigations indicated that the relative large number of channels being read, stressed the MOD-BUS. Adding delays between writing and reading 
 * WavinController registers seemed to solved the issue.
 *
 * OTA update:
 * The Esp8266 MQTT interface did not answer to requests.
 * It turned out, that reading all channels took longer than the defined POLL_TIME_MS = 5000, and since lastUpdateTime were updated at the beginning
 * of the iteration, the call to ArduinoOTA.handle() were only done once on each loop!
 * Move the update of lastUpdateTime to the end, gave at least a break in scanning the cannels, however it still only gave a short window for the OTA
 * process to establish contace.
 * 
 * Introducing af "delayForOAT" function, which creates a delay between writing and reading 
 * WavinController registers and calling ArduinoOTA.handle() during the delay, solved the issues of Changing target temeratur and  OTA update.
 * 
 * ISSUES V0.0.7
 * Changing target temeratur:
 * Target temperature will get updated, but it usually takes a long time!
 * The problem is, that iterating through all channels takes quite a while, which will cause the response to a change of temperature will take a while.
 * 
 * Instead of waiting for the iteration process to finally get the target temperature changed, call the process for readng the setpont dirctly from the 
 * MQTT callback function.
 * 
 */

// MQTT defines
// Esp8266 MAC will be added to the device name, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/1/3/target', where 1 is the Modbus Device number and 3 is the output id and XXXXXXXXXXXX is the mac

const String   MQTT_PREFIX              = "heat/";       // include tailing '/' in prefix
const String   MQTT_DEVICE_NAME         = "floor";       // only alfanumeric and no '/'
const String   MQTT_ONLINE              = "/online";      
const String   MQTT_SUFFIX_CURRENT      = "/current";    // include heading '/' in all suffixes
const String   MQTT_SUFFIX_SETPOINT_GET = "/target";
const String   MQTT_SUFFIX_SETPOINT_SET = "/target_set";
const String   MQTT_SUFFIX_MODE_GET     = "/mode";
const String   MQTT_SUFFIX_MODE_SET     = "/mode_set";
const String   MQTT_SUFFIX_BATTERY      = "/battery";
const String   MQTT_SUFFIX_OUTPUT       = "/output";

const String   MQTT_VALUE_MODE_STANDBY  = "off";
const String   MQTT_VALUE_MODE_MANUAL   = "heat";

const String   MQTT_CLIENT = "Wavin-AHC-9000-mqtt";       // mqtt client_id prefix. Will be suffixed with Esp8266 mac to make it unique

String mqttDeviceNameWithMac;
String mqttClientWithMac;

// Operating mode is controlled by the MQTT_SUFFIX_MODE_ topic.
// When mode is set to MQTT_VALUE_MODE_MANUAL, temperature is set to the value of MQTT_SUFFIX_SETPOINT_
// When mode is set to MQTT_VALUE_MODE_STANDBY, the following temperature will be used
const float STANDBY_TEMPERATURE_DEG = 5.0;

const uint8_t TX_ENABLE_PIN = 5;
const bool SWAP_SERIAL_PINS = true;
const uint16_t RECIEVE_TIMEOUT_MS = 1000;
WavinController wavinController(TX_ENABLE_PIN, SWAP_SERIAL_PINS, RECIEVE_TIMEOUT_MS);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastUpdateTime = 0;

const uint16_t POLL_TIME_MS = 5000;

// Create a structure for each channel on all heat controllers controlled by this project.
struct lastKnownValue_t {
  uint16_t temperature;
  uint16_t setpoint;
  uint16_t battery;
  uint16_t status;
  uint16_t mode;
} lastSentValues[WavinController::NUMBER_OF_CHANNELS * WavinController::NUMBER_OF_DEVICES];

const uint16_t LAST_VALUE_UNKNOWN = 0xFFFF;

bool configurationPublished[WavinController::NUMBER_OF_CHANNELS * WavinController::NUMBER_OF_DEVICES];

/* Delayfunction, which call ArduinoOTA.handle() 'iterations' number of times to create a delay, 
 * and at the same time opens for OTA updates during the "delay"
 */
void delayForOAT(uint16_t iterations)
{
  for ( uint16_t ii = 0; ii < iterations ; ii++)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      // Check for over the air update request and (if present) flash it
      ArduinoOTA.handle();
    }
  }
}

// Read a float value from a non zero terminated array of bytes and
// return 10 times the value as an integer
uint16_t temperatureFromString(String payload)
{
  float targetf = payload.toFloat();
  return (unsigned short)(targetf * 10);
}


// Returns temperature in degrees with one decimal
String temperatureAsFloatString(uint16_t temperature)
{
  float temperatureAsFloat = ((float)temperature) / 10;
  return String(temperatureAsFloat, 1);
}


uint8_t getIdFromTopic(char* topic)
{
  unsigned int startIndex = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/").length();
  int i = 0;
  uint8_t result = 0;

  // The topic now includes a number, defining the heat controller / Device. This number will now be found at topic[startIndex].
  // To filter out Modbus Device number - the easy (and dirty way): device number will be one character. Plus the seperator (/) makes it two chars.
  // So - Increasing startIndex by 2, will point at the ID we are looking for.
  startIndex += 2;

  while(topic[startIndex + i] != '/' && i<3)
  {
    result = result * 10 + (topic[startIndex+i]-'0');
    i++;
  }

  return result;
}

uint8_t getModbusDevideFromTopic(char* topic)
{
  unsigned int startIndex = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/").length();
  int i = 0;
  uint8_t result = 0;

  while(topic[startIndex+i] != '/' && i<3)
  {
    result = result * 10 + (topic[startIndex+i]-'0');
    i++;
  }

  return result;
}

void publishIfNewValue(String topic, String payload, uint16_t newValue, uint16_t *lastSentValue)
{
  if (newValue != *lastSentValue)
  {
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true))
    {
        *lastSentValue = newValue;
    }
    else
    {
      *lastSentValue = LAST_VALUE_UNKNOWN;
    }
  }
}

void readSetpoint( uint8_t device, uint8_t channel, uint16_t registers[11])
{

  if (wavinController.readRegisters(MODBUS_DEVICES[device], WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 1, registers))
  {
    uint16_t setpoint = registers[0];
    
    String topic = String( MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_SETPOINT_GET);
    String payload = temperatureAsFloatString(setpoint);

    publishIfNewValue(topic, payload, setpoint, &(lastSentValues[ (device * WavinController::NUMBER_OF_CHANNELS) + channel].setpoint));
  }


}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  String topicString = String(topic);

  char terminatedPayload[length+1];
  for(unsigned int i=0; i<length; i++)
  {
    terminatedPayload[i] = payload[i];
  }
  terminatedPayload[length] = 0;
  String payloadString = String(terminatedPayload);

  uint8_t id = getIdFromTopic(topic);

  uint8_t modbusDevice = getModbusDevideFromTopic(topic);

  // Force re-read of registers from controller  
  lastUpdateTime = 0;

  if(topicString.endsWith(MQTT_SUFFIX_SETPOINT_SET))
  {
    uint16_t target = temperatureFromString(payloadString);
    wavinController.writeRegister(
      MODBUS_DEVICES[modbusDevice], 
      WavinController::CATEGORY_PACKED_DATA, 
      id, 
      WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 
      target);

    delayForOAT(1000);

    uint16_t registers[11];
    if (wavinController.readRegisters(MODBUS_DEVICES[modbusDevice], WavinController::CATEGORY_CHANNELS, id, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
    {
      delayForOAT(1000);

      readSetpoint( modbusDevice, id, registers);
    }
    
  }
  else if(topicString.endsWith(MQTT_SUFFIX_MODE_SET))
  {
    if(payloadString == MQTT_VALUE_MODE_MANUAL) 
    {
      wavinController.writeMaskedRegister(
        MODBUS_DEVICES[modbusDevice],
        WavinController::CATEGORY_PACKED_DATA,
        id,
        WavinController::PACKED_DATA_CONFIGURATION,
        WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL,
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
    else if (payloadString == MQTT_VALUE_MODE_STANDBY)
    {
      wavinController.writeMaskedRegister(
        MODBUS_DEVICES[modbusDevice],
        WavinController::CATEGORY_PACKED_DATA, 
        id, 
        WavinController::PACKED_DATA_CONFIGURATION, 
        WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY, 
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
  }
  
}


void resetLastSentValues()
{
  for(int8_t i = 0; i < (WavinController::NUMBER_OF_CHANNELS * WavinController::NUMBER_OF_DEVICES); i++)
  {
    lastSentValues[i].temperature = LAST_VALUE_UNKNOWN;
    lastSentValues[i].setpoint = LAST_VALUE_UNKNOWN;
    lastSentValues[i].battery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].status = LAST_VALUE_UNKNOWN;
    lastSentValues[i].mode = LAST_VALUE_UNKNOWN;

    configurationPublished[i] = false;
  }
}



// Publish discovery messages for HomeAssistant
// See https://www.home-assistant.io/docs/mqtt/discovery/
void publishConfiguration(uint8_t device, uint8_t channel)
{
  /*
   * Homeassistand discovery topic needs to follow a specific format: <discovery_prefix>/<component>/[<node_id>/]<object_id>/config
   * To get a <object_id> form <device> and <channel> I decidec to combine this as "(device * 100) + channel"
   * So e.g. homeassistant/climate/floorXXXXXXXXXXXX/1/3/config will change to homeassistant/climate/floorXXXXXXXXXXXX/103/config
   * That made discovery of things work in OpebHAB, but the discovery of channels did not.
   */

  uint8_t device_channel = (device * 100) + channel;
  String room = String(rooms[(ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel]);
  
  String climateTopic = String("homeassistant/climate/" + mqttDeviceNameWithMac + "/" + device_channel + "/config");
  String climateMessage = String(
    "{\"name\": \"" + room + "\", "
    "\"unique_id\": \"" + mqttDeviceNameWithMac + "_" + device + "_" + channel +  "_climate_id\", "
    "\"action_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_OUTPUT + "\", " 
    "\"current_temperature_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_CURRENT + "\", " 
    "\"temperature_command_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac  + "/" + device+ "/" + channel + MQTT_SUFFIX_SETPOINT_SET + "\", " 
    "\"temperature_state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_SETPOINT_GET + "\", " 
    "\"mode_command_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_MODE_SET + "\", " 
    "\"mode_state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_MODE_GET + "\", " 
    "\"modes\": [\"" + MQTT_VALUE_MODE_MANUAL + "\", \"" + MQTT_VALUE_MODE_STANDBY + "\"], " 
    "\"availability_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE +"\", "
    "\"payload_available\": \"True\", "
    "\"payload_not_available\": \"False\", "
    "\"min_temp\": \"" + String(MIN_TEMP, 1) + "\", "
    "\"max_temp\": \"" + String(MAX_TEMP, 1) + "\", "
    "\"temp_step\": \"" + String(TEMP_STEP, 1) + "\", "
    "\"qos\": \"0\"}"
  );
  
  String Battery = "Batteri på rumtermostat i ";
  String batteryTopic = String("homeassistant/sensor/" + mqttDeviceNameWithMac + "/" + device_channel + "/config");
  String batteryMessage = String(
    "{\"name\": \"" + Battery + room + "\", "
    "\"unique_id\": \"" + mqttDeviceNameWithMac + "_" + device + "_" + channel +  "_battery_id\", "
    "\"state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + "/battery\", " 
    "\"availability_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE +"\", "
    "\"payload_available\": \"True\", "
    "\"payload_not_available\": \"False\", "
    "\"device_class\": \"battery\", "
    "\"unit_of_measurement\": \"%\", "
    "\"qos\": \"0\"}"
  );

  mqttClient.publish(climateTopic.c_str(), climateMessage.c_str(), true);  
  mqttClient.publish(batteryTopic.c_str(), batteryMessage.c_str(), true);
  
  configurationPublished[ (device * WavinController::NUMBER_OF_CHANNELS) + channel] = true;
}

/***************************************************************************************************************
 ***************************************************************************************************************
 ***************************************************************************************************************
 ******************************   S E T U P     B E G I N ****************************************************** 
 ***************************************************************************************************************
 ***************************************************************************************************************
 ***************************************************************************************************************/
void setup()
{
  pinMode(ALT_LED_BUILTIN, OUTPUT);  // Initialize the LED pin as an output
  digitalWrite(ALT_LED_BUILTIN, LOW);  // Turn the LED on to indicate initialazation and not connneted for WiFi (Note that LOW is the voltage level

  
  uint8_t mac[6];
  WiFi.macAddress(mac);

  char macStr[13] = {0};
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  mqttDeviceNameWithMac = String(MQTT_DEVICE_NAME + macStr);
  mqttClientWithMac = String(MQTT_CLIENT + macStr);

  mqttClient.setServer(MQTT_SERVER.c_str(), MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  /*
   * Enable OTA update
   */
  ArduinoOTA.begin();
}

/***************************************************************************************************************
 ***************************************************************************************************************
 ***************************************************************************************************************
 ********************************   L O O P     B E G I N ****************************************************** 
 ***************************************************************************************************************
 ***************************************************************************************************************
 ***************************************************************************************************************/

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(ALT_LED_BUILTIN, LOW);  // Turn ON the LED to indicate WL connections is lost.
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED) return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    // Check for over the air update request and (if present) flash it
    ArduinoOTA.handle();
  
    digitalWrite(ALT_LED_BUILTIN, HIGH);  // Turn the LED off to indicate WL is connected
    if (!mqttClient.connected())
    {
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
      if (mqttClient.connect(mqttClientWithMac.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str(), will.c_str(), 1, true, "False") )
      {
          String setpointSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+/+" + MQTT_SUFFIX_SETPOINT_SET);
          mqttClient.subscribe(setpointSetTopic.c_str(), 1);
          
          String modeSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+/+" + MQTT_SUFFIX_MODE_SET);
          mqttClient.subscribe(modeSetTopic.c_str(), 1);
          
          mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, true);

          // Forces resending of all parameters to server
          resetLastSentValues();
      }
      else
      {
          return;
      }
    }
  
    // Process incomming messages and maintain connection to the server
    if(!mqttClient.loop())
    {
        return;
    }


    if (lastUpdateTime + POLL_TIME_MS < millis())
    {
      uint16_t registers[11];
      for(uint8_t device = 0; device < WavinController::NUMBER_OF_DEVICES; device++)
      {
        for(uint8_t channel = 0; channel < WavinController::NUMBER_OF_CHANNELS; channel++)
        {
          if (wavinController.readRegisters(MODBUS_DEVICES[device], WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
          {
            uint16_t primaryElement = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ELEMENT_MASK;
            bool allThermostatsLost = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ALL_TP_LOST_MASK;

            if(primaryElement==0)
            {
                // Channel not used
                continue;
            }

            // Publish configuration if not poblished.
            if(!configurationPublished[ (device * WavinController::NUMBER_OF_CHANNELS) + channel])
            {
              uint16_t standbyTemperature = STANDBY_TEMPERATURE_DEG * 10;
              wavinController.writeRegister(MODBUS_DEVICES[device], WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_STANDBY_TEMPERATURE, standbyTemperature);
              publishConfiguration(device, channel);
            }

            delayForOAT(125);

            // Read the current setpoint for the chanel
            readSetpoint( device, channel, registers);

            // Read the current mode for the channel
            if (wavinController.readRegisters(MODBUS_DEVICES[device], WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_CONFIGURATION, 1, registers))
            {
              uint16_t mode = registers[0] & WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK; 

              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_MODE_GET);
              if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY)
              {
                publishIfNewValue(topic, MQTT_VALUE_MODE_STANDBY, mode, &(lastSentValues[ (device * WavinController::NUMBER_OF_CHANNELS) + channel].mode));
              }
              else if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL)
              {
                publishIfNewValue(topic, MQTT_VALUE_MODE_MANUAL, mode, &(lastSentValues[ (device * WavinController::NUMBER_OF_CHANNELS) + channel].mode));
              }            
            }

            // Read the current status of the output for channel
            if (wavinController.readRegisters(MODBUS_DEVICES[device], WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_TIMER_EVENT, 1, registers))
            {
              uint16_t status = registers[0] & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK;

              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_OUTPUT);
              String payload;
              if (status & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK)
                payload = "heating";
              else
                payload = "off";

              publishIfNewValue(topic, payload, status, &(lastSentValues[ (device * WavinController::NUMBER_OF_CHANNELS) + channel].status));
            }

            // If a thermostat for the channel is connected to the controller
            if(!allThermostatsLost)
            {
              // Read values from the primary thermostat connected to this channel 
              // Primary element from controller is returned as index+1, so 1 i subtracted here to read the correct element
              if (wavinController.readRegisters(MODBUS_DEVICES[device], WavinController::CATEGORY_ELEMENTS, primaryElement-1, 0, 11, registers))
              {
                uint16_t temperature = registers[WavinController::ELEMENTS_AIR_TEMPERATURE];
                uint16_t battery = registers[WavinController::ELEMENTS_BATTERY_STATUS]; // In 10% steps

                String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_CURRENT);
                String payload = temperatureAsFloatString(temperature);

                publishIfNewValue(topic, payload, temperature, &(lastSentValues[ (device * WavinController::NUMBER_OF_CHANNELS) + channel].temperature));

                topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_BATTERY);
                payload = String(battery*10);

                publishIfNewValue(topic, payload, battery, &(lastSentValues[ (device * WavinController::NUMBER_OF_CHANNELS) + channel].battery));
              }
            }         
          }

          // Process incomming messages and maintain connection to the server
          if(!mqttClient.loop())
          {
              return;
          }
        }
      }
      lastUpdateTime = millis();
    }
  }
}
