#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "WavinController.h"
#include "PrivateConfig.h"

#define SKTECH_VERSION "Esp8266 MQTT Wavin AHC 9000 interface - V0.1.1"

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
 * It turned out, that reading all channels took longer than the defined POLL_TIME_MS = 5000 (Change to POLL_TIME SEC i ver. 0.1.0), and since lastUpdateTime were updated at the beginning
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
 * Issues now documented on github
 * Version history documented in README.md
 */

/*
 * ######################################################################################################################################
 * ######################################################################################################################################
 *                       V  A  R  I  A  B  L  E      D  E  F  I  N  A  I  T  O  N  S
 * ######################################################################################################################################
 * ######################################################################################################################################
 */

#define WIFI_CONNECT_POSTPONE 30        // Number of secunds between WiFi connect attempts, when WiFi.begin fails to connect.
#define MQTT_CONNECT_POSTPONE 30        // Number of secunds between MQTT connect dattempts, when MQTT connect fails to connect.

#define RETAINED true                   // Used in MQTT puplications. Can be changed during development and bugfixing.

// MQTT defines
// Esp8266 MAC will be added to the device name, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/1/3/target', where 1 is the Modbus Device number and 3 is the output id and XXXXXXXXXXXX is the mac

const String   MQTT_PREFIX              = "heat/";       // include tailing '/' in prefix
const String   MQTT_DEVICE_NAME         = "floor";       // only alfanumeric and no '/'
const String   MQTT_ONLINE              = "/online";      
const String   MQTT_SKTECH_VERSION      = "/sketch_version";
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

unsigned long lastUpdateTime = 0;       // Timestamp in Secunds when channels were read and HA updated.

/* Wariables to handle connect postpone */
unsigned long WiFiConnectAttempt = 0;   // Timestamp in Secunds when an attempt to connect to WiFi were done
unsigned long MQTTConnectAttempt = 0;   // Timestamp in Secunds when an attempt to connect to MQtT were done

unsigned long WiFiConnectPostpone = 0;  // Secunds between each attempt to connect to WiFi.
unsigned long MQTTConnectPostpone = 0;  // Secunds between each attempt to connect to MQTT.

const uint16_t POLL_TIME_SEC = 5;        // Secunds between each poll of channels

// Create a structure for each channel on all heat controllers controlled by this project.
struct lastKnownValue_t {
  uint16_t temperature;
  uint16_t setpoint;
  uint16_t battery;
  uint16_t status;
  uint16_t mode;
} lastSentValues[ PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[PrivateConfig::NUMBER_OF_DEVICES]];

const uint16_t LAST_VALUE_UNKNOWN = 0xFFFF;

bool configurationPublished[PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[PrivateConfig::NUMBER_OF_DEVICES]];

/* ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 * ##########################    F U N C T I O N    D E C L A R A T I O N S    ##############################
 * ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
*/
void delayForOAT(uint16_t);
uint16_t temperatureFromString(String);
String temperatureAsFloatString(uint16_t);
uint8_t getIdFromTopic(char*);
uint8_t getModbusDeviceFromTopic(char*);
void publishIfNewValue(String, String, uint16_t, uint16_t*);
void readSetpoint( uint8_t, uint8_t, uint16_t registers[11]);
void mqttCallback(char*, byte*, unsigned int);
void resetLastSentValues();
void publishConfiguration(uint8_t device, uint8_t channel);
void publish_sketch_version();
/* Variables only used by SEC() to keep track on millis() overruns*/
uint16_t millisOverruns = 0;
unsigned long secLastCalledAt = 0;
unsigned long sec();

/* ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 * ################################   S E T U P     B E G I N   ###########################################
 * ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 */
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

  mqttClient.setServer(PrivateConfig::MQTT_SERVER.c_str(), PrivateConfig::MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  /*
   * Enable OTA update
   */
  ArduinoOTA.begin();
}
/* ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 * ################################     L O O P    B E G I N    ###########################################
 * ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 */
void loop()
{
  if (WiFi.status() != WL_CONNECTED and sec() > WiFiConnectAttempt + WiFiConnectPostpone)
  {
    digitalWrite(ALT_LED_BUILTIN, LOW);  // Turn ON the LED to indicate WL connections is lost.
    
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(PrivateConfig::WIFI_SSID.c_str(), PrivateConfig::WIFI_PASS.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      WiFiConnectAttempt = sec();
      WiFiConnectPostpone = WIFI_CONNECT_POSTPONE;
      return;
    } 
    else
    {
      WiFiConnectAttempt = 0;   // In case WiFi is lost, attempt to reconnect immediatly. 
      WiFiConnectPostpone = 0;
    }
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    // Check for over the air update request and (if present) flash it
    ArduinoOTA.handle();
  
    digitalWrite(ALT_LED_BUILTIN, HIGH);  // Turn the LED off to indicate WL is connected
    if (!mqttClient.connected() and sec() > MQTTConnectAttempt + MQTTConnectPostpone )
    {
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
      if (mqttClient.connect(mqttClientWithMac.c_str(), PrivateConfig::MQTT_USER.c_str(), PrivateConfig::MQTT_PASS.c_str(), will.c_str(), 1, true, "False") )
      {
        MQTTConnectAttempt = 0;
        MQTTConnectPostpone = 0;

        publish_sketch_version();

        String setpointSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+/+" + MQTT_SUFFIX_SETPOINT_SET);
        mqttClient.subscribe(setpointSetTopic.c_str(), 1);
        
        String modeSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+/+" + MQTT_SUFFIX_MODE_SET);
        mqttClient.subscribe(modeSetTopic.c_str(), 1);
        
        mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, RETAINED);

        // Forces resending of all parameters to server
        resetLastSentValues();
      }
      else
      {
        MQTTConnectAttempt = sec();
        MQTTConnectPostpone = MQTT_CONNECT_POSTPONE;
        return;
      }
    }
  
    // Process incomming messages and maintain connection to the server
    if(!mqttClient.loop())
    {
        return;
    }
    
    // Polling all channels on all devices
    if (lastUpdateTime + POLL_TIME_SEC < sec())
    {
      uint16_t registers[11];
      for(uint8_t device = 0; device < PrivateConfig::NUMBER_OF_DEVICES; device++)
      {
        for(uint8_t channel = 0; channel < PrivateConfig::NUMBER_OF_CHANNELS_MONITORED_PER_DEVICE[device]; channel++)
        {
          if (wavinController.readRegisters(PrivateConfig::MODBUS_DEVICES[device], WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
          {
            uint16_t primaryElement = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ELEMENT_MASK;
            bool allThermostatsLost = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ALL_TP_LOST_MASK;

            if(primaryElement==0)
            {
                // Channel not used
                continue;
            }

            // Publish configuration if not poblished.
            if(!configurationPublished[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel])
            {
              uint16_t standbyTemperature = STANDBY_TEMPERATURE_DEG * 10;
              wavinController.writeRegister(PrivateConfig::MODBUS_DEVICES[device], WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_STANDBY_TEMPERATURE, standbyTemperature);
              publishConfiguration(device, channel);
            }

            delayForOAT(125);

            // Read the current setpoint for the chanel
            readSetpoint( device, channel, registers);

            // Read the current mode for the channel
            if (wavinController.readRegisters(PrivateConfig::MODBUS_DEVICES[device], WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_CONFIGURATION, 1, registers))
            {
              uint16_t mode = registers[0] & WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK; 

              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_MODE_GET);
              if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY)
              {
                publishIfNewValue(topic, MQTT_VALUE_MODE_STANDBY, mode, &(lastSentValues[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel].mode));
              }
              else if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL)
              {
                publishIfNewValue(topic, MQTT_VALUE_MODE_MANUAL, mode, &(lastSentValues[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel].mode));
              }            
            }

            // Read the current status of the output for channel
            if (wavinController.readRegisters(PrivateConfig::MODBUS_DEVICES[device], WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_TIMER_EVENT, 1, registers))
            {
              uint16_t status = registers[0] & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK;

              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_OUTPUT);
              String payload;
              if (status & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK)
                payload = "heating";
              else
                payload = "off";

              publishIfNewValue(topic, payload, status, &(lastSentValues[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel].status));
            }

            // If a thermostat for the channel is connected to the controller
            if(!allThermostatsLost)
            {
              // Read values from the primary thermostat connected to this channel 
              // Primary element from controller is returned as index+1, so 1 i subtracted here to read the correct element
              if (wavinController.readRegisters(PrivateConfig::MODBUS_DEVICES[device], WavinController::CATEGORY_ELEMENTS, primaryElement-1, 0, 11, registers))
              {
                uint16_t temperature = registers[WavinController::ELEMENTS_AIR_TEMPERATURE];
                uint16_t battery = registers[WavinController::ELEMENTS_BATTERY_STATUS]; // In 10% steps

                String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_CURRENT);
                String payload = temperatureAsFloatString(temperature);

                publishIfNewValue(topic, payload, temperature, &(lastSentValues[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel].temperature));

                topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_BATTERY);
                payload = String(battery*10);

                publishIfNewValue(topic, payload, battery, &(lastSentValues[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel].battery));
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
      lastUpdateTime = sec();
    }
  }
}
/* ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 * #####################   F U N C T I O N    D  E  F  I  N  I  T  I  O  N  S    ##########################
 * ########################################################################################################
 * ########################################################################################################
 * ########################################################################################################
 */

/* ###################################################################################################
 *                                    F U N C T I O N    N A M E
 * ###################################################################################################
 * fundction()
 * Description:
 * Syntax: 
 * Parameters:
 *  -  
 * Returns: 
 */


/* ###################################################################################################
 *                                    D E L A Y   F O R   O A T
 * ###################################################################################################
 * delayForOAT()
 * Description: Alternative to delay(). Pauses the program for the amount of time, the time it takes
 *              to call ArduinoOTA.handle() a number of times, specified as parameter.
 *              The functino makes it possible to OTA during a delay.
 * Syntax: delayForOAT(iterations)
 * Parameters:
 *  - iterations: the number of times ArduinoOTA.handle() is called to create a delay
 * Returns: Nothing
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

/* ###################################################################################################
 *                T E M P E R A T U R E   F R O M   S T R I N G
 * ###################################################################################################
 * temperatureFromString()
 * Description: Read a float value from a non zero terminated array of bytes and 
 *              return 10 times the value as an integer
 * Syntax: temperatureFromString( thisString)
 * Parameters:
 *  - thisString: Floatvalue represented in a string.
 * Returns: The float value as an integer, multiplied by 10. 
 */

uint16_t temperatureFromString(String payload)
{
  float targetf = payload.toFloat();
  return (unsigned short)(targetf * 10);
}

/* ###################################################################################################
 *                       T E M P E R A T U R E   A S   F L O A T   S T R I N G
 * ###################################################################################################
 * temperatureAsFloatString()
 * Description: Transform an intergervalue, representing a temperature with one decimal, to a foat
 *              with one decimal
 * Syntax: temperatureAsFloatString(temperatureAsInteger)
 * Parameters:
 *  - temperatureAsInteger: An integer representation of a temperature having one decimal.
 * Returns: Temperature in degrees with one decimal
 */

// Returns temperature in degrees with one decimal
String temperatureAsFloatString(uint16_t temperature)
{
  float temperatureAsFloat = ((float)temperature) / 10;
  return String(temperatureAsFloat, 1);
}

/* ###################################################################################################
 *                      G E T    I D    F R O M   T O P I C
 * ###################################################################################################
 * getIdFromTopic()
 * Description: Get the room thermostat ID number from the topic string.
 * Syntax: getIdFromTopic(thisTopic)
 * Parameters:
 *  -  thisTopic: A chararray in the form [MQTT_PREFIX + mqttDeviceNameWithMac + "/" + ModbusDevice + "/" + ID +"/#"]
 * Returns: Room thermostat ID number
 */
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

/* ###################################################################################################
 *                     G E T   M O D B U S    D E V I C E    F R O M    T O P I C 
 * ###################################################################################################
 * getModbusDeviceFromTopic()
 * Description: Get the Modbus device (AHC9000 Control Unit) number from the topic string.
 * Syntax: getIdFromTopic(thisTopic)
 * Parameters:
 *  - thisTopic: A chararray in the form [MQTT_PREFIX + mqttDeviceNameWithMac + "/" + ModbusDevice + "/" + ID +"/#"]
 * Returns: Modbus device (AHC9000 Control Unit) number
 */

uint8_t getModbusDeviceFromTopic(char* topic)
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

/* ###################################################################################################
 *                              P U B L I S H    I F    N E W    V A L U E
 * ###################################################################################################
 * publishIfNewValue()
 * Description: Publist newValue as (streig)payload, if newValue is != lastSentValue
 * Syntax: publishIfNewValue(topic, payload, newValue, lastSentValue)
 * Parameters:
 *  - topic:        A char array (String) in the form [MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_...]
 *  - payload       NewValue as a char array (String)
 *  - newValue      A interger value
 *  - lastSentValue A pointer to tha last sent value
 * Returns: Nothing
 */
void publishIfNewValue(String topic, String payload, uint16_t newValue, uint16_t *lastSentValue)
{
  if (newValue != *lastSentValue)
  {
    if (mqttClient.publish(topic.c_str(), payload.c_str(), RETAINED))
    {
        *lastSentValue = newValue;
    }
    else
    {
      *lastSentValue = LAST_VALUE_UNKNOWN;
    }
  }
}


/* ###################################################################################################
 *                                  R E A D    S E T    P O I N T 
 * ###################################################################################################
 * readSetpoint()
 * Description: Read setpoint from Wavin controller at publish setpoint to MQTT
 * Syntax: readSetpoint( device, channel, registers)
 * Parameters:
 *  - device:       Modbus device (AHC9000 Control Unit) number
 *  - channel:      Thermostat ID number
 *  - registers:    
 * Returns: Nothing
 */
void readSetpoint( uint8_t device, uint8_t channel, uint16_t registers[11])
{

  if (wavinController.readRegisters(PrivateConfig::MODBUS_DEVICES[device], WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 1, registers))
  {
    uint16_t setpoint = registers[0];
    
    String topic = String( MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_SETPOINT_GET);
    String payload = temperatureAsFloatString(setpoint);

    publishIfNewValue(topic, payload, setpoint, &(lastSentValues[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel].setpoint));
  }


}


/* ###################################################################################################
 *                         M Q T T    C A L L    B A C K
 * ###################################################################################################
 * mqttCallback()
 * Description: Called when MQTT message arrive
 * Syntax:        mqttCallback(* topic,* payload, length)
 * Parameters:
 *  - * topic:
 *  - * payload:
 *  - length
 * Returns: Nothing
 */
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

  uint8_t modbusDevice = getModbusDeviceFromTopic(topic);

  // Force re-read of registers from controller  
  lastUpdateTime = 0;

  if(topicString.endsWith(MQTT_SUFFIX_SETPOINT_SET))
  {
    uint16_t target = temperatureFromString(payloadString);
    wavinController.writeRegister(
      PrivateConfig::MODBUS_DEVICES[modbusDevice], 
      WavinController::CATEGORY_PACKED_DATA, 
      id, 
      WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 
      target);

    delayForOAT(1000);

    uint16_t registers[11];
    if (wavinController.readRegisters(PrivateConfig::MODBUS_DEVICES[modbusDevice], WavinController::CATEGORY_CHANNELS, id, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
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
        PrivateConfig::MODBUS_DEVICES[modbusDevice],
        WavinController::CATEGORY_PACKED_DATA,
        id,
        WavinController::PACKED_DATA_CONFIGURATION,
        WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL,
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
    else if (payloadString == MQTT_VALUE_MODE_STANDBY)
    {
      wavinController.writeMaskedRegister(
        PrivateConfig::MODBUS_DEVICES[modbusDevice],
        WavinController::CATEGORY_PACKED_DATA, 
        id, 
        WavinController::PACKED_DATA_CONFIGURATION, 
        WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY, 
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
  }
  
}


/* ###################################################################################################
 *                     R E S E T    L A S T    S E N T    V A L U E S
 * ###################################################################################################
 * resetLastSentValues()
 * Description: Resetting last sent values
 * Syntax: resetLastSentValues()
 * Parameters: none
 * Returns: Global variabes set to LAST_VALUE_UNKNOWN
 */
void resetLastSentValues()
{
  for(int8_t i = 0; i < (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[PrivateConfig::NUMBER_OF_DEVICES]); i++)
  {
    lastSentValues[i].temperature = LAST_VALUE_UNKNOWN;
    lastSentValues[i].setpoint = LAST_VALUE_UNKNOWN;
    lastSentValues[i].battery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].status = LAST_VALUE_UNKNOWN;
    lastSentValues[i].mode = LAST_VALUE_UNKNOWN;

    configurationPublished[i] = false;
  }
}


/* ###################################################################################################
 *                       P U B L I S H    C O N F I G U R A T I O N
 * ###################################################################################################
 * publishConfiguration()
 * Description: Publish discovery messages for HomeAssistant
 *              See https://www.home-assistant.io/docs/mqtt/discovery/
 * Syntax: publishConfiguration( device, channel)
 * Parameters:
 *  - device:
 *  - channel
 * Returns: 
 */
void publishConfiguration(uint8_t device, uint8_t channel)
{
  /*
   * Home Assistand discovery topic need to follow a specific format: <discovery_prefix>/<component>/[<node_id>/]<object_id>/config
   * To get a <object_id> form <device> and <channel> I decidec to combine this as "(device * 100) + channel"
   * So e.g. configuration for device 1 channel 3 will be homeassistant/climate/floorXXXXXXXXXXXX/103/config
   * 
   */
  uint8_t payload[1024];
  JsonDocument climateDoc;
  JsonDocument sensorDoc;

  uint8_t device_channel = (device * 100) + channel;

  String room = String(PrivateConfig::ROOMS[(PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel]);
  
  climateDoc["name"] = room;
  climateDoc["unique_id"] = String(mqttDeviceNameWithMac + "_" + device + "_" + channel +  "_climate_id");
  climateDoc["action_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_OUTPUT);
  climateDoc["current_temperature_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_CURRENT);
  climateDoc["temperature_command_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_SETPOINT_SET);
  climateDoc["temperature_state_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_SETPOINT_GET);
  climateDoc["mode_command_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_MODE_SET);
  climateDoc["mode_state_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + MQTT_SUFFIX_MODE_GET);

  climateDoc["modes"][0] = String(MQTT_VALUE_MODE_MANUAL);
  climateDoc["modes"][1] = String(MQTT_VALUE_MODE_STANDBY);
  
  climateDoc["availability_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
  climateDoc["payload_available"] = "True";
  climateDoc["payload_not_available"] = "False";
  climateDoc["min_temp"] = String(PrivateConfig::MIN_TEMP, 1);
  climateDoc["max_temp"] = String(PrivateConfig::MAX_TEMP, 1);
  climateDoc["temp_step"] = String(PrivateConfig::TEMP_STEP, 1);
  climateDoc["qos"] = "0";

  size_t climateLength = serializeJson(climateDoc, payload);
  String climateTopic = String("homeassistant/climate/" + mqttDeviceNameWithMac + "/" + device_channel + "/config");

  mqttClient.publish(climateTopic.c_str(), payload, climateLength, RETAINED);


  String Battery = "Batteri på rumtermostat i ";

  sensorDoc["name"] = String(Battery + room);
  sensorDoc["unique_id"] = String(mqttDeviceNameWithMac + "_" + device + "_" + channel + "_battery_id");
  sensorDoc["state_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + device + "/" + channel + "/battery");
  sensorDoc["availability_topic"] = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
  sensorDoc["payload_available"] = "True";
  sensorDoc["payload_not_available"] = "False";
  sensorDoc["device_class"] = "battery";
  sensorDoc["unit_of_measurement"] = "%";
  sensorDoc["qos"] = "0";

  size_t sensorlength = serializeJson(sensorDoc, payload);
  String sensorTopic = String("homeassistant/sensor/" + mqttDeviceNameWithMac + "/" + device_channel + "/config");
  
  mqttClient.publish(sensorTopic.c_str(), payload, sensorlength, RETAINED);
  
  configurationPublished[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel] = true;
}

/* ###################################################################################################
 *                  P U B L I S H   S K E T C H   V E R S I O N
 * ###################################################################################################
 *  As the sketch will esecute in silence, one way to se which version of the SW is running will be
 *  to subscribe to topic defined as (MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SKTECH_VERSION)
 *  on the MQTT broker, configured in the privateConfig.h file.
 *  For the current settings, subscribe to: heat/+/sketch_version
 *  This will return the value of (SKETCH_VERSION) plus the SSID it is connected to and the IP address.
 */
void publish_sketch_version()   // Publish only once at every reboot.
{  
  IPAddress ip = WiFi.localIP();
  String versionTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_SKTECH_VERSION);
  String versionMessage = String(SKTECH_VERSION + String("\nConnected to SSID: \'") +\
                                  String(PrivateConfig::WIFI_SSID) + String("\' at: ") +\
                                  String(ip[0]) + String(".") +\
                                  String(ip[1]) + String(".") +\
                                  String(ip[2]) + String(".") +\
                                  String(ip[3]));

  mqttClient.publish(versionTopic.c_str(), versionMessage.c_str(), RETAINED);
}

/*
 * ###################################################################################################
 *              S E C   -   S Y S T E M T I M E    I N   S E C U N D S
 * ###################################################################################################
 * 
 * sec()
 * 
 * [Time]
 *
 * Description
 * 
 * Returns the number of secunds passed since the board began running the current program. 
 * This number will overflow (go back to zero), after approximately 136 years.
 * 
 * This version is based on millis(), which overflows (go back to zero), after approximately 50 days.
 * It requires a two global variables:
 * - uint16_t millisOverruns: To keeb track on, how many overruns has occoured.
 * - unsigned long secLastCalledAt: To keep track on, when an overrun has occoured.
 * 
 * Limitations
 * 
 * The function has to be called at least a couple of times, within approximately 50 days to keep track 
 * of overruns.
 * 
 * Syntax
 * 
 * time = sec()
 * 
 * Parameters
 * 
 * None
 * 
 * Returns
 * 
 * Number of seconds passed since the program started. Data type: unsigned long.
 * 
 * Example Code
 * This example code prints on the serial port the number of seconds passed since the board started running the code itself.


unsigned long myTime;

// Variables used by SEC() to keep track on millis() overruns
uint16_t millisOverruns;
unsigned long secLastCalledAt;
// Fundcion definition.
unsigned long sec();
 

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Time: ");
  unsigned long myTime = sec();

  Serial.println(myTime); // prints time since program started
  delay(1000);          // wait a second so as not to send massive amounts of data
}
 *
 */

unsigned long sec()
{
  unsigned long m = millis();

  if ( m < secLastCalledAt)
    millisOverruns++;

  secLastCalledAt = m;

  return (millisOverruns * (pow(2, 32) / 1000)) + ( m / 1000);
}

