# **Wavin-AHC-9000-mqtt**


A Esp8266 mqtt interface for more Wavin AHC-9000 controllers connected together, configured and controlled by one Wavin display.

The goal is to be able to control the Wavin AHC 9000 heating controllers from [Home Assistant](https://www.home-assistant.io/) (HA), and still be able to use the Wavin display as well. However MODBUS specifications does not allow more than one master. The Wavin display will not be working together with the Esp8266 mqtt interface, at the same tine, in the current version. The current solution is to use a 4 poles toggle switch, to manually switch between the Esp8266 mqtt interface and the Wavin display.

The original goal of being able to control the Wavin AHC 9000 heating controllers from [OpenHAP](https://www.openhab.org/), has been abandoned because I gave up finding a discovery function for MQTT integrations, like the one in HA.

### **Operation**

The Esp8266 mqtt interface is generally operated from [HA](https://www.home-assistant.io/). When started, it will publish a configuration for each thermostat connected to a MQTT Broker, from where HA will pickup the configurations.

Besides controlling the single thermostat from HA, the Esp8266 mqtt interface provide a MQTT switch for HA named "Wavin interface operation mode", which provide an easy way to change the setpoints for all thermostats connected. Currently four operations modes are configured in the PrivateConfig.h file:
  
0. Normal operation
1. Holiday operation
2. Surplus heat operation
3. Maintenance operation**

These operations modes can also be selected by publishing a [JSON](https://en.wikipedia.org/wiki/JSON) to:
````bash
heat/floorE8DB84E05CC8/config

````
Publish the following JSON will set the operation mode to **Holiday operation**
````bash
{
  "opmode": "1"
}
````

**Maintenance operation:** This operation mode will set the setpoint temperature for all thermostats to maximum temperature (MAX_TEMP) and run in this mode for a period of time, and then return to previous operation mode. All parameters are defined in PrivateConfig.h

### **Current project status**

The Wavin AHC-9000 controllers are initial installed and configured using the Wavin display, which configures each controller's MOC-BUS device number.

Future solution might be to use a 2 Circuit IC Switch [CD4052B](https://www.ti.com/product/CD4052B-MIL#tech-docs), managed by the Esp8266 mqtt interface, to control which controller (Wavin display or Esp8266) will be in action.


### **History**

This project has started as a git clone of [dkjonas/Wavin-AHC-9000-mqtt](https://github.com/dkjonas/Wavin-AHC-9000-mqtt.git) but restructured for Firmware (FW), Software (SW) and Hardware (HW).

This project takes on the challenge mentioned in [Issue #3](https://github.com/dkjonas/Wavin-AHC-9000-mqtt/issues/3#issuecomment-435690672)

##### "connect two AHC's together and control them by one display. I think this is rather unusual setup, so my code doesn't handle this case; It can only control one AHC."


Another interesting project: [nic6911/Wavin-AHC-9000-mqtt](https://github.com/nic6911/Wavin-AHC-9000-mqtt)


## **Esp8266 mqtt interface**

See the cloned Firmware markdown [here](Firmware/README.md).

## **SOFTWARE**

### Configuration

src/PrivateConfig.h contains constants, that should be changed to fit your own setup.

#### Notes for some of the constants:
MODBUS_DEVICES[] 
>This array defines the Modbus device id, for the AHC9000 Control Units, connected.<br> 
When AHC9000 Control Units are connected together with the AHC9000 Touch Screen Display (Display), 
then Modbus ID's are configured by the Display.<br>
The ID's are in binary representation starting with 0x02<br>
Having two AHC9000 Control Units connected, the definition will be:
````bash
MODBUS_DEVICES[] = {0x02, 0x03};
````

NUMBER_OF_CHANNELS_MONITORED_PER_DEVICE[]
>This is an array, defines how many AHC9000 Thermostats are connected to / monitored by each AHC9000 Control Units.<br>
When 11 Thermostats are connected to AHC9000 Control Unit 0x02 and 4 Thermostats are connected to AHC9000 Control Unit 0x03, the definition vill be:
````bash
NUMBER_OF_CHANNELS_MONITORED_PER_DEVICE[] = {11, 4}
````

ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[]
>This array is used to define number of elements in other arrays and to find specific element in in these other arrays.<br>
Elements in other arrays represents Thermostats connected.<br>
The first element in this array is always zero. <br>
The next element (the second element) is the number of Thermostats connected to the first AHC Controller. <br>
The third element is the number of Thermostats connected to the first AHC Controller plus the number of Thermostats connected to the second AHC Controller.
The foruth element is the number of Thermostats connected to the first AHC Controller plus the number of Thermostats connected to the second AHC Controller plus the number of Thermostats connected to the third AHC Controller.<br>
And so on...
The last element in this array will represent the total number of Thermostats conencted / monitored.<br>
This way number of elements in other arrays can be defined as:<br>
bool configurationPublished[PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[PrivateConfig::NUMBER_OF_DEVICES]];
Elements in an array can be found by:<br>
configurationPublished[ (PrivateConfig::ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[device]) + channel]<br>
Where "channel" represent the Thermostat for the "device" in question.
When 11 Thermostats are connected to AHC9000 Control Unit 0x02 and 4 Thermostats are connected to AHC9000 Control Unit 0x03, the definition vill be:
````bash
ELEMENT_OFFSET_ON_ROOMS_FOR_DEVICE[] = {0,11,15}
````

### Compiling

The sketch use [PubSubClient](https://docs.arduino.cc/libraries/pubsubclient/)2.6 and [ArduinoJson](https://github.com/bblanchon/ArduinoJson/releases)7.0.1. Add the following to the plarform.ini file.
````bash
lib_deps =
  PubSubClient@2.6
  bblanchon/ArduinoJson@7.0.1
````

Larger buffer is needed for HomeAssistant discovery messages, which are quite large.

Add the following to the plarform.ini file.
````bash
build_flags = -D MQTT_MAX_PACKET_SIZE=1024
````
Over The Air update (OTA) is implemented in main.cpp. 

Compiled with [PlatformIO](https://platformio.org/) installed in VS Code and adding the follow to platformio.ini the code will provide the possability of setting PlatformIO Project Environment to   `env:nodemcu_ota` and upload will happen OTA.

````bash
[env:nodemcu_ota]
extends = env:nodemcu
upload_protocol = espota
upload_port = <IP Address of NodeMCU>
````


## **Mosquitto broker**

### **Requirements**

- Raspberry Pi model B v1.2. Installed with Raspberry Pi OS Lite (32-bit) <span id="a1">[[1]](#f1)</span>.
- Internet access.
- Docker Engin and Docker Compose installed <span id="a2">[[2]](#f2)</span>.


### **Create docker-compose file**

In folder `Wavin-AHC-9000-mqtt` create file `docker-compose.yaml`

```bash
vi docker-compose.yaml
```

Add the following contend:

```bash
# Docker-compose file for building Wavin-AHC-9000-mqtt program stack.

# Generel note about environment settings - see README.md file about .env file

version: '3.8'

services:
  mosquitto-mqtt:
  container_name: mosquitto-mqtt
  image: eclipse-mosquitto:2
  restart: always
  command: mosquitto -c /mosquitto-no-auth.conf
  ports:
    - 1883:1883
  volumes:
    - mosquitto-conf:/mosquitto/config
    - mosquitto-data:/mosquitto/data
    - mosquitto-log:/mosquitto/log

volumes:
  mosquitto-conf:
    name: mosquitto-conf
  mosquitto-data:
    name: mosquitto-data
  mosquitto-log:
    name: mosquitto-log

````


 Start the docker container with `docker-compose up -d`.

```bash
docker-compose up -d
```

## Change LOG

|       | Module              | Version | Change
|---    |---                  | ----    |----
|       |                     | 0.0.1   | Initial commit. On-board LED at PIN 16 added to indicate active boot up. It turns off at successful connect to Wireless Lan. 
| FW    | WavinController.h   | 0.0.2   | MODBUS_DEVICE changed from = 0x01 to 0x02
| FW    | WavinController.h   | 0.0.3   | MODBUS_DEVICE changed from = 0x02 to 0x03
| FW    | WavinController.h   | 0.0.4   | Added MODBUS_DEVICES and NUMBER_OF_DEVIDES
| FW    | WavinController.cpp | 0.0.4   | Objects modified to handle MODBUS_DEVICE as an argument instad of fixed value.
| FW    | All modules         | 0.0.4   | Calls to objects, which has changed because of the objects being modified for handle MODBUS_DEVICE as an argument. MQTT Publish functions has been added Modbus Device.
| FW    | main.cpp            | 0.0.5   | publishConfiguration() changed to comply with homeassistant topic.
| FW    | main.cpp            | 0.0.6   | Implementing OTA update.
| FW    | main.cpp            | 0.0.7   | OTA and temerature setting issues.
| FW    | main.cpp & PrivateConfig.h | 0.0.8   | Configurable names for chanels added to PrivateConfig.h and implemented in mail.cpp
| FW    | main.cpp            | 0.0.9 | Re-arrange code structure. Insert function definitions, startup() and loot() in beginning.
| FW    | main.cpp            | 0.0.10 | #4: Insert WiFi reconnect postpone time <br>#5: call WiFi.disconnect() before "reconnect" (WiFi.mode(WIFI_STA) og WiFi.begin() <br>#2: Publish device info to MQTT broker
| FW    | main.cpp            | 0.0.11 | #3: implement Arduino JSON
| FW    | main.cpp            | 0.1.0  | #11: millis() overrun: Implementing sec() and change code from using milli seconds, to use secunds for timestamps
| FW    | main.cpp            | 0.1.1  | #10: Scan only installed sensors<br>PrivateConfig.h streamlined with WavinController.h. Global constants moved into class PrivateConfig as static ...
| FW    | main.cpp            | 0.1.2  | Tracking wavinController.readRegisters. This version publishes failed calls to readRegisters in order og verify how long it stakes before alle Thermostats are read.
| FW    | main.cpp            | 0.1.3  | #8: Implement reading of floor temperature
| FW    | main.cpp            | 0.1.4  | Track unsuccessful for calls to wavinController.readRegisters
| FW    | main.cpp            | 1.0.0  | #6: Preset functionality (Holiday function, Removal of surplus heat)
| FW    | main.cpp            | 1.1.0  | #9 Implement Maintenance fuktionality 
| FW    | main.cpp            | 1.2.0  | Setpoints 
| FW    | main.cpp            | 1.2.1  | Setpoints bugfixes.
| FW    | main.cpp            | 1.2.3  | #12 Set temperature not change, when changing operation mode 





###### "#n" Refers to github issue number





## Credits
- dkjonas/Wavin-AHC-9000-mqtt

#### **Footnotes**
=======
###### 1. <span id="f1"></span> See document: [Setting up and configure Raspberry Pi 3 MODEL B+ for the Docker Traefik router project](./docs/SettingUpRaspberryPi.md). [$\hookleftarrow$](#a1)

###### 2. <span id="f2"></span> See document [Install Docker Engin and Docker Compose](./docs/InstallDockerEnginAndCompose.md)[$\hookleftarrow$](#a3)
