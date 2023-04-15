# **Wavin-AHC-9000-mqtt**

Git clone of https://github.com/dkjonas/Wavin-AHC-9000-mqtt.git but restructured for Firmware (FW), Software (SW) and Hardware (HW).

Taking on the challange mentioned in [Issue #3](https://github.com/dkjonas/Wavin-AHC-9000-mqtt/issues/3#issuecomment-435690672) 

##### "connect two AHC's together and control them by one display. I think this is rather unusual setup, so my code doesn't handle this case; It can only control one AHC."

I happen to have such a setup, and would like still to be able to manage floor heating form the display.

## **Currently under development!!!**

Project planned to be:

One simple Esp8266 mqtt interface for Wavin AHC-9000/Jablotron AC-116, with the goal of being able to control two Wavin AHC 9000 heating controllers from [OpenHAP](https://www.openhab.org) automation software for your home.

The heating controllers will still be connected to one display, from where controle can be done as well.

The project include a [Kicad](https://www.kicad.org/) PCB layout and a docker based mosquitto broker based on [eclipse-mosquitto](https://hub.docker.com/_/eclipse-mosquitto).

## **Esp8266 mqtt interface**

See Firmware markdown [here](Firmware/README.md).

On-board LED at PIN 16 added to indicate active boot up. It turns off at successful connect to Wireless Lan.

## **Mosquitto broker**

### **Requirements**

- Raspberry Pi model B v1.2. Installed with Raspberry Pi OS Lite (32-bit) <span id="a1">[[1]](#f1)</span>.
- Internet access.
- Raspberry Pi hardened <span id="a2">[[2]](#f2)</span>.
- Docker Engin and Docker Compose installed <span id="a3">[[3]](#f3)</span>.


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

Before starting the docker container, create a .env file in the same directory as the docker-compose.yaml file.

```bash
vi .env
```

Add the following contend:

 ````bash
MQTT_SERVER='mosquitto-mqtt'        # MQTT Broke's hostname or IP Address (here the `mosquitto-mqtt` docker container name is used.)
MQTT_PORT=1883                      # MQTT Broker's port number. TCP/IP port 1883 is reserved with IANA for use with MQTT. 
                                    # TCP/IP port 8883 is also registered, for using MQTT over SSL.
MQTT_CLIENT_ID='webhook-subscriber' # Name used to identify the MQTT client.
NOTIFICATION_E_MAIL=*E-mail addres* # E-mail address to which notifications will be send
# If the one or more of the following environment variables is NOT set, e-mail notificatinos will NOT be send.
MQTT_POWERUP_NOTIFICATION='YES'     # IF set: Energy Meter Powerup notifications will be send. 
MQTT_DISCONNECT_NOTIFICATION='YES'  # IF set: Disconnect notifications will be send
MQTT_RECONNECT_NOTIFICATION='YES'   # IF set: Re-connect  notifications will be send
MQTT_ALIVE_NOTIFICATION=`YES`
````

Verify that the environment variables is succesfull read by docker compose.

````bash
docker compose config
````

 Start the docker container with `docker-compose up -d`.

```bash
docker-compose up -d
```

## Project development tasks

- Verify if the sp8266 mqtt interface can controle only one of the two heat controllers.
  - Currently the sp8266 mqtt interface addresses both controllers simultaneously. Meaning, setting a target temperature, it seems to be set for both… and the current temperature are read from both controllers, resulting in two different temperatures for the same channel...

## Credits
- dkjonas/Wavin-AHC-9000-mqtt

#### **Footnotes**
=======
###### 1. <span id="f1"></span> See document: [Setting up and configure Raspberry Pi 3 MODEL B+ for the Docker Traefik router project](./docs/SettingUpRaspberryPi.md). [$\hookleftarrow$](#a1)

###### 2. <span id="f2"></span> See document [Hardening Raspberry Pi 3 MODEL B+ for the Docker Traefik router project](./docs/HardeningRaspberryPi.md)[$\hookleftarrow$](#a2)

###### 3. <span id="f3"></span> See document [Install Docker Engin and Docker Compose](./docs/InstallDockerEnginAndCompose.md)[$\hookleftarrow$](#a3)
