# **Wavin-AHC-9000-mqtt**

Git clone of https://github.com/dkjonas/Wavin-AHC-9000-mqtt.git but restructured for Firmware (FW), Software (SW) and Hardware (HW).

## **Currently under development!!!**

Project planned to be:

A simple Esp8266 mqtt interface for Wavin AHC-9000/Jablotron AC-116, with the goal of being able to control this heating controller from [OpenHAP](https://www.openhab.org) automation software for your home.

The project include a [Kicad](https://www.kicad.org/) PCB layout and a mosquitto broker based on [eclipse-mosquitto](https://hub.docker.com/_/eclipse-mosquitto).

## **Esp8266 mqtt interface**

See Firmware markdown [here](Firmware/README.md).

## **Mosquitto broker**

## **Requirements**

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

## Credits
- dkjonas/Wavin-AHC-9000-mqtt

#### **Footnotes**
=======
###### 1. <span id="f1"></span> See document: [Setting up and configure Raspberry Pi 3 MODEL B+ for the Docker Traefik router project](./docs/SettingUpRaspberryPi.md). [$\hookleftarrow$](#a1)

###### 2. <span id="f2"></span> See document [Hardening Raspberry Pi 3 MODEL B+ for the Docker Traefik router project](./docs/HardeningRaspberryPi.md)[$\hookleftarrow$](#a2)

###### 3. <span id="f3"></span> See document [Install Docker Engin and Docker Compose](./docs/InstallDockerEnginAndCompose.md)[$\hookleftarrow$](#a3)
