# Setting up and configure Raspberry Pi 3 MODEL B+ for the Docker projects

## Pre-requisuits:
    • USB Keyboard
    • Terminal kabel (HDMI → VGA)
    • LAN kabel (Internet access)
    • Raspberry Pi
    • FAST SC kort

## Setting up SD card:
Using Raspberry Pi Imanager to create SD Card installed with Raspberry Pi OS Lite (32-bit).

See: [Setting up your Raspberry Pi](https://projects.raspberrypi.org/en/projects/raspberry-pi-setting-up/2) for detailed instructions.

## Configure Raspberry Pi 3 MODEL B+ for the Docker projects

    • Connet: 
      - LAN kabel (Internet access), 
      - local keyboard and
      - terminal.
    • Take a note about LAN address at boot up.
    • Boot up and log in as:
      - User: pi
      - Passwd: raspberry

## Run Raspberry Pi configuration by executing

````bash
sudo raspi-config
````

### Update this tool to lates version - Main menu item 8

    • Select item: 8 Update this tool to latest version.

### System options - Main menu item 1:

##### As the pi will be running in a demilitarized zone, wireless network will not be configured.

    • Submenu S4 Hostname = pi-mqtt
    • Submenu S6 Network at Boot = Yes

### Interface Options - Main menu item 3:
    • Submenu P2 SSH = remote command line access using SSH = Enable.

### Perfoamance Options - Main menu item 4:
    • P4 Fan: GPIO 18 → Temp: 80.

### Localisation Options - Main menu item 5:
    • L1 Locale →  add:  da_DK-UTF-8 UTF-8
    • L2 TimeZone → Europe → Copenhagen.
    • L3 Keyboard → Generid 105-key PC (intl) → Danish (With Win keys) → default → No comppose key.

### Advanced Options - Main menu item 6:
    • A1 Expand Filesystem.

## **Exit raspi-config and reboot** -> Login and verify IP addresses

<br>

# From here on - Connect via SSH with PUTTY
## Update OS: 
```bash
 sudo apt-get update
 sudo apt-get -y upgrade –fix-missing
 sudo reboot now

```
