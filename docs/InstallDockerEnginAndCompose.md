# Install Docker Engin and Docker Compose

## **Log in as routermgr and run the commands below:**

## remove any previously docker installation.
```bash
sudo apt-get remove docker docker-engine docker.io containerd runc
```


## Get docker engine and run install schript
```bash
curl -sSL https://get.docker.com | sh
```

## Enable user ”routermgr” to run docker withour using sudo
```bash
sudo usermod -aG docker routermgr
```

## **Log out and log in again to run docker commands withour sudo**
```bash
exit
```

## Test docker installation
```bash
docker run hello-world
```
If the installation of docker is successful, then message **Hello from Docker!** will show up.

## Clean up docker to verify docker-compose
```bash
docker rm -f  $(docker ps -aq)
```
```bash
docker rmi $(docker images -aq)
```

## Install dependincies for installing docker compose
```bash
sudo apt-get install -y libffi-dev libssl-dev
```
```bash
sudo apt-get install -y python3 python3-pip
```
```bash
sudo apt-get remove python-configparser

```

## Install docker compose
```bash
sudo pip3 -v install docker-compose
```

## Test docker-compose:
On Linux host create file **docker-compose.yml** and adde the following lines:
```bash
version: '3.4'
services:
  hello-world:
    image: hello-world
```
PS: If using **vi**, the folowing commands are usefull:
```bash
vi docker-compose.yml   # Will open the vi editor
a                       # for "add" will place the editor in insermode and add characters into the editor window
[ESC]                   # Exit wil exit insertmode mode. 
:[Ctrl-V][Ctrl-Q]       # Will: enter command line (:), Write to file [Ctrl-V] and exit vi [Ctrl-V][Ctrl-Q].
```

## Next: run the docker-compose command to verify the installation
```bash
docker-compose up
```

   If the installation of docker is successful, then message **Hello from Docker!** will show up.

## Clean up docker environment.
```bash
docker rm -f  $(docker ps -aq)
```
```bash
docker rmi $(docker images -aq)
```
```bash
docker system prune -f –volumes
```
```bash
rm docker-compose.yml
```

