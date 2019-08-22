# Autonomous Driving V2

Clone this branch onto the Computer you want to deploy (should be Leia). First, if not already happened, create the docker folder (`mkdir docker`). Then, clone the .Dockerfiles from this repo using the command
```
git clone -b master https://github.com/Roboy/autonomous_driving_v2.git  ./docker
```

Build  the containers like so:
```
docker build -t ad-planning -f planning.Dockerfile .
docker build -t ad-slam -f slam.Dockerfile .
docker build -t ad-control -f control.Dockerfile .
docker build -t ad-sensors -f sensors.Dockerfile .
```

In general, add `--network=host` to enable network connection via the host PC.\
For building the sensors Docker add `-d --device=/dev/ttyUSB0` to the docker run
```
sudo docker run -it -d --device=/dev/ttyUSB0 --network=host --name docker_name docker_name:latest bash
```

## Booting
### Startup

To start the container:
```
sudo docker start docker_name
```
### Interaction

To enter a docker bash:
```
sudo docker exec -it docker_name bash
```

### Exiting

To exit the docker, in the docker shell type exit
```
root@ubuntu:/home/ros# exit
```

### Stopping

To stop the container:
```
sudo docker stop docker_name
```
### Useful commands:

``` sudo docker kill docker_name``` forces shutdown of docker docker_name

``` sudo docker ps``` (shows active dockers)

``` sudo docker ps -a``` (shows all dockers)

