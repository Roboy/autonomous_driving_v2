# Autonomous Driving V2

## Docker

Clone this branch onto the Computer you want to deploy (should be Leia). First, if not already happened, create the docker folder (`mkdir docker`). Then, clone the Dockerfile from this repo using the command
```
git clone -b docker https://github.com/Roboy/autonomous_driving_v2.git  ./docker
```

Build  the container with the this command:
```
docker build -t adv2 .
```

In general, add `--network=host` to enable network connection via the host PC.\
For building the sensors Docker add `-d --device=/dev/ttyUSB0` to the docker run
```
sudo docker run -it -d --device=/dev/ttyUSB0 --network=host --name docker_name docker_name:latest
```
