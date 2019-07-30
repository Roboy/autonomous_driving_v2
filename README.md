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
