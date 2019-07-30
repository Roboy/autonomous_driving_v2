# Autonomous Driving V2

Clone this branch onto the Computer you want to deploy (should be Leia). First, if not already happened, create the docker folder (`mkdir docker`). Then, clone the .Dockerfiles from this repo using the command
```
git clone https://github.com/Roboy/autonomous_driving_v2.git  ./docker
```

Build  the containers like so:
```
docker build -t ad-planning -f planning.Dockerfile .
docker build -t ad-slam -f slam.Dockerfile .
```
