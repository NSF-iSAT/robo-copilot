#!/bin/bash

sudo docker exec -ti $(docker ps -aq --filter ancestor=robo_copilot:latest --filter status=running) bash