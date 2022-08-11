#!/bin/bash

sudo docker exec -ti $(sudo docker ps -aq --filter ancestor=robo_copilot:latest --filter status=running) bash