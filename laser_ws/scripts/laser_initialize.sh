#!/bin/bash
sudo chmod 1 ./scripts/upgrade
sudo chmod 1 ./scripts/reset.cmd
sudo chmod a+rw /dev/ttyACM0
./scripts/upgrade /dev/ttyACM0 ./scripts/reset.cmd
