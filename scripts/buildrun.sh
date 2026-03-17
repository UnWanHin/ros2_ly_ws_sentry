#!/bin/bash

cd ..
colcon build --symlink-install || exit
cd ./scripts
./start_competition_autoaim_test.sh

