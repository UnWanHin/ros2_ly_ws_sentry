#!/bin/bash

cd ..
colcon build --symlink-install && set SUCCESS=1
if not SUCCESS; do 
	exit
done

cd ./scripts
./start_shooting_table_calib.sh


