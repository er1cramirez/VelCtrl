#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./VelCtrl_rt
else
	EXEC=./VelCtrl_nrt
fi

$EXEC -n Drone_0 -a 127.0.0.1 -p 9000 -l ./ -x setup_x8.xml -t x8_simu
