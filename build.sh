#!/bin/bash
mkdir -p atlgnd
mkdir -p smogpgnd

mkdir -p atlgnd/download
mkdir -p atlgnd/packets
mkdir -p atlgnd/partial
mkdir -p atlgnd/spectrum
mkdir -p atlgnd/json

mkdir -p smogpgnd/download
mkdir -p smogpgnd/packets
mkdir -p smogpgnd/partial
mkdir -p smogpgnd/spectrum
mkdir -p smogpgnd/json

gcc -D BUILD_SMOGP main.c lib/*.c ./ts_radecoder/*.c ./ao40/long/*.c ./ao40/short/*.c -o smogpgnd/smogpgnd -lm -pthread -lrt -O3 -std=gnu90 #-Wall
gcc -D BUILD_ATL1  main.c lib/*.c ./ts_radecoder/*.c ./ao40/long/*.c ./ao40/short/*.c -o atlgnd/atlgnd     -lm -pthread -lrt -O3 -std=gnu90 #-Wall
gcc packet_looter.c -o packet_looter -O3

cp packet_looter atlgnd/
mv packet_looter smogpgnd/


