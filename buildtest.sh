#!/bin/sh
gcc -L/usr/lib/arm-linux-gnueabihf/libiio.so -o pluto-tx-fm tx-fm.c wiggle.c -liio -lm

