#!/bin/sh
set -x
make clean
make && sudo make flash
echo "Press enter to quit script..."
read dummy
