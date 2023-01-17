#!/bin/bash
"$pwd" = "x"
make all
echo "$pwd" | sudo make install

linuxcnc  -v /home/marian/linuxcnc/configs/ecat3/ecat.ini 

