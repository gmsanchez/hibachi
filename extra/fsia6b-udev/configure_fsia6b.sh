#!/bin/bash

modprobe serio
modprobe fsia6b
sleep 5
inputattach --fsia6b /dev/ttyFLYSKY
chown root:input /dev/input/js0
chmod 660 /dev/input/js0
