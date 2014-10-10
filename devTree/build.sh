#!/bin/bash

echo "Compiling the overlay from .dts to .dtbo"

dtc -O dtb -o PM-PWM-Test-00A0.dtbo -b 0 -@ PM-PWM-Test-00A0.dts
