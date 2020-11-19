#!/bin/bash
# Script for mapping the device serial port /dev/ttyUSBx to /dev/radariq


# The MIT License
#
# Copyright (c) 2020 RadarIQ Limited https://radariq.io
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

rules=$'KERNEL=="ttyACM*", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="0ed5", MODE:="0777", SYMLINK+="radariq"'

echo "This tool will install/unistall the Linux udev rules to make the RadarIQ module appear as /dev/radariq when plugged in."
PS3='Please enter your choice: '
options=("Install" "Uninstall" "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Install")
            echo "Mapping the device serial port /dev/ttyUSBx to /dev/radariq"
            echo $rules | sudo tee  /etc/udev/rules.d/radariq.rules > /dev/null
            echo ""
            echo "Reloading udev"
            echo ""
            sudo service udev reload
            echo "Complete"
            break
            ;;
        "Uninstall")
            echo "Removing the device serial port mapping of /dev/ttyUSBx to /dev/radariq"
            sudo rm /etc/udev/rules.d/radariq.rules
            echo ""
            echo "Reloading udev"
            echo ""
            sudo service udev reload
            echo "Complete"
            break
            ;;
        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done