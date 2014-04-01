#!/bin/bash
HOST=`hostname`
[ $ROS_HOSTNAME != "" ]; HOST=$ROS_HOSTNAME
[ $ROS_IP != "" ]; HOST=$ROS_IP
PORT=11311
echo "Generate QR code for http://$HOST:$PORT/"
wget "http://chart.apis.google.com/chart?cht=qr&chs=350x350&chl=URL:+http://$HOST:$PORT/" -O /tmp/qrcode.png -q
gnome-open /tmp/qrcode.png