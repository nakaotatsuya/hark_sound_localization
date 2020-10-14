#!/bin/sh

TF=`rospack find hark_sound_source_localization`/networks/TF/tamago_rectf.zip
DEVICE=plughw:1,0

echo ""
echo "Location of your TF file : $TF"
echo ""
echo "ALSA Audio Device ID : $DEVICE"
echo ""

rosrun hark_sound_source_localization localization_ROS_microcone_test.n ${TF} ${DEVICE}
