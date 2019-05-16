#! /bash/sh
event_num=$1
ave_x=$2
ave_y=$3
scale_x=$4
scale_y=$5

#untouch
sendevent /dev/input/event$event_num 3 57 4294967295
sendevent /dev/input/event$event_num 3 47 0
sendevent /dev/input/event$event_num 3 57 0
sendevent /dev/input/event$event_num 1 330 0
sendevent /dev/input/event$event_num 0 0 0
