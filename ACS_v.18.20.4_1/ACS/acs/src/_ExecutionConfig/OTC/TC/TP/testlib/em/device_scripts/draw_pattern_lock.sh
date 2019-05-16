
N=10
dev=$1
x1=$2
y1=$3
x2=$4
y2=$5

x=$x1
y=$y1
dx=$((($x2-$x1)/$N))
dy=$((($y2-$y1)/$N))

sendevent /dev/input/event$1 3 47 0
sendevent /dev/input/event$1 3 57 53

sendevent /dev/input/event$1 3 57 0
sendevent /dev/input/event$1 3 53 $x
sendevent /dev/input/event$1 3 54 $y
sendevent /dev/input/event$1 1 330 1
sendevent /dev/input/event$1 0 0 0

i=0
while [ $y -lt $y2 ]
do
    sendevent /dev/input/event$1 3 54 $y
    sendevent /dev/input/event$1 0 0 0
    y=$(($y+$dy))
    #i=$(($i+1))
done
sendevent /dev/input/event$1 3 54 $y2
sendevent /dev/input/event$1 0 0 0

i=0
while [ $x -lt $x2 ]
do
    sendevent /dev/input/event$1 3 53 $x
    sendevent /dev/input/event$1 0 0 0
    x=$(($x+$dx))
    #i=$(($i+1))
done
sendevent /dev/input/event$1 3 53 $x2
sendevent /dev/input/event$1 0 0 0

sendevent /dev/input/event$1 3 57 4294967295
sendevent /dev/input/event$1 1 330 0
sendevent /dev/input/event$1 0 0 0

