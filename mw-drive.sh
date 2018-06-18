#!/bin/bash -x

# Does not need any arguments

cwd=$(pwd)
ip=$(echo $SSH_CLIENT | awk '{ print $1; }')

# source env/bin/activate

darner_dir=/disk2/ubuntu/darner

killall darner
rm -rf $darner_dir
mkdir -p $darner_dir

$HOME/src/darner/build/darner -d $darner_dir >darner.log 2>&1 &

export PYTHONPATH=$cwd

# exec python3 examples/tk1.py
python3 donkeycar/fanout.py
python3 donkeycar/parts/sensors/cameras.py &

sleep 5

python3 donkeycar/parts/datastore.py &
python3 donkeycar/parts/controllers/web.py &
python3 donkeycar/parts/sensors/teensy_rcin.py &
python3 donkeycar/parts/sensors/astar_speed.py &

exec python3 donkeycar/parts/actuators/actuators.py
