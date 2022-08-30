#!/bin/bash


SESSION=$1
HOST_IP=$2

# nick@192.168.0.154

tmuxstart() {
    if [[ $(tmux has-session -t "$1") -eq 0 ]] ; then
        echo "Killing previous session with name $1"
        tmux kill-session -t  "$1"
    fi
    #rest of tmux script to create session named "sess"
    tmux new-session -d -s "$1"
}

splitandrun() {
    tmux send-keys -t $1 "tmux split-window -h $2 && tmux select-layout even-horizontal" ENTER
}

sendcmd() {
    tmux send-keys -t $1 "$2" ENTER
}
rostopic list >/dev/null 2>&1
if [ $? -ne 0 ] ; then
    echo "rosmaster not started! Exiting."
    exit 1
fi

IP=$(sed 's&.*@\(\)&\1&' <<< ${HOST_IP})

until ping -c1 ${IP} >/dev/null 2>&1; do 
    echo "Pinging $IP...";
done

read -p "Destination $IP reached. Press enter to begin tmux session and ssh to remote vehicle"

tmuxstart ${SESSION}

# Split panes then ssh to the vehicle in each pane
splitandrun ${SESSION} "ssh -X ${HOST_IP}"
splitandrun ${SESSION} "ssh -X ${HOST_IP}"

# ssh to the vehicle in the original pane
sendcmd 0 "ssh -tt -X ${HOST_IP}"

# Must wait, otherwise panes other than 0 may not initialize properly.
    echo "Wait for panes to fully initialize."

for COUNTDOWN in 3 2 1
do
    echo $COUNTDOWN
    sleep 1
done

sendcmd 0 "a29760116"
sendcmd 1 "a29760116"
sendcmd 2 "a29760116"

for COUNTDOWN in 3 2 1
do
    echo $COUNTDOWN
    sleep 1
done

sendcmd 0 "roslaunch mavros px4.launch"

# sendcmd 1 "rosservice call /mavros/set_stream_rate '{message_rate: 100, on_off: 1}'"
sendcmd 2 "rosrun vision cam.py"


## Create the windows on which each node or .launch file is going to run

gnome-terminal --tab -- tmux attach -t ${SESSION}

for COUNTDOWN in 5 4 3 2 1
do
    echo $COUNTDOWN
    sleep 2
done
sendcmd 1 "rosservice call /mavros/set_stream_rate '{message_rate: 100, on_off: 1}'"