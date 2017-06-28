#!/bin/bash
# start stop script to run or kill all external controller

ARM_ROBOT_PID_FILE=$HOME/run/arm_robot_control.pid
DIR="$( cd "$( dirname "$0" )" && pwd )"

start() {
    echo "START: arm_robor contrller talking to you."

    python $DIR/controller.py &
    echo $! > $ARM_ROBOT_PID_FILE
}

stop() {
    echo "STOP: arm_robor contrller talking to you."
    killall -9 `cat $ARM_ROBOT_PID_FILE`
    rm -rf $ARM_ROBOT_PID_FILE
}

mode=$1

case $mode in
'start')
    start
    exit 0
    ;;
'stop')
    stop
    exit 0
    ;;
*)
    echo "Unknown mode $mode."
    exit 1
    ;;
esac
