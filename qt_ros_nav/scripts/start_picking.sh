#!/bin/bash
LOG=/tmp/picking_system.log
echo "[start_picking.sh] === NEW START $(date) ===" > "$LOG"
echo "[start_picking.sh] Args: $@" >> "$LOG"
echo "[start_picking.sh] PID=$$ PPID=$PPID" >> "$LOG"
exec 2>> "$LOG"

trap 'echo "[start_picking.sh] TRAPPED signal (code=$?)" >> "$LOG"' EXIT TERM INT HUP

source /opt/ros/noetic/setup.bash >> "$LOG" 2>&1
source /home/y/dual_rokae_ws/devel/setup.bash >> "$LOG" 2>&1

SERIAL_PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"

if [ -e "$SERIAL_PORT" ]; then
    rosrun comm serial _port:="$SERIAL_PORT" _baudrate:="$BAUD" >/tmp/comm_serial.log 2>&1 &
    echo "[start_picking.sh] comm serial PID=$!" >> "$LOG"
else
    echo "[start_picking.sh] SKIP comm serial: $SERIAL_PORT not found" >> "$LOG"
fi

rosrun rokae right_robot_control_node >/tmp/right_robot_control_node.log 2>&1 &
echo "[start_picking.sh] right_robot_control_node PID=$!" >> "$LOG"

rosrun rokae left_robot_control_node >/tmp/left_robot_control_node.log 2>&1 &
echo "[start_picking.sh] left_robot_control_node PID=$!" >> "$LOG"

rosrun task_assign target_assign >/tmp/target_assign.log 2>&1 &
echo "[start_picking.sh] target_assign PID=$!" >> "$LOG"

rosrun img_detect advance_vision_node >/tmp/advance_vision_node.log 2>&1 &
echo "[start_picking.sh] advance_vision_node PID=$!" >> "$LOG"

echo "[start_picking.sh] All launched, entering wait..." >> "$LOG"
wait
echo "[start_picking.sh] wait returned, code=$?" >> "$LOG"
