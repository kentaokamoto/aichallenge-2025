#!/bin/bash

# Help function to display usage
usage() {
    echo "Usage: $0 [OPTION]"
    echo "Options:"
    echo "  screen      Capture screen via service call"
    echo "  control     Request control mode change"
    echo "  initial     Set initial pose"
    echo "  all         Execute all commands in sequence"
    echo "  help        Display this help message"
    exit 1
}

# Function to capture screen
capture_screen() {
    echo "Capturing screen..."
    ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger >/dev/null
    echo "Screen capture requested"
}

# Function to request control mode
request_control() {
    echo "Requesting control mode change..."
    ros2 service call /control/control_mode_request autoware_auto_vehicle_msgs/srv/ControlModeCommand '{mode: 1}' >/dev/null
    echo "Control mode change requested"
}

# Function to set initial pose
# Assignment 1 set correct initial pose
#            x: 89633.29,
#            y: 43127.57,
#            z: 0.8778,
#            w: 0.4788
set_initial_pose() {
    echo "Setting initial pose..."
    ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{ 
      header: {
        frame_id: 'map'
      },
      pose: {
        pose: {
          position: {
            x: 89634.00,
            y: 43129.00,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.8000,
            w: 0.4000
          }
        }
      }
    }" >/dev/null
    echo "Initial pose set successfully"
    sleep 1
}
check_awsim() {
    # /awsim/control_cmd トピックが現れるまで待つ
    while ! ros2 topic list | grep -q "/awsim/control_cmd"; do
        sleep 0.5
        echo "Waiting for /awsim/control_cmd topic to be available..."
    done
    # 型情報が取得できるまで待つ
    while ! ros2 topic info /awsim/control_cmd 2>/dev/null | grep -q "Type:"; do
        sleep 0.5
        echo "Waiting for /awsim/control_cmd topic type to be available..."
    done
    sleep 1
    echo "System is ready, executing publish commands..."
}

check_capture() {
    # Start recording rviz2
    echo "Check if screen capture is ready"
    until (ros2 service type /debug/service/capture_screen >/dev/null); do
        sleep 5
        echo "Check if screen capture is not ready"
    done
}

# Check if an argument was provided
if [ $# -eq 0 ]; then
    usage
fi

# Process based on provided argument
case "$1" in
check)
    check_capture
    check_awsim
    ;;
screen)
    capture_screen
    ;;
control)
    request_control
    ;;
initial)
    set_initial_pose
    ;;
all)
    sleep 5
    set_initial_pose
    request_control
    ;;
help)
    usage
    ;;
*)
    echo "Error: Invalid option '$1'"
    usage
    ;;
esac

exit 0
