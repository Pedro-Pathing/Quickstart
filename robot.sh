#!/bin/bash

# how to use?
# run:
# bash robot.sh setup

ROBOT_IP="192.168.43.1:5555"

usage() {
  echo "robot"
  echo "----------------------------"
  echo "Usage: ./robot.sh <command> | robot <command>"
  echo
  echo "Commands:"
  echo "  setup       checks and installs for adb (along with making it globally avaliable)"
  echo "  connect     connects to the Robot Hub ($ROBOT_IP)."
  echo "  disconnect  disconnects from all adb devices"
  echo "  help        shows help"
}

setup() {
  echo "🤖 Checking for adb..."
  if command -v adb &> /dev/null; then
    echo "adb installed"
  else
    echo "adb not found > installing from homebrew"
    if command -v brew &> /dev/null; then
      echo "installing..."
      brew install android-platform-tools
    else
      echo "homebrew not found :("
      echo "download it from https://brew.sh/ then try again"
      return 1
    fi

    if command -v adb &> /dev/null; then
      echo "adb installed!"
    else
      echo "something went wrong :("
      return 1
    fi
  fi

  SCRIPT_PATH="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
  chmod +x "$SCRIPT_PATH"

  echo "copying script to /usr/local/bin/robot (overwriting if exists)..."
  sudo cp "$SCRIPT_PATH" /usr/local/bin/robot
  echo "now available as 'robot' globally"

  echo "done"
}



connect() {
  echo "connecting to $ROBOT_IP..."
  adb connect $ROBOT_IP
}

disconnect() {
  echo "Disconnecting from all adb devices..."
  adb disconnect
}

if [ -z "$1" ]; then
  echo "Error: No command provided."
  usage
  exit 1
fi

COMMAND=$1

case "$COMMAND" in
  setup)
    setup
    ;;
  connect)
    connect
    ;;
  disconnect)
    disconnect
    ;;
  help)
    usage
    ;;
  *)
    echo "Error: Unknown command '$COMMAND'"
    usage
    exit 1
    ;;
esac
