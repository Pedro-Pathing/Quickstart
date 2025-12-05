#!/bin/bash
# run: bash robot.sh setup
ROBOT_IP="192.168.43.1:5555"

usage() {
  echo "robot"
  echo "----------------------------"
  echo "Usage: ./robot.sh <command> | robot <command>"
  echo
  echo "Commands:"
  echo "  setup       installs homebrew (if needed), installs adb, and makes script global"
  echo "  connect     connects to the Robot Hub ($ROBOT_IP) and enforces hardwaremap"
  echo "  disconnect  disconnects from all adb devices"
  echo "  help        shows help"
}

setup() {
  echo "checking homebrew..."
  if command -v brew &> /dev/null; then
    echo "homebrew installed"
  else
    echo "homebrew not found > installing homebrew"

    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

    # shellcheck disable=SC2016
    echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
    eval "$(/opt/homebrew/bin/brew shellenv)"

    if command -v brew &> /dev/null; then
      echo "hombrew installed!"
    else
      echo "homebrew installation failed :("
      return 1
    fi
  fi

  echo "checking for adb"
  if command -v adb &> /dev/null; then
    echo "adb installed"
  else
    echo "adb not found > installing android-platform-tools..."
    brew install android-platform-tools
  fi

  if ! command -v adb &> /dev/null; then
    echo "adb failed to install :("
    return 1
  fi

  SCRIPT_PATH="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
  chmod +x "$SCRIPT_PATH"

  TARGET_BIN=""

  if [[ "$(uname -m)" == "arm64" ]]; then
    TARGET_BIN="/opt/homebrew/bin"
  else
    TARGET_BIN="/usr/local/bin"
  fi

  echo "copying script to /usr/local/bin/robot (overwriting if exists)..."
  sudo cp "$SCRIPT_PATH" "$TARGET_BIN/robot"
  echo "now available as 'robot' globally"

  echo "done"
}

connect() {
  echo "connecting to $ROBOT_IP..."
  adb connect $ROBOT_IP

  if [ $? -ne 0 ]; then
    echo "failed to connect to $ROBOT_IP"
    return 1
  fi

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
