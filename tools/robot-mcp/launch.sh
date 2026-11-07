#!/bin/sh
# Starts the robot MCP server (stdio) from any working directory. The first run lets uv create the
# environment (Python 3.12 and the locked dependencies), which takes a little longer.
here="$(cd "$(dirname "$0")" && pwd)"
if ! command -v uv >/dev/null 2>&1; then
  echo "robot-mcp: uv is not installed (https://docs.astral.sh/uv/; macOS: brew install uv)" >&2
  exit 127
fi
exec uv run --quiet --directory "$here" robot-mcp "$@"
