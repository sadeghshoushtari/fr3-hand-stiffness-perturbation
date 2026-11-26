#!/usr/bin/env bash
# Starts the teleop session; see session.sh
exec bash "$(dirname "${BASH_SOURCE[0]}")/session.sh" teleop "$@"
