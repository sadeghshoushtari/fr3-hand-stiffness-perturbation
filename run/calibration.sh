#!/usr/bin/env bash
# Starts the calibration session; see session.sh
exec bash "$(dirname "${BASH_SOURCE[0]}")/session.sh" calibration "$@"
