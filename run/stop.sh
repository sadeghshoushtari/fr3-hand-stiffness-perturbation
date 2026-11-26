#!/usr/bin/env bash
# Ends a running session from any terminal; see session.sh
exec bash "$(dirname "${BASH_SOURCE[0]}")/session.sh" stop "$@"
