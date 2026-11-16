#!/bin/bash
# Makes the short commands available in every terminal, run once:
#   sensors       USB bridge (board -> MOSAIC port 50500, vibrator commands on port 50601)
#   perturbation  perturbation program, robot 169.254.105.210, MOSAIC mode (built first if needed)
#   mosaic        MOSAIC from <thesis folder>/mosaic-2.0
# They are links in ~/.local/bin to the scripts in this folder (so a git pull updates them).
set -e
here=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
mkdir -p "$HOME/.local/bin"
for pair in sensors:sensors.sh perturbation:perturbation.sh mosaic:mosaic.sh; do
  name=${pair%%:*}
  other=$(type -P "$name" || true)
  if [ -n "$other" ] && [ "$(readlink -f "$other")" != "$here/${pair#*:}" ]; then
    echo "note: '$name' also exists as $other; the new one in ~/.local/bin is used first"
  fi
  ln -sf "$here/${pair#*:}" "$HOME/.local/bin/$name"
done
if ! grep -q 'HOME/.local/bin' "$HOME/.bashrc" 2>/dev/null; then
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
fi
echo "done: open a new terminal, then type sensors, perturbation or mosaic"
