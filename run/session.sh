#!/usr/bin/env bash
# Calibration and teleoperation sessions on the robot PC, from one terminal.
#
#   run/calibration.sh [--dry-run] [--test]
#   run/teleop.sh [--dry-run] [--test] [--logger <label>]
#   run/stop.sh            ends a running session from any terminal (same shutdown as q in the control pane)
#
# 1. Checks: both repositories on their GitHub branch without local edits (offers the reset), perturbation /
#    slave_ti / master_ti built from the current sources and MOSAIC built with every block of this repository (builds
#    them if not), robot reachable, board port present and free, UDP ports free, no program of an earlier session
#    still running; teleoperation also the model file (format, date, calibration verdicts).
# 2. Starts everything in the tmux session "thesis", one pane per program, each pane logged to the session folder
#    <thesis>/sessions/<date_time>_<kind>/ (with the git commits, the robot and sensor logs and the model file).
# 3. The control pane (top left) lists the steps. q + Enter there ends the session: robot programs stopped, vibrator
#    off (also sent to the board directly), MOSAIC closed, results and logs copied, summary printed. The same shutdown
#    runs on Ctrl+C in the control pane, when the tmux session is closed, and with run/stop.sh.
#
# --dry-run: the checks only, nothing changed or started. --test: on a PC without robot and board (stand-in programs,
# no checks or builds), to try the panes and the shutdown. Environment: THESIS_DIR, ROBOT_IP, BOARD_PORT, DOTNET,
# SESSIONS_DIR override the defaults below.
set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CAL=$(cd "$HERE/.." && pwd)
THESIS=${THESIS_DIR:-$(cd "$CAL/.." && pwd)}
TI=$THESIS/tele-impedance
MOSAIC=$THESIS/mosaic-2.0/MOSAIC
MODELS=$THESIS/mosaic_models
MLOGS=$THESIS/mosaic_logs
ROBOT_IP=${ROBOT_IP:-169.254.105.210}
BOARD=${BOARD_PORT:-/dev/ttyUSB0}
DOTNET=${DOTNET:-$HOME/.dotnet/dotnet}
[ -x "$DOTNET" ] || DOTNET=$(command -v dotnet || echo dotnet)
TS=thesis  # tmux session

KIND=${1:-}
shift || true
DRY=false
TEST=false
LOGGER=""
while [ $# -gt 0 ]; do
  case $1 in
    --dry-run) DRY=true ;;
    --test) TEST=true ;;
    --logger) LOGGER=${2:?--logger needs a label}; shift ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
if $TEST; then
  SESSIONS=${SESSIONS_DIR:-${TMPDIR:-/tmp}/thesis_sessions_test}
else
  SESSIONS=${SESSIONS_DIR:-$THESIS/sessions}
fi
CURRENT=$SESSIONS/.current  # "kind folder" of the running session

FAILS=0
ok() { printf '  \033[32mok\033[0m    %s\n' "$*"; }
warn() { printf '  \033[33mwarn\033[0m  %s\n' "$*"; }
bad() { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; FAILS=$((FAILS + 1)); }
ask() {
  if $DRY; then echo "        (dry run: would ask \"$1\")"; return 1; fi
  local a
  read -r -p "        $1 [y/N] " a
  [[ $a == [yY]* ]]
}

# processes of our programs, by command line (the repository path contains "perturbation", so match the binary)
PATTERNS=('sensors\.py' '(^|/)perturbation( |$)' '(^|/)slave_ti( |$)' '(^|/)master_ti( |$)' 'MOSAIC\.Desktop'
  'tank_logger\.py' 'teleop_logger\.py' 'stiffness_sender\.py')
ours() {
  local p
  for p in "${PATTERNS[@]}"; do pgrep -af "$p"; done | grep -v -e "session\.sh" -e "pgrep" | sort -u
}

# --- checks ---

check_tools() {
  local t missing=()
  for t in tmux git g++ python3 ping ss; do command -v "$t" >/dev/null || missing+=("$t"); done
  if [ ${#missing[@]} -eq 0 ]; then ok "tools: tmux, git, g++, python3, ping, ss"; else bad "missing: ${missing[*]}"; fi
}

check_repo() {  # dir branch name
  local dir=$1 branch=$2 name=$3
  if ! git -C "$dir" rev-parse --git-dir >/dev/null 2>&1; then bad "$name: $dir is not a git clone"; return; fi
  local cur
  cur=$(git -C "$dir" rev-parse --abbrev-ref HEAD)
  if [ "$cur" != "$branch" ]; then bad "$name: on branch $cur, expected $branch"; return; fi
  timeout 20 git -C "$dir" fetch -q origin "$branch" 2>/dev/null || warn "$name: GitHub not reachable, local state only"
  local dirty head remote
  dirty=$(git -C "$dir" status --porcelain --untracked-files=no)
  head=$(git -C "$dir" rev-parse HEAD)
  remote=$(git -C "$dir" rev-parse "origin/$branch" 2>/dev/null || echo "$head")
  if [ -z "$dirty" ] && [ "$head" = "$remote" ]; then
    ok "$name: $branch at ${head:0:7}, same as GitHub, no local edits"
    return
  fi
  [ -n "$dirty" ] && warn "$name: local edits in $(echo "$dirty" | awk '{print $2}' | tr '\n' ' ')"
  [ "$head" != "$remote" ] && warn "$name: ${head:0:7} is not GitHub's ${remote:0:7}"
  if ask "reset $name to GitHub (origin/$branch)? Local edits are lost"; then
    if git -C "$dir" reset -q --hard "origin/$branch"; then ok "$name: reset to ${remote:0:7}"; else bad "$name: reset failed"; fi
  else
    warn "$name: running with the local state"
  fi
}

newest_mtime() { find "$@" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cs' \) -printf '%T@\n' 2>/dev/null | sort -n | tail -1 | cut -d. -f1; }

check_binary() {  # name dir binary "build command" sources...
  local name=$1 dir=$2 bin=$3 cmd=$4
  shift 4
  local src
  src=$(newest_mtime "$@")
  if [ -x "$dir/$bin" ] && [ "$(stat -c %Y "$dir/$bin")" -ge "${src:-0}" ]; then
    ok "$name: built from the current sources"
    return
  fi
  warn "$name: $([ -x "$dir/$bin" ] && echo "older than its sources" || echo "not built")"
  if $DRY; then echo "        (dry run: would build in $dir: $cmd)"; return; fi
  echo "        building $name ..."
  if (cd "$dir" && eval "$cmd") >"$BUILDLOG/$bin.log" 2>&1; then ok "$name: built"; else bad "$name: build failed, see $BUILDLOG/$bin.log"; fi
}

mosaic_tfm() {  # the framework the VS Code build task uses (net10.0) if MOSAIC lists it, else its first one
  local t
  t=$(grep -o 'net[0-9][0-9.]*' "$MOSAIC/MOSAIC.Desktop/MOSAIC.Desktop.csproj" 2>/dev/null | sort -u)
  if echo "$t" | grep -qx net10.0; then echo net10.0; else grep -o 'net[0-9][0-9.]*' "$MOSAIC/MOSAIC.Desktop/MOSAIC.Desktop.csproj" 2>/dev/null | head -1; fi
}
dotnet_env() { case $DOTNET in /*) echo "DOTNET_ROOT=$(dirname "$DOTNET")" ;; esac; }

check_mosaic() {
  local el=$MOSAIC/MOSAIC/Models/Elastography f t missing=() changed=()
  if [ ! -d "$el" ]; then bad "MOSAIC: $el missing (mosaic/blocks/README.md, install)"; return; fi
  for f in "$CAL"/mosaic/blocks/*.cs; do
    t=$el/$(basename "$f")
    if [ ! -e "$t" ]; then missing+=("$(basename "$f")"); elif ! cmp -s "$f" "$t"; then changed+=("$(basename "$f")"); fi
  done
  local factory=$MOSAIC/MOSAIC/Components/Factory/BlockFactory.cs
  if grep -q '"perturbationcalibration"' "$factory" 2>/dev/null && grep -q '"stiffnessprojection"' "$factory"; then
    ok "MOSAIC: both blocks registered in BlockFactory.cs"
  else
    bad "MOSAIC: blocks not registered in BlockFactory.cs (mosaic/blocks/README.md, step 2)"
  fi
  local dll src tfm
  tfm=$(mosaic_tfm)
  dll=$(find "$MOSAIC/MOSAIC.Desktop/bin" -name MOSAIC.dll -path "*$tfm*" -printf '%T@\n' 2>/dev/null | sort -n | tail -1 | cut -d. -f1)
  src=$(find -L "$el" -name '*.cs' -printf '%T@\n' 2>/dev/null | sort -n | tail -1 | cut -d. -f1)
  if [ ${#missing[@]} -eq 0 ] && [ ${#changed[@]} -eq 0 ] && [ -n "$dll" ] && [ "$dll" -ge "${src:-0}" ]; then
    ok "MOSAIC: all $(ls "$CAL"/mosaic/blocks/*.cs | wc -l) blocks of this repository installed, build ($tfm) is newer"
    return
  fi
  [ ${#missing[@]} -gt 0 ] && warn "MOSAIC: blocks not installed: ${missing[*]}"
  [ ${#changed[@]} -gt 0 ] && warn "MOSAIC: installed blocks differ from the repository: ${changed[*]}"
  [ -z "$dll" ] && warn "MOSAIC: no build for $tfm found"
  [ -n "$dll" ] && [ "$dll" -lt "${src:-0}" ] && warn "MOSAIC: build older than the blocks"
  local cmd="ln -sf $CAL/mosaic/blocks/*.cs MOSAIC/Models/Elastography/ && $(dotnet_env) $DOTNET build MOSAIC.Desktop/MOSAIC.Desktop.csproj -f $tfm"
  if $DRY; then echo "        (dry run: would run in $MOSAIC: $cmd)"; return; fi
  echo "        linking the blocks and building MOSAIC ($tfm), about a minute ..."
  if (cd "$MOSAIC" && eval "$cmd") >"$BUILDLOG/mosaic.log" 2>&1; then ok "MOSAIC: built"; else bad "MOSAIC: build failed, see $BUILDLOG/mosaic.log"; fi
}

check_robot() {
  if ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1; then ok "robot $ROBOT_IP answers"; else bad "robot $ROBOT_IP does not answer (cable, robot on?)"; fi
}

check_board() {
  if [ ! -e "$BOARD" ]; then
    local found
    found=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)
    if [ "$(echo "$found" | grep -c .)" -eq 1 ]; then
      warn "board: $BOARD missing, using $found"
      BOARD=$found
    else
      bad "board: no serial port ($BOARD missing; found: ${found:-none}). USB cable plugged in?"
      return
    fi
  fi
  if [ ! -r "$BOARD" ] || [ ! -w "$BOARD" ]; then bad "board: no access to $BOARD (user in group dialout?)"; return; fi
  local users
  users=$(fuser "$BOARD" 2>/dev/null | xargs)
  if [ -n "$users" ]; then bad "board: $BOARD is open in process $users ($(ps -o comm= -p "${users// /,}" | xargs))"; return; fi
  ok "board: $BOARD present and free"
}

check_leftovers() {
  local left
  left=$(ours)
  if [ -z "$left" ]; then ok "no program of an earlier session running"; return; fi
  warn "still running from an earlier session:"
  echo "$left" | sed 's/^/          /'
  if ask "stop them (robot programs and vibrator are stopped first)?"; then
    stop_leftovers
    if [ -z "$(ours)" ]; then ok "stopped"; else bad "could not stop: $(ours | xargs)"; fi
  else
    bad "programs of an earlier session still running"
  fi
}

check_ports() {  # ports...
  local p busy=()
  for p in "$@"; do
    ss -Hulnp 2>/dev/null | awk '{print $4, $0}' | grep -qE "[:.]$p " && busy+=("$p")
  done
  if [ ${#busy[@]} -eq 0 ]; then ok "UDP ports free: $*"; return; fi
  bad "UDP ports in use: ${busy[*]}"
  for p in "${busy[@]}"; do ss -Hulnp 2>/dev/null | grep -E "[:.]$p " | sed 's/^/          /'; done
}

check_model() {
  local model=$MODELS/stiffness_projection.json
  if [ ! -f "$model" ]; then bad "model file $model missing: run the calibration first"; return; fi
  python3 - "$model" "$MLOGS/calibration" <<'EOF'
import glob, json, os, sys, time
m = json.load(open(sys.argv[1]))
fit = m.get("fit") or {}
age_h = (time.time() - os.path.getmtime(sys.argv[1])) / 3600
fmt = m.get("format", "?")
off = (m.get("measurement") or {}).get("handle_offset_m")
print(f"MODEL {fmt} | fitted {fit.get('date', '?')} ({age_h:.1f} h ago) | model {fit.get('model', 'groups')}, reduction "
      f"{m.get('reduction', '-')}, handle offset {off} m")
sessions = sorted(glob.glob(os.path.join(sys.argv[2], "calibration_*.json")), key=os.path.getmtime)
if sessions:
    s = json.load(open(sessions[-1]))
    chk = (s.get("check") or {}).get("verdict", "none")
    val = (s.get("validation_check") or {}).get("result", "none")
    print(f"SESSION {os.path.basename(sessions[-1])}: check {chk}, validation {val}")
print(f"AGE {age_h:.1f}")
print(f"FORMAT {fmt}")
EOF
}

report_model() {
  local out line
  out=$(check_model) || { bad "model file unreadable"; return; }
  [ -z "$out" ] && return
  while IFS= read -r line; do
    case $line in
      MODEL*) ok "model: ${line#MODEL }" ;;
      SESSION*) ok "last calibration ${line#SESSION }" ;;
      AGE*) awk -v a="${line#AGE }" 'BEGIN{exit !(a > 6)}' && warn "model older than 6 h: calibrate again if the sensors were taken off since" ;;
      FORMAT*) [ "${line#FORMAT }" = "stiffness_projection/3" ] || warn "model is ${line#FORMAT }, not the hand model (format 3): calibrate again" ;;
    esac
  done <<<"$out"
  echo "$out" | grep -q "validation PASS" || warn "the last calibration has no VALIDATION PASS"
}

# --- stopping ---

stop_leftovers() {  # robot programs first, then the rest; INT, then TERM
  local p sig
  for sig in INT INT TERM; do
    for p in '(^|/)master_ti( |$)' '(^|/)slave_ti( |$)' '(^|/)perturbation( |$)' 'sensors\.py' 'MOSAIC\.Desktop' \
      'stiffness_sender\.py' 'tank_logger\.py' 'teleop_logger\.py'; do
      pkill -"$sig" -f "$p" 2>/dev/null
    done
    sleep 2
    [ -z "$(ours)" ] && break
  done
  board_stop
}

board_stop() {  # vibrator off on the board itself, in case sensors.py did not get to it
  $TEST && return
  [ -e "$BOARD" ] || return
  [ -n "$(fuser "$BOARD" 2>/dev/null)" ] && return
  stty -F "$BOARD" 115200 raw -echo 2>/dev/null && printf 'stop\n' >"$BOARD" 2>/dev/null && echo "  vibrator: stop sent to $BOARD"
}

# panes carry their name in the pane option @name (programs can change the title, not this)
pid_of() { tmux list-panes -s -t "$TS" -F '#{pane_id} #{@name}' 2>/dev/null | awk -v n="$1" '$2 == n {print $1; exit}'; }
pane_busy() {  # a program runs in the pane (its shell is not in the foreground)
  local id
  id=$(pid_of "$1")
  [ -n "$id" ] && [ "$(tmux display -p -t "$id" '#{pane_current_command}' 2>/dev/null)" != "bash" ]
}

stop_pane() {  # name q|C-c seconds. q: "q" + Enter (sensors.py only). Never Enter to a robot program: before its
  # home move it waits for Enter and would start the move. If the program does not end in time, its process gets TERM.
  local name=$1 keys=$2 wait=${3:-10} id i
  id=$(pid_of "$name")
  [ -n "$id" ] || return
  pane_busy "$name" || return
  if [ "$keys" = q ]; then tmux send-keys -t "$id" q Enter; else tmux send-keys -t "$id" C-c; fi
  for ((i = 0; i < wait * 5; i++)); do pane_busy "$name" || return; sleep 0.2; done
  echo "  $name did not end in ${wait} s: TERM"
  pkill -TERM -P "$(tmux display -p -t "$id" '#{pane_pid}')" 2>/dev/null
  sleep 1
}

collect() {  # kind folder
  local kind=$1 dir=$2
  mkdir -p "$dir/mosaic_logs" "$dir/model"
  if [ -d "$MLOGS" ]; then
    (cd "$MLOGS" && find . -type f -newer "$dir/.start" -exec cp -p --parents {} "$dir/mosaic_logs/" \;) 2>/dev/null
  fi
  cp -p "$MODELS/stiffness_projection.json" "$dir/model/after.json" 2>/dev/null
  {
    echo "session $(basename "$dir"), ended $(date '+%Y-%m-%d %H:%M:%S')"
    echo "folder: $dir"
    if [ "$kind" = calibration ] && [ -f "$dir/panes/mosaic.log" ]; then
      echo
      echo "calibration (from the MOSAIC output):"
      sed 's/\x1b\[[0-9;]*[a-zA-Z]//g' "$dir/panes/mosaic.log" | grep -E "CHECK|VERDICT|VALIDATION|fit: (a0|hand|translation command)|model written" | sed 's/^/  /'
    fi
    echo
    echo "files:"
    (cd "$dir" && find . -type f ! -name '.start' | sort | sed 's/^\.\//  /')
    echo
    echo "next: entry in Thesis Report/notes/experiments.md (data copy + scripts), see session.md here"
  } >"$dir/summary.txt"
}

shutdown_session() {  # kind folder
  local kind=$1 dir=$2
  echo
  echo "ending the $kind session ..."
  if [ "$kind" = calibration ]; then
    stop_pane robot C-c 15
    stop_pane sensors q
    stop_pane mosaic C-c
  else
    stop_pane master C-c
    stop_pane slave C-c 15
    stop_pane mosaic C-c
    stop_pane sensors q
    stop_pane slave_tank C-c
    stop_pane master_tank C-c
    stop_pane teleop_logger C-c
  fi
  $TEST || { [ -n "$(ours)" ] && { echo "  stopping what is left: $(ours | awk '{print $2}' | xargs)"; stop_leftovers; }; }
  board_stop
  sleep 0.5
  tmux list-panes -s -t "$TS" -F '#{pane_id}' 2>/dev/null | while read -r p; do tmux pipe-pane -t "$p"; done
  sleep 0.5
  collect "$kind" "$dir"
  rm -f "$CURRENT"
  echo "  done; summary in $dir/summary.txt"
}

# --- control pane ---

checklist() {
  local file
  if [ "$1" = calibration ]; then
    file=$CAL/mosaic/session_calibration.json
    cat <<EOF
CALIBRATION   (panes: sensors below, robot right, MOSAIC below it; click a pane to type in it)
 Before: bench check on the laptop ended with VERDICT: BOTH PASS; Desk unlocked, FCI active; emergency stop in reach.
 1. MOSAIC window: File -> Open -> $file
 2. robot pane: hands off the handle, press Enter (home move); wait for "Ready".
 3. MOSAIC Vibrator card: All, Apply to device (sensors pane: "vibrator ON").
 4. Trials in this order: empty, relaxed, medium, max (Effort card: select, Start capture; about 43 s each; rest a
    minute). Elbow on the support, upper arm forward, forearm up, hand holds the handle without pressing.
 5. After max: CHECK / VERDICT in the MOSAIC pane. GOOD: validation trials relaxed, medium, max -> VALIDATION.
 6. q + Enter here ends the session (robot program, vibrator, MOSAIC; logs and results into the session folder).
 Here: q = end, v = vibrator off now, s = status, Enter = this list.
EOF
  else
    file=$CAL/mosaic/session_run.json
    cat <<EOF
TELEOPERATION   (click a pane to type in it; the loggers are in window 1: Ctrl+b then n switches windows)
 Before: model from today's calibration with VALIDATION PASS; Desk unlocked, FCI active; emergency stop in reach.
 1. MOSAIC window: File -> Open -> $file; Vibrator card: All, Apply to device.
 2. Session card: relaxed, Start capture, 5 s relaxed, Stop capture; then tense the same way (direction of each pair).
 3. slave pane: press Enter (home move). master pane: the command is typed, press Enter to start master_ti; switch on
    the VIRTUOSE power button (force feedback) and release the deadman once.
 4. q + Enter here ends the session (master, slave, MOSAIC, vibrator; logs into the session folder).
 Here: q = end, v = vibrator off now, s = status, Enter = this list.
EOF
  fi
}

control() {  # kind folder  (runs in the control pane)
  local kind=$1 dir=$2 done_=false cmd
  finish() {
    $done_ && return
    done_=true
    shutdown_session "$kind" "$dir"
    echo
    cat "$dir/summary.txt"
    sleep 2
    tmux kill-session -t "$TS" 2>/dev/null
  }
  trap finish EXIT
  trap 'exit 0' INT TERM HUP
  checklist "$kind"
  while true; do
    read -r -p "> " cmd || cmd=q
    case $cmd in
      q)
        read -r -p "end the session and stop the robot programs? [y/N] " cmd || cmd=y
        [[ $cmd == [yY]* ]] && break ;;
      v) tmux send-keys -t "$(pid_of sensors)" off Enter && echo "vibrator off sent" ;;
      s)
        tmux list-panes -s -t "$TS" -F '  #{@name}: #{pane_current_command}'
        $TEST || ours | sed 's/^/  /' ;;
      *) checklist "$kind" ;;
    esac
  done
  exit 0
}

# --- start ---

pane() {  # window-target title "command" [typed-only]
  local target=$1 title=$2 cmd=$3 typed=${4:-}
  tmux select-pane -t "$target" -T "$title"
  tmux set -p -t "$target" @name "$title"
  tmux pipe-pane -t "$target" -o "cat >> '$DIR/panes/$title.log'"
  tmux send-keys -t "$target" -l "$cmd"
  [ -n "$typed" ] || tmux send-keys -t "$target" Enter
}

fake() {  # name stop-key: stand-in that ends on q + Enter or on Ctrl+C
  if [ "$2" = q ]; then
    echo "python3 -c \"import sys; print('[test] $1 stand-in, q + Enter ends'); [sys.exit(print('[test] $1 stopped')) if l.strip() == 'q' else print('[test] $1 got', l.strip()) for l in sys.stdin]\""
  else
    echo "python3 -c \"import time; print('[test] $1 stand-in, Ctrl+C ends'); time.sleep(1e9)\""
  fi
}

start_session() {
  local stamp
  stamp=$(date +%Y%m%d_%H%M%S)
  DIR=$SESSIONS/${stamp}_$KIND
  mkdir -p "$DIR/panes" "$DIR/model"
  touch "$DIR/.start"
  echo "$KIND $DIR" >"$CURRENT"
  cp -p "$MODELS/stiffness_projection.json" "$DIR/model/before.json" 2>/dev/null
  {
    echo "# $KIND session $stamp"
    echo
    echo "- fr3-hand-stiffness-perturbation: $(git -C "$CAL" log -1 --format='%h %s' 2>/dev/null)"
    echo "- tele-impedance: $(git -C "$TI" log -1 --format='%h %s' 2>/dev/null)"
    echo "- robot $ROBOT_IP, board $BOARD, MOSAIC $(mosaic_tfm)"
    echo "- model before: $(python3 -c "import json,sys; m=json.load(open(sys.argv[1])); print(m.get('format'), (m.get('fit') or {}).get('date'))" "$DIR/model/before.json" 2>/dev/null || echo none)"
    $TEST && echo "- TEST RUN with stand-in programs"
    echo
    echo "## Notes (operator, mounting, what happened)"
    echo
  } >"$DIR/session.md"

  local sens robot mosaic slave master
  local mosaic_cmd="cd '$MOSAIC' && $(dotnet_env) $DOTNET run --no-build --project MOSAIC.Desktop -f $(mosaic_tfm)"
  sens="cd '$CAL' && python3 sensors.py --port $BOARD --out '$DIR/sensors'"
  if $TEST; then sens=$(fake sensors q); mosaic_cmd=$(fake MOSAIC C-c); fi

  tmux new-session -d -s "$TS" -x 240 -y 64 -n main "bash --noprofile --norc"
  tmux set -t "$TS" mouse on
  tmux set -w -t "$TS:main" pane-border-status top
  tmux set -w -t "$TS:main" pane-border-format ' #{@name} '
  # control pane: top left; it gets the settings of this run explicitly (a running tmux server has its own environment)
  tmux select-pane -t "$TS:main.0" -T control
  tmux set -p -t "$TS:main.0" @name control
  tmux pipe-pane -t "$TS:main.0" -o "cat >> '$DIR/panes/control.log'"
  local flags=""
  $TEST && flags+=" --test"
  tmux send-keys -t "$TS:main.0" -l "clear; THESIS_DIR='$THESIS' SESSIONS_DIR='$SESSIONS' BOARD_PORT='$BOARD' ROBOT_IP='$ROBOT_IP' DOTNET='$DOTNET' exec bash '$HERE/session.sh' control$flags"
  tmux send-keys -t "$TS:main.0" Enter
  tmux split-window -h -t "$TS:main.0" "bash --noprofile --norc"     # right: .1
  tmux split-window -v -t "$TS:main.0" "bash --noprofile --norc"     # left bottom: .1, right moves to .2
  if [ "$KIND" = calibration ]; then
    robot="cd '$CAL' && ./perturbation $ROBOT_IP --out '$DIR/robot'"
    $TEST && robot=$(fake perturbation C-c)
    tmux split-window -v -t "$TS:main.2" "bash --noprofile --norc"   # right bottom: .3
    pane "$TS:main.1" sensors "$sens"
    pane "$TS:main.2" robot "$robot"
    pane "$TS:main.3" mosaic "$mosaic_cmd"
  else
    slave="cd '$TI/SlaveTeleImpedance' && ./slave_ti $ROBOT_IP"
    master="cd '$TI/MasterTeleImpedance' && ./master_ti"
    $TEST && { slave=$(fake slave_ti C-c); master=$(fake master_ti C-c); }
    tmux split-window -v -t "$TS:main.2" "bash --noprofile --norc"   # right bottom: .3
    tmux split-window -v -t "$TS:main.1" "bash --noprofile --norc"   # left bottom split: .2, right moves to .3/.4
    pane "$TS:main.1" sensors "$sens"
    pane "$TS:main.2" mosaic "$mosaic_cmd"
    pane "$TS:main.3" slave "$slave"
    pane "$TS:main.4" master "$master" typed
    tmux new-window -d -t "$TS:1" -n loggers "bash --noprofile --norc"
    tmux set -w -t "$TS:loggers" pane-border-status top
    tmux set -w -t "$TS:loggers" pane-border-format ' #{@name} '
    tmux split-window -v -t "$TS:loggers.0" "bash --noprofile --norc"
    local tl="cd '$TI' && python3 SlaveTeleImpedance/tank_logger.py '$DIR/slave_tank.csv'"
    local ml="cd '$TI' && python3 MasterTeleImpedance/master_tank_logger.py '$DIR/master_tank.csv'"
    $TEST && { tl=$(fake tank_logger C-c); ml=$(fake master_tank_logger C-c); }
    pane "$TS:loggers.0" slave_tank "$tl"
    pane "$TS:loggers.1" master_tank "$ml"
    if [ -n "$LOGGER" ]; then
      tmux split-window -v -t "$TS:loggers.1" "bash --noprofile --norc"
      pane "$TS:loggers.2" teleop_logger "cd '$TI' && sudo python3 teleop_logger.py --label '$LOGGER' --project ."
    fi
  fi
  tmux select-pane -t "$TS:main.0"
}

case $KIND in
  calibration | teleop)
    if tmux has-session -t "$TS" 2>/dev/null; then
      echo "a session is already running (tmux session \"$TS\"): tmux attach -t $TS, or run/stop.sh"
      exit 1
    fi
    BUILDLOG=$(mktemp -d)
    echo "checks for the $KIND session ($THESIS)"
    if $TEST; then
      warn "test run: stand-in programs, no checks"
      check_tools
    else
      check_tools
      check_repo "$CAL" main fr3-hand-stiffness-perturbation
      [ "$KIND" = teleop ] && check_repo "$TI" tele-impedance-sensors tele-impedance
      if [ "$KIND" = calibration ]; then
        check_binary perturbation "$CAL" perturbation \
          "g++ -std=c++17 -O2 -Wall -Wextra perturbation.cpp -I/usr/include/eigen3 -o perturbation -lfranka -pthread" \
          "$CAL/perturbation.cpp"
      else
        check_binary slave_ti "$TI/SlaveTeleImpedance" slave_ti \
          "g++ -std=c++17 -O2 -Wall -Wextra slave_ti.cpp examples_common.cpp -I. -I../Common -I/usr/include/eigen3 -o slave_ti -lfranka -pthread" \
          "$TI/SlaveTeleImpedance" "$TI/Common"
        if [ -f "$TI/MasterTeleImpedance/virtuoseAPI.so" ]; then
          check_binary master_ti "$TI/MasterTeleImpedance" master_ti \
            "g++ -std=c++17 -O2 -Wall -Wextra master_ti.cpp -I. -I../Common -I/usr/include/eigen3 ./virtuoseAPI.so -Wl,-rpath,\$PWD -o master_ti -pthread" \
            "$TI/MasterTeleImpedance" "$TI/Common"
        else
          bad "master_ti: MasterTeleImpedance/virtuoseAPI.so missing (not in git, copy it in)"
        fi
      fi
      check_mosaic
      check_robot
      check_board
      check_leftovers
      if [ "$KIND" = calibration ]; then check_ports 50500 50601 50900 50910; else check_ports 50500 50601 4500 11061 11062; fi
      [ "$KIND" = teleop ] && report_model
    fi
    if [ "$FAILS" -gt 0 ]; then
      echo
      echo "$FAILS check(s) failed: fix them and start again (or follow mosaic/SESSION_PIPELINES.md by hand)."
      exit 1
    fi
    if $DRY; then echo; echo "dry run: checks done, nothing started."; exit 0; fi
    mkdir -p "$SESSIONS"
    start_session
    echo "starting: tmux session \"$TS\" (folder $DIR)"
    if [ -n "${TMUX:-}" ]; then
      tmux switch-client -t "$TS"
    elif [ -t 0 ] && [ -t 1 ]; then
      tmux attach -t "$TS"
    else
      echo "no terminal here: open one and run  tmux attach -t $TS"
      exit 0
    fi
    [ -f "$DIR/summary.txt" ] && { echo; cat "$DIR/summary.txt"; }
    ;;
  control)
    read -r kind dir <"$CURRENT"
    control "$kind" "$dir"
    ;;
  stop)
    kind="" dir=""
    [ -f "$CURRENT" ] && read -r kind dir <"$CURRENT"
    if tmux has-session -t "$TS" 2>/dev/null; then
      # the control pane does the shutdown and then closes the tmux session
      echo "ending the session in tmux session \"$TS\" ..."
      tmux send-keys -t "$(pid_of control)" q Enter y Enter
      for _ in $(seq 90); do tmux has-session -t "$TS" 2>/dev/null || break; sleep 1; done
      if tmux has-session -t "$TS" 2>/dev/null; then
        echo "the control pane did not end the session: stopping the programs directly"
        [ -n "$dir" ] && shutdown_session "$kind" "$dir"
        $TEST || stop_leftovers
        tmux kill-session -t "$TS"
      fi
      [ -n "$dir" ] && [ -f "$dir/summary.txt" ] && cat "$dir/summary.txt"
    elif [ -n "$dir" ]; then
      shutdown_session "$kind" "$dir"
    else
      echo "no session running; stopping any program left over"
      stop_leftovers
    fi
    ;;
  *)
    echo "usage: run/calibration.sh | run/teleop.sh [--dry-run] [--test] [--logger <label>] | run/stop.sh"
    exit 2
    ;;
esac
