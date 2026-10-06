#!/usr/bin/env bash
#
# led.sh -- manually control the Crazyflie bottom Color LED deck (colorLedBot).
#
# Maps color NAMES to the firmware param colorLedBot.wrgb8888 (uint32 packed as
# 0xWWRRGGBB) and sets it live via the `ros2 param set` CLI.
#
# REQUIREMENTS / WHEN TO RUN:
#   The crazyflie_server must be running, i.e. start it first with:
#       ros2 launch crazyflie launch.py
#   and this shell must have ROS sourced (/opt/ros/$ROS_DISTRO/setup.bash and
#   install/setup.bash) with a matching ROS_DOMAIN_ID (unset/default on this rig).
#
# WHY THE CLI AND NOT rclpy:
#   On this rig, fresh rclpy processes (and crazyflie_py's Crazyswarm()) fail:
#   DDS discovery never completes so set_parameters/list_parameters service
#   calls time out, and Crazyswarm() hangs on all/emergency. The long-running
#   `ros2` CLI daemon keeps the ROS graph warm, so `ros2 param set` /
#   `ros2 param list` complete reliably. This script uses only that path.
#
# USAGE:
#   ./scripts/led.sh Yellow     # one-shot: set color and exit
#   ./scripts/led.sh 3          # same, by number key
#   ./scripts/led.sh Black      # LED off (same as 0)
#   ./scripts/led.sh            # interactive: press a number key to switch,
#                               # no Enter needed; q to quit
#
# NUMBER KEYS:
#   0=off(black)  1=green  2=red  3=yellow  4=blue  5=purple  9=white

set -u

SERVER='/crazyflie_server'
LED_SUFFIX='params.colorLedBot.wrgb8888'
# If the server advertises no LED params, fall back to the enabled fleet from
# crazyflies.yaml rather than a hardcoded name. This used to be
# 'cf11.params...': cf11 is not in the fleet and is on a channel nothing uses,
# so the fallback could only ever fail.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FLEET_YAML="$REPO_DIR/src/crazyswarm2/crazyflie/config/crazyflies.yaml"

fallback_params() {
    python3 - "$FLEET_YAML" "$LED_SUFFIX" <<'PYEOF' 2>/dev/null
import sys

import yaml

robots = (yaml.safe_load(open(sys.argv[1])) or {}).get('robots') or {}
for name in sorted(n for n, r in robots.items() if (r or {}).get('enabled')):
    print(f'{name}.{sys.argv[2]}')
PYEOF
}

# Named colors -> DECIMAL wrgb value (ros2 param set needs decimal integers).
# Values match src/crazyswarm2/crazyflie_examples/crazyflie_examples/color_led.py:
#   red=0xFF0000 yellow=0xFFFF00 green=0x00FF00 blue=0x0000FF
#   purple=0x9400D3 pink=0xFF69B4 white=0xFF000000 (W channel) black=0
color_value() {
    case "$1" in
        red)    echo 16711680 ;;   # 0xFF0000
        yellow) echo 16776960 ;;   # 0xFFFF00
        green)  echo 65280 ;;      # 0x00FF00
        blue)   echo 255 ;;        # 0x0000FF
        purple) echo 9699539 ;;    # 0x9400D3
        white)  echo 4278190080 ;; # 0xFF000000 dedicated W channel (RGB mix looks purplish)
        black)  echo 0 ;;          # 0x000000 (LED off)
        *)      return 1 ;;
    esac
}

VALID_COLORS='red yellow green blue purple white black'
KEY_HELP='0=off(black)  1=green  2=red  3=yellow  4=blue  5=purple  9=white'

# Number keys -> color names (0 = off).
key_to_color() {
    case "$1" in
        0) echo black ;;
        1) echo green ;;
        2) echo red ;;
        3) echo yellow ;;
        4) echo blue ;;
        5) echo purple ;;
        9) echo white ;;
        *) return 1 ;;
    esac
}

# Discover every drone's LED param from the live server. Trims whitespace that
# `ros2 param list` indents its entries with.
discover_led_params() {
    ros2 param list "$SERVER" 2>/dev/null \
        | grep "$LED_SUFFIX" \
        | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' \
        | grep -v '^$'
}

# Apply a color name to all discovered LED params. Returns non-zero on bad name.
apply_named() {
    local raw="$1"
    local name value
    name=$(printf '%s' "$raw" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')

    # Accept a number key as well as a color name.
    case "$name" in
        [0-9])
            if ! name=$(key_to_color "$name"); then
                echo "no color on key '$raw'. keys: $KEY_HELP"
                return 1
            fi
            ;;
    esac

    if ! value=$(color_value "$name"); then
        echo "unknown color '$raw'. choose from: $VALID_COLORS (or keys: $KEY_HELP)"
        return 1
    fi

    # Discover drones fresh each time (a drone may (re)connect between calls).
    local params
    mapfile -t params < <(discover_led_params)

    if [ "${#params[@]}" -eq 0 ]; then
        mapfile -t params < <(fallback_params)
        if [ "${#params[@]}" -eq 0 ]; then
            echo "ERROR: no '$LED_SUFFIX' params on $SERVER and no enabled" \
                 "drones in $FLEET_YAML" >&2
            return 1
        fi
        echo "WARNING: no '$LED_SUFFIX' params found on $SERVER;" \
             "falling back to the enabled fleet (${params[*]%%.*})"
    fi

    local hex
    hex=$(printf '0x%08X' "$value")
    printf 'Setting %s (%s = %d) on %d drone(s):\n' \
        "$name" "$hex" "$value" "${#params[@]}"

    local p rc=0
    for p in "${params[@]}"; do
        if ros2 param set "$SERVER" "$p" "$value" >/dev/null 2>&1; then
            echo "  OK   $p"
        else
            echo "  FAIL $p"
            rc=1
        fi
    done
    return $rc
}

main() {
    if [ "$#" -ge 1 ]; then
        # One-shot mode.
        apply_named "$1"
        exit $?
    fi

    # Interactive mode: one keypress = one color change, no Enter needed.
    echo "Manual Color LED control - press a number key:"
    echo "  $KEY_HELP   q=quit"
    local key
    while true; do
        if ! read -rsn1 key; then
            echo
            break
        fi
        case "$key" in
            q|Q)   break ;;
            [0-9]) apply_named "$key" ;;
            *)     : ;;  # ignore everything else (incl. escape sequences)
        esac
    done
}

main "$@"
