#!/usr/bin/env python3
"""
Standalone cflib script to light up the bottom-mounted Color LED deck on a
Crazyflie 2.1.

This bypasses crazyswarm2 / ROS 2 entirely and talks to the Crazyflie directly
over the Crazyradio USB dongle using cflib.

IMPORTANT — radio contention:
    cflib and the crazyswarm2 `crazyflie_server` BOTH use the Crazyradio dongle
    and cannot both own the link at the same time. STOP your `ros2 launch ...`
    (the crazyflie_server) BEFORE running this script, otherwise the link will
    fail to open.

Usage:
    # First stop the running crazyswarm2 launch / crazyflie_server, then:
    python3 scripts/color_led_cflib.py                 # first enabled drone
    python3 scripts/color_led_cflib.py --drone cf3
    python3 scripts/color_led_cflib.py --list
    python3 scripts/color_led_cflib.py --uri radio://0/80/2M/E7E7E7E703

Deck params used (firmware 2026.04):
    colorLedBot.wrgb8888  (uint32, packed 0xWWRRGGBB)  -> the color to display
    colorLedBot.brightCorr (uint8, 0/1, default 1)      -> brightness correction
"""

import argparse
import logging
import os
import sys
import time

import cflib.crtp
import yaml
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ----------------------------------------------------------------------------
# Which drone to talk to. Derived from crazyflies.yaml -- the only roster in
# this workspace -- not hardcoded. The literal that used to sit here was
# 'radio://0/81/2M/E7E7E7E711': cf11 is not in the fleet, channel 81 is used
# by nothing, and cf11 never spoke 2M anyway (it was the 1M drone that taught
# this rig the datarate-mismatch lesson). Override with --drone cfN or --uri.
# ----------------------------------------------------------------------------
DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'src', 'crazyswarm2', 'crazyflie', 'config', 'crazyflies.yaml')


def fleet_uris(yaml_path=DEFAULT_YAML):
    """{name: uri} for every enabled drone, dongle wildcard pinned to 0.

    cflib's SyncCrazyflie needs a concrete dongle index; the server accepts
    `radio://*/`.
    """
    with open(yaml_path) as fh:
        robots = (yaml.safe_load(fh) or {}).get('robots') or {}
    return {name: r['uri'].replace('radio://*/', 'radio://0/')
            for name, r in robots.items() if (r or {}).get('enabled')}

# Param names on the Color LED deck (bottom).
PARAM_COLOR = 'colorLedBot.wrgb8888'
PARAM_BRIGHT = 'colorLedBot.brightCorr'

# Only surface errors from cflib's own logging; our status prints use flush.
logging.basicConfig(level=logging.ERROR)


def wrgb(r, g, b, w=0):
    """Pack white/red/green/blue (each 0-255) into a uint32 0xWWRRGGBB value."""
    return ((w & 0xFF) << 24) | ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)


# Color sequence: (label, r, g, b, w, hold_seconds)
SEQUENCE = [
    ('green', 0, 255, 0, 0, 2.0),
    ('red',   255, 0, 0, 0, 1.5),
    ('green', 0, 255, 0, 0, 1.5),
    ('blue',  0, 0, 255, 0, 1.5),
    ('white', 0, 0, 0, 255, 1.5),
    ('off',   0, 0, 0, 0, 1.0),
]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--drone', help='drone name from crazyflies.yaml, e.g. cf3')
    ap.add_argument('--uri', help='explicit radio URI, bypassing the yaml')
    ap.add_argument('--yaml', default=DEFAULT_YAML, help='fleet file to read')
    ap.add_argument('--list', action='store_true',
                    help='print the enabled fleet and exit')
    return ap.parse_args(argv)


def resolve_uri(args):
    """The URI to talk to, and the name it belongs to."""
    if args.uri:
        return args.uri, args.uri
    uris = fleet_uris(args.yaml)
    if not uris:
        raise SystemExit(f'no enabled drones in {args.yaml}')
    if args.drone:
        if args.drone not in uris:
            raise SystemExit(f'{args.drone} is not enabled in {args.yaml} '
                             f'(enabled: {", ".join(sorted(uris))})')
        return uris[args.drone], args.drone
    name = sorted(uris)[0]
    return uris[name], name


def run(argv=None):
    args = parse_args(argv)
    uris = fleet_uris(args.yaml) if not args.uri else {}
    if args.list:
        for name in sorted(uris):
            print(f'  {name:6} {uris[name]}')
        return 0
    uri, name = resolve_uri(args)

    cflib.crtp.init_drivers()

    # rw_cache lets cflib cache the log/param TOC between runs so it does not
    # have to be re-downloaded every connect (faster reconnects). Harmless if
    # the dir does not exist yet — cflib creates it.
    cf = Crazyflie(rw_cache='./cache')

    print('Connecting to %s (%s) ...' % (name, uri), flush=True)

    # SyncCrazyflie.open_link() (via the context manager) blocks until the
    # link is up AND the log/param TOCs have been downloaded. param.set_value()
    # additionally waits internally for the param TOC to be initialised, so it
    # is safe to set params immediately after the context manager enters.
    try:
        scf = SyncCrazyflie(uri, cf=cf)
        scf.open_link()
    except Exception as exc:  # noqa: BLE001 - any link error should print the hint
        print('Could not open radio link — is the crazyflie_server / ros2 '
              'launch still running? Stop it first (it holds the Crazyradio).',
              flush=True)
        print('  (underlying error: %s)' % exc, flush=True)
        return 1

    try:
        print('Connected. Enabling brightness correction.', flush=True)
        scf.cf.param.set_value(PARAM_BRIGHT, 1)

        for label, r, g, b, w, hold in SEQUENCE:
            value = wrgb(r, g, b, w)
            print('  -> %-6s (0x%08X)' % (label, value), flush=True)
            scf.cf.param.set_value(PARAM_COLOR, value)
            time.sleep(hold)

        # Make sure the LED ends up off before we disconnect.
        scf.cf.param.set_value(PARAM_COLOR, 0)
        print('Done. LED off.', flush=True)
    finally:
        scf.close_link()

    return 0


if __name__ == '__main__':
    sys.exit(run())
