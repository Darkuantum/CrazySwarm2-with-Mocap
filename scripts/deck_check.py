#!/usr/bin/env python3
"""Which LED decks are actually fitted, and what are they showing?

Read-only. Connects to each enabled drone over the radio, reads the deck
detection params and the current LED state, and prints a table. Never arms,
never spins a motor, never sends a setpoint.

Answers the question the param TOC cannot: `src/crazyswarm2/crazyflie_shows/reference/firmware_param_toc.csv`
lists every deck driver compiled into the firmware, including decks this rig
does not own. Only the *value* of deck.bc* says what is physically attached.

    python3 scripts/deck_check.py              # the enabled fleet in crazyflies.yaml
    python3 scripts/deck_check.py --yaml X.yaml

Needs the Crazyradio plugged in and the drones powered. Does not need ROS,
mocap, or the crazyflie_server -- run it before or instead of a launch, and
STOP the server first: there is one radio owner.
"""

import argparse
import os

import yaml

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# The fleet comes from crazyflies.yaml -- the workspace's ground truth -- rather
# than a list in this file. The hardcoded copy it replaced still named the
# retired cf10/cf12 and missed cf5/cf8.
DEFAULT_YAML = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'src', 'crazyswarm2', 'crazyflie', 'config', 'crazyflies.yaml'))


def enabled_uris(yaml_path):
    """{name: uri} for every enabled drone, in the yaml's order.

    `radio://*/...` (cf1's wildcard dongle) is pinned to dongle 0: the server
    accepts the wildcard, cflib's SyncCrazyflie needs a concrete index.
    """
    with open(yaml_path) as fh:
        robots = (yaml.safe_load(fh) or {}).get('robots') or {}
    return {name: r['uri'].replace('radio://*/', 'radio://0/')
            for name, r in robots.items() if r.get('enabled')}

# Deck presence flags (read-only uint8, 1 = detected at boot).
DECKS = [
    ('deck.bcLedRing',     'LED-ring (12x WS2812B)'),
    ('deck.bcColorLedTop', 'ColorLed top (RGBW)'),
    ('deck.bcColorLedBot', 'ColorLed bottom (RGBW)'),
]

# Current light state, to explain what the halo actually is.
STATE = [
    'ring.effect',          # 0=off 6=double spinner (default) 7=solid colour
    'ring.solidRed', 'ring.solidGreen', 'ring.solidBlue',
    'colorLedTop.wrgb8888',  # packed 0xWWRRGGBB
    'colorLedBot.wrgb8888',
]

RING_EFFECTS = {
    0: 'off', 1: 'white spinner', 2: 'colour spinner', 3: 'tilt',
    4: 'brightness', 5: 'colour spinner 2', 6: 'double spinner',
    7: 'SOLID COLOUR', 8: 'factory test', 9: 'battery status',
    10: 'boat lights', 11: 'alert', 12: 'gravity', 13: 'virtual memory',
    14: 'fade colour', 17: 'LED timing from memory',
}


def read_params(scf, names):
    """Read named params, returning {name: value or None if absent}.

    ``get_value`` takes one dotted name and raises KeyError for anything not
    in this firmware's TOC -- which is the answer for a deck whose driver was
    not compiled in, so it is a result, not an error.
    """
    out = {}
    for name in names:
        try:
            out[name] = scf.cf.param.get_value(name)
        except KeyError:
            out[name] = None
    return out


def describe(vals):
    """One line summarising what this drone is showing."""
    bits = []

    effect = vals.get('ring.effect')
    if effect is not None:
        e = int(effect)
        label = RING_EFFECTS.get(e, f'effect {e}')
        if e == 7:
            rgb = tuple(int(vals.get(f'ring.solid{c}') or 0)
                        for c in ('Red', 'Green', 'Blue'))
            label += f' rgb{rgb}'
        bits.append(f'ring: {label}')

    for side in ('Top', 'Bot'):
        packed = vals.get(f'colorLed{side}.wrgb8888')
        if packed is not None:
            v = int(packed)
            w, r, g, b = (v >> 24) & 0xFF, (v >> 16) & 0xFF, \
                         (v >> 8) & 0xFF, v & 0xFF
            if v:
                bits.append(f'colorLed{side}: w{w} r{r} g{g} b{b}')

    return '; '.join(bits) if bits else 'nothing lit'


def main():
    ap = argparse.ArgumentParser(description='Which LED decks are fitted, per drone.')
    ap.add_argument('--yaml', default=DEFAULT_YAML,
                    help='crazyflies.yaml to take the enabled fleet from')
    args = ap.parse_args()
    uris = enabled_uris(args.yaml)
    if not uris:
        raise SystemExit(f'{args.yaml}: no drones have `enabled: true`')

    cflib.crtp.init_drivers()
    print(f'{"drone":6} {"ring":>5} {"cTop":>5} {"cBot":>5}  showing')
    print('-' * 64)

    unreachable = []
    for name, uri in uris.items():
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                fitted = read_params(scf, [p for p, _ in DECKS])
                state = read_params(scf, STATE)
                cells = ''.join(
                    f'{("YES" if str(fitted[p]) not in ("0", "None") else "-"):>6}'
                    for p, _ in DECKS)
                print(f'{name:6}{cells}  {describe(state)}')
        except Exception as exc:
            unreachable.append(name)
            print(f'{name:6}{"?":>6}{"?":>6}{"?":>6}  unreachable: '
                  f'{type(exc).__name__}: {exc}')

    print()
    for param, label in DECKS:
        print(f'  {param:22} {label}')
    if unreachable:
        print(f'\n  not answering: {", ".join(unreachable)} '
              '-- powered on? radio in range?')


if __name__ == '__main__':
    main()
