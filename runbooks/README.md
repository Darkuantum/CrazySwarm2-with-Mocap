# Runcards

One card per show: **what to type, in order, with every argument.** No rationale
— that lives in the show's own document, linked at the top of each card. These
exist so a show can be flown by someone who is not going to read 25 KB first.

| show | card | design doc | duration |
|---|---|---|---|
| carousel | [CAROUSEL.runcard.md](CAROUSEL.runcard.md) | `../src/crazyswarm2/crazyflie_shows/SHOW_GUIDE.md` | ~63 s |
| constellation | [CONSTELLATION.runcard.md](CONSTELLATION.runcard.md) | `../src/crazyswarm2/crazyflie_shows/CONSTELLATION.md` | 76 s |
| escort | [ESCORT.runcard.md](ESCORT.runcard.md) | `../src/crazyswarm2/crazyflie_shows/ESCORT.md` | script length, or 120 s |

The escort also has two guest-facing documents:
[ESCORT.narrative.md](ESCORT.narrative.md) — what to say at each stage, a
light legend, and which claims to keep honest — and
[ESCORT.qna.md](ESCORT.qna.md) for everything asked that the script does not
cover: answers by audience, what to say when a run fails in front of someone,
the honest list of limits, and a numbers card to read off instead of
remembering.

Every show follows the same spine, and skipping a step is how the known
accidents happened:

```
plan (no radio)  ->  sim  ->  dry_run on hardware  ->  fly
```

**Re-run the planner after every `sync_initial_positions.py`.** The carousel
sits at 99 % of its separation budget; a position sync can invalidate a plan
that passed an hour ago.

## Two rules that apply to all three

**Scan every enabled address before every launch.** The server connects drones
in lexicographic order and hangs *silently* on the first enabled drone that
does not answer — no error, no `/all/*` services. Derive the list from the
yaml rather than from memory; the enabled set changes between shows:

```bash
cd ~/CrazySwarm2-with-Mocap
ADDRS=$(python3 -c "
import yaml
d = yaml.safe_load(open('src/crazyswarm2/crazyflie/config/crazyflies.yaml'))['robots']
print(' '.join('0x' + v['uri'].split('/')[-1] for k, v in sorted(d.items()) if v.get('enabled')))
")
echo "$ADDRS"
for a in $ADDRS; do echo "-- $a"; ros2 run crazyflie scan --address "$a"; done
```

Every address must answer, **and at the datarate its URI pins**. A URI that
says `2M` against a drone answering only `1M` hangs the server forever.

**E-STOP, and what it costs.** Any of: the console's E-STOP button, `e` in the
preflight GUI, or

```bash
ros2 service call /all/emergency std_srvs/srv/Empty '{}'
```

It cuts motors instantly — the drones *drop*. Afterwards the firmware
supervisor latches **LOCKED** (`/cfX/status` `supervisor_info` bit `0x40`, with
`CAN_BE_ARMED` clear) and the only way out is a **battery out-and-in on every
drone**. Nothing will arm until you do that, and the only symptom is that
nothing flies.
