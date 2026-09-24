#!/usr/bin/env python3
"""Sync `initial_position` in crazyflies.yaml from the live mocap /poses stream.

Why this exists
---------------
`initial_position` seeds the onboard Kalman estimate at server start. If a drone
physically sits somewhere other than the position the yaml claims, the estimate
starts wrong; the mocap then force-fuses the true position (locSrv.extPosStdDev
is 1e-3) and the controller flies hard at the difference -- the runaway/darting
behaviour at takeoff. Keeping the yaml in step with reality is therefore a
go/no-go item, and doing it by hand (echo /poses, read numbers, retype them) is
slow and easy to get wrong.

This script reads the live `/poses` stream, matches each ENABLED drone in
crazyflies.yaml to the rigid body of the same name, and rewrites just the
`initial_position` numbers in place. Comments, ordering and formatting in the
yaml are preserved -- only the bracketed triples change.

IMPORTANT
  * `initial_position` MUST come from /poses (the mocap's own view), never from
    /cfX/pose -- the onboard estimate is seeded by this very yaml, so copying it
    back is circular and will happily preserve an existing error.
  * The yaml is read ONLY at server launch. Run this BEFORE `ros2 launch`, with
    the mocap up but the crazyflie server not yet started (or restart it after).

Usage
-----
  # mocap running (e.g. `ros2 launch crazyflie launch.py`, or just the mocap node)
  python3 scripts/sync_initial_positions.py            # show the diff, ask, apply
  python3 scripts/sync_initial_positions.py --dry-run  # show the diff only
  python3 scripts/sync_initial_positions.py --yes      # no prompt (scripted use)

Exit codes: 0 ok / nothing to do, 1 refused (failed a safety check), 2 no data.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from collections import defaultdict

# --- tunables ---------------------------------------------------------------
DEFAULT_SAMPLES = 50           # ~1 s at the rig's 50 Hz streaming rate
DEFAULT_TIMEOUT = 10.0         # s to wait for that many samples
JITTER_ABORT_M = 0.010         # 10 mm: body moving / tracking unstable -> refuse
MIN_SEPARATION_M = 1.0         # CLAUDE.md rig rule: enabled drones >= 1 m apart
Z_SANITY_M = 0.5               # a grounded drone should not be this high

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "src", "crazyswarm2", "crazyflie", "config", "crazyflies.yaml")

#: The OTHER copies of crazyflies.yaml that must not be left behind.
#:
#: There are two in this workspace and they are not a mistake: show_launch.py
#: passes the crazyflie_shows copy as `crazyflies_yaml_file`, so it is what the
#: SERVER reads for a show launch, and it is what plan_show / plan_constellation
#: / plan_escort read always. Syncing one alone leaves the planner verifying
#: against the old marks while the server seeds the new ones -- which is
#: exactly the state this rig was found in on 2026-09-24, the two copies
#: 1-2 cm apart. Writing every copy that has the same enabled fleet is safer
#: than remembering; --only sticks to --yaml when a copy is deliberately
#: different.
EXTRA_YAMLS = [os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "src", "crazyswarm2", "crazyflie_shows", "config", "crazyflies.yaml")]


def sibling_yamls(primary):
    """Other crazyflies.yaml copies to keep in step with ``primary``."""
    out = []
    primary = os.path.realpath(primary)
    for path in EXTRA_YAMLS:
        path = os.path.normpath(path)
        if os.path.exists(path) and os.path.realpath(path) != primary:
            out.append(path)
    return out


# --- yaml I/O ---------------------------------------------------------------
def read_enabled(yaml_path):
    """Return {name: [x, y, z]} for robots with `enabled: true`.

    Parsed with PyYAML (read only). Writing goes through rewrite_positions(),
    which edits text in place -- a yaml.dump() round-trip would strip every
    comment in the file, and this file is heavily commented.
    """
    try:
        import yaml
    except ImportError:
        sys.exit("PyYAML missing. Install with: sudo apt install python3-yaml")

    with open(yaml_path) as f:
        doc = yaml.safe_load(f)

    robots = (doc or {}).get("robots") or {}
    out = {}
    for name, cfg in robots.items():
        if not isinstance(cfg, dict) or not cfg.get("enabled", False):
            continue
        pos = cfg.get("initial_position", [0.0, 0.0, 0.0])
        out[name] = [float(v) for v in pos]
    return out


def rewrite_positions(yaml_path, updates, keep_z=True):
    """Rewrite `initial_position` for each name in `updates` -> (text, changed).

    Edits the raw text so comments/formatting survive. For each robot we find
    its `<indent>name:` header, then the first `initial_position:` line that is
    indented deeper than the header and before the next key at header depth --
    i.e. strictly inside that robot's own block.
    """
    with open(yaml_path) as f:
        lines = f.readlines()

    changed = {}
    for name, xyz in updates.items():
        head = None
        for i, line in enumerate(lines):
            m = re.match(r"^(\s*)" + re.escape(name) + r":\s*(#.*)?$", line)
            if m:
                head, head_indent = i, len(m.group(1))
                break
        if head is None:
            raise KeyError(f"{name}: no `{name}:` block found in {yaml_path}")

        for j in range(head + 1, len(lines)):
            line = lines[j]
            if line.strip() and not line.lstrip().startswith("#"):
                indent = len(line) - len(line.lstrip())
                if indent <= head_indent:
                    break  # left this robot's block
            m = re.match(r"^(\s*initial_position:\s*)\[([^\]]*)\](.*)$", line)
            if not m:
                continue
            old = [float(v) for v in m.group(2).split(",")]
            z = old[2] if (keep_z and len(old) > 2) else xyz[2]
            new = [xyz[0], xyz[1], z]
            lines[j] = "{}[{}, {}, {}]{}\n".format(
                m.group(1), fmt(new[0]), fmt(new[1]), fmt(new[2]), m.group(3).rstrip())
            changed[name] = (old, new)
            break
        else:
            raise KeyError(f"{name}: no `initial_position:` inside its block")

    return "".join(lines), changed


def fmt(v):
    """Round to mm and drop float noise -- mocap is not precise past ~1 mm.

    Always keeps a decimal point so the value stays a YAML float: a bare `2`
    would load as an int, making initial_position a mixed int/float array.
    """
    out = f"{round(v, 4):g}"
    return out + ".0" if ("." not in out and "e" not in out) else out


# --- mocap sampling ---------------------------------------------------------
def collect_poses(topic, n_samples, timeout):
    """Average `n_samples` frames of `topic` -> ({name: (x,y,z)}, {name: jitter}).

    Subscribes with SensorDataQoS (BEST_EFFORT): the mocap node publishes
    `poses` with SensorDataQoS, and a default RELIABLE subscription is
    incompatible with it -- it would connect to nothing and sit silent.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=None)
    node = Node("sync_initial_positions")

    msg_type = resolve_topic_type(node, topic)

    samples = defaultdict(list)
    count = [0]

    def on_poses(msg):
        for p in msg.poses:
            pos = p.pose.position
            samples[p.name].append((pos.x, pos.y, pos.z))
        count[0] += 1

    node.create_subscription(msg_type, topic, on_poses, qos_profile_sensor_data)

    deadline = node.get_clock().now().nanoseconds * 1e-9 + timeout
    while rclpy.ok() and count[0] < n_samples:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds * 1e-9 > deadline:
            break

    got = count[0]
    node.destroy_node()
    rclpy.shutdown()

    if got == 0:
        return None, None, 0

    means, jitter = {}, {}
    for name, pts in samples.items():
        n = len(pts)
        mean = tuple(sum(p[k] for p in pts) / n for k in range(3))
        # jitter = worst per-axis spread across the window
        jitter[name] = max(
            max(p[k] for p in pts) - min(p[k] for p in pts) for k in range(3))
        means[name] = mean
    return means, jitter, got


def resolve_topic_type(node, topic):
    """Pick NamedPoseArray vs NamedPoseArrayV2 by asking the live graph.

    motion_capture_tracking publishes either one depending on `poses_version`
    in motion_capture.yaml, and the two are NOT interchangeable on the wire.
    """
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    try:
        from motion_capture_tracking_interfaces.msg import NamedPoseArrayV2
    except ImportError:
        NamedPoseArrayV2 = None

    known = {"motion_capture_tracking_interfaces/msg/NamedPoseArray": NamedPoseArray}
    if NamedPoseArrayV2 is not None:
        known["motion_capture_tracking_interfaces/msg/NamedPoseArrayV2"] = NamedPoseArrayV2

    for info in node.get_publishers_info_by_topic(topic):
        t = info.topic_type
        if t in known:
            return known[t]
        print(f"WARNING: {topic} has unexpected type {t}; assuming NamedPoseArray")
    return NamedPoseArray


# --- checks -----------------------------------------------------------------
def run_checks(enabled, means, jitter, args):
    """Return a list of refusal strings (empty == safe to write)."""
    problems = []

    missing = [n for n in enabled if n not in means]
    if missing:
        problems.append(
            "not streamed by Motive: " + ", ".join(sorted(missing)) +
            "\n    The rigid body name in Motive must match the yaml key exactly "
            "(case-sensitive).\n    Streaming now: " +
            (", ".join(sorted(means)) or "<nothing>"))

    for name in sorted(set(enabled) & set(means)):
        x, y, z = means[name]
        if any(math.isnan(v) or math.isinf(v) for v in (x, y, z)):
            problems.append(f"{name}: non-finite position {means[name]} (occluded?)")
            continue
        if jitter[name] > JITTER_ABORT_M:
            problems.append(
                f"{name}: moving/unstable -- {jitter[name] * 1000:.1f} mm spread over the "
                f"sample window (limit {JITTER_ABORT_M * 1000:.0f} mm). "
                "Drones must be at rest on the floor.")
        if abs(z) > Z_SANITY_M:
            problems.append(
                f"{name}: z = {z:.3f} m, expected a grounded drone near 0. "
                "Wrong rigid body, or the Motive ground plane is off.")

    present = sorted(set(enabled) & set(means))
    for i, a in enumerate(present):
        for b in present[i + 1:]:
            d = math.dist(means[a][:2], means[b][:2])
            if d < MIN_SEPARATION_M:
                problems.append(
                    f"{a} and {b} are only {d:.2f} m apart (rig rule: "
                    f">= {MIN_SEPARATION_M:.1f} m). Same-trajectory flight preserves "
                    "start separation, so this spacing carries into the air.")

    extra = sorted(set(means) - set(enabled))
    if extra:
        print("note: streamed but not enabled in the yaml: " + ", ".join(extra))

    return problems


# --- main -------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Update initial_position in crazyflies.yaml from live /poses.")
    ap.add_argument("--yaml", default=os.path.normpath(DEFAULT_YAML))
    ap.add_argument("--only", action="store_true",
                    help="write only --yaml, not the other copies of "
                         "crazyflies.yaml in this workspace")
    ap.add_argument("--topic", default="/poses")
    ap.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    ap.add_argument("--dry-run", action="store_true", help="show the diff, write nothing")
    ap.add_argument("--yes", "-y", action="store_true", help="apply without prompting")
    ap.add_argument("--with-z", action="store_true",
                    help="also write the measured z (default: keep the yaml's z, "
                         "normally 0 -- the drones are on the floor)")
    ap.add_argument("--force", action="store_true",
                    help="write even if a safety check fails (you own the outcome)")
    args = ap.parse_args()

    if not os.path.exists(args.yaml):
        sys.exit(f"no such file: {args.yaml}")

    enabled = read_enabled(args.yaml)
    if not enabled:
        sys.exit("no robots with `enabled: true` in " + args.yaml)
    print(f"enabled in yaml ({len(enabled)}): {', '.join(sorted(enabled))}")
    print(f"sampling {args.topic} for {args.samples} frames "
          f"(timeout {args.timeout:g} s) ...")

    means, jitter, got = collect_poses(args.topic, args.samples, args.timeout)
    if not got:
        print(f"\nNo messages on {args.topic} within {args.timeout:g} s.", file=sys.stderr)
        print("  * Is the mocap node running?   ros2 node list | grep motion_capture\n"
              "  * Publishers on the topic?     ros2 topic info -v /poses\n"
              "  * Known rig failure: a leftover process holding UDP 1511 starves the\n"
              "    mocap node silently -- check `ss -uanp | grep :1511`, kill the orphan,\n"
              "    relaunch. (Ping to the Motive PC proves nothing: unicast != multicast.)",
              file=sys.stderr)
        return 2
    if got < args.samples:
        print(f"warning: only {got}/{args.samples} frames arrived -- mocap rate low?")

    problems = run_checks(enabled, means, jitter, args)

    updates = {n: means[n] for n in enabled if n in means}
    if not updates:
        print("\nREFUSED: none of the enabled drones are being streamed.", file=sys.stderr)
        for p in problems:
            print("  - " + p, file=sys.stderr)
        return 1

    new_text, changed = rewrite_positions(args.yaml, updates, keep_z=not args.with_z)

    print("\n  drone      yaml now                 mocap says               delta")
    print("  " + "-" * 70)
    any_move = False
    for name in sorted(changed):
        old, new = changed[name]
        d = math.dist(old[:2], new[:2])
        any_move = any_move or d > 0.0005
        print(f"  {name:<10} [{old[0]:>7.3f},{old[1]:>7.3f}]      "
              f"[{new[0]:>7.3f},{new[1]:>7.3f}]      {d * 100:6.1f} cm"
              f"{'  <-- CHECK PLACEMENT' if d > 0.5 else ''}")
    print(f"\n  (jitter over window: max "
          f"{max(jitter[n] for n in changed) * 1000:.1f} mm)")

    if problems:
        print("\n" + ("SAFETY CHECKS FAILED:" if not args.force else
                      "SAFETY CHECKS FAILED (overridden by --force):"), file=sys.stderr)
        for p in problems:
            print("  - " + p, file=sys.stderr)
        if not args.force:
            print("\nRefusing to write. Fix the placement/tracking, or re-run with "
                  "--force if you are certain.", file=sys.stderr)
            return 1

    if not any_move:
        print("\nAlready in sync (all deltas < 0.5 mm); nothing to write.")
        return 0

    if args.dry_run:
        print("\n--dry-run: nothing written.")
        return 0

    if not args.yes:
        try:
            if input("\nWrite these positions to the yaml? [y/N] ").strip().lower() not in ("y", "yes"):
                print("aborted; nothing written.")
                return 0
        except EOFError:
            print("\nnot a tty and --yes not given; nothing written.")
            return 0

    with open(args.yaml, "w") as f:
        f.write(new_text)
    print(f"\nwrote {args.yaml}")

    for other in ([] if args.only else sibling_yamls(args.yaml)):
        try:
            other_enabled = read_enabled(other)
        except Exception as e:                                  # noqa: BLE001
            print(f"  ! could not read {other} ({e}); left alone")
            continue
        if set(other_enabled) != set(enabled):
            print(f"  ! {other} has a different enabled fleet "
                  f"({sorted(other_enabled)}); left alone deliberately")
            continue
        other_text, _ = rewrite_positions(other, updates, keep_z=not args.with_z)
        with open(other, "w") as f:
            f.write(other_text)
        print(f"wrote {other}")
    print("The crazyflie server reads this file only at launch -- (re)start it now "
          "for the new positions to take effect.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
