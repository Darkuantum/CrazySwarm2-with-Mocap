"""The command catalog: every button in the console is one entry in here.

Design rule for this file: **the argv built here is the argv executed, and it is
what the UI shows you.** No hidden wrapper, no extra environment munging at call
time (the environment is set once, in run.sh, exactly as you would set it in a
terminal). So a command you learn here is a command you can type.

Each action carries `why` (what it does / when to use it) and `teaches` (how to
read the command line itself), because the point is to be able to stop using
the GUI.
"""

from __future__ import annotations

import ast
import glob
import os
import re
import shlex

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# Preset payloads for the service-call actions. Keeping them here means the UI
# shows the real ros2 service call YAML, which is the fiddly part to learn.
TAKEOFF_HEIGHT, TAKEOFF_SEC = 0.5, 3
LAND_HEIGHT, LAND_SEC = 0.04, 4


def _num(value, default):
    """A number field the user has cleared must not produce '{height: }' -- that
    is invalid YAML and the service call would fail with a parser error."""
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return float(default)


def _duration(sec):
    sec = float(sec)
    whole = int(sec)
    return f'{{sec: {whole}, nanosec: {int(round((sec - whole) * 1e9))}}}'


def action(id, group, label, build, why, teaches='', kind='task', danger='none',
           requires='any', params=(), docs='', confirm=''):
    return {
        'id': id, 'group': group, 'label': label, 'build': build, 'why': why,
        'teaches': teaches, 'kind': kind, 'danger': danger, 'requires': requires,
        'params': list(params), 'docs': docs, 'confirm': confirm,
    }


def p(name, label, type='text', default='', options=None, help='', descriptions=None,
      grouped=False):
    # grouped: options are "<group> <item>" and render as <optgroup>s, so a
    # script list reads as package headings instead of a long prefixed string.
    return {'name': name, 'label': label, 'type': type, 'default': default,
            'options': options or [], 'help': help,
            'descriptions': descriptions or {}, 'grouped': grouped}


def _bashc(script):
    """A shell one-liner, shown verbatim. Used only where a pipe or loop is the
    honest answer (`ros2 service list | grep ...`), never to hide anything."""
    return ['bash', '-c', script]


# ------------------------------------------------------------ discovery
# Flight scripts are FOUND, not listed. Any package in this workspace that
# depends on crazyflie_py is a package of flight scripts (crazyflie_examples,
# crazyflie_shows, whatever you drop in next); its executables are what
# `ros2 pkg executables <pkg>` would print. Build a new show package and it
# shows up here with no console change.
#
# Descriptions come from each script's own module docstring, read with `ast`
# -- never imported, because importing a flight script can have side effects.

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
INSTALL = os.path.join(REPO, 'install')

# Hand-written notes win over docstrings where the safety detail matters.
SCRIPT_NOTES = {
    'hello_world': 'Arms the FIRST enabled drone, takes off to 0.5 m, hovers 5 s, '
                   'lands, disarms. The standard first flight.',
    'arming': 'GROUND TEST: arms every drone and spins the props at ~15% for 10 s. '
              'Props spin -- keep fingers clear.',
    'multi_trajectory': 'Every drone flies the same relative traj1.csv (~50 s) and '
                        'returns over its own initial_position.',
    'multi_trajectory_formation': 'Waypoint tour, pentagon gather, 360 spin, triangle '
                                  'morph, rigid orbit at R=1.2 m (~58 s). Keep a '
                                  '~2.24 m radius around the room centre clear.',
    'figure8': 'Classic single-drone figure-eight.',
    'nice_hover': 'Smooth hover demo.',
    'swap': 'Drones exchange positions -- watch the clearances.',
    'teleop_xbox': 'Gamepad flight for the first drone, from /dev/input/js0. Launch '
                   'the stack with teleop:=False first: one owner per controller.',
    'swarm_show': 'The full show. Refuses to arm if any live pose is >0.25 m from '
                  'initial_position. Run plan_show first.',
}
# Executables that never command a drone: offered as ground checks, no
# "drones will move" confirmation.
GROUND = {'color_led', 'set_param'}


def _is_ground(name):
    return name in GROUND or name.startswith('plan_') or name.endswith('.sh')


def _first_para(text):
    text = (text or '').strip()
    return ' '.join(text.split('\n\n')[0].split()) if text else ''


def _script_doc(path):
    """Module docstring (python) or leading comment block (shell)."""
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            src = fh.read()
    except OSError:
        return ''
    if path.endswith('.py'):
        try:
            return _first_para(ast.get_docstring(ast.parse(src)))
        except SyntaxError:
            return ''
    lines = []
    for ln in src.splitlines()[1:]:
        if not ln.startswith('#'):
            if lines:
                break
            continue
        lines.append(ln.lstrip('# ').rstrip())
    return _first_para('\n'.join(lines))


def _entry_points(pkg_prefix):
    """{script: module_file} from the package's egg-info, if it has one."""
    out = {}
    for ep in glob.glob(os.path.join(pkg_prefix, '**', '*.egg-info', 'entry_points.txt'),
                        recursive=True):
        dist = os.path.dirname(os.path.dirname(ep))
        section = None
        with open(ep, encoding='utf-8') as fh:
            for raw in fh:
                line = raw.strip()
                if line.startswith('['):
                    section = line
                elif section == '[console_scripts]' and '=' in line:
                    name, target = (x.strip() for x in line.split('=', 1))
                    module = target.split(':')[0]
                    out[name] = os.path.join(dist, *module.split('.')) + '.py'
    return out


def discover_scripts():
    """Every runnable script in every crazyflie_py-dependent package.

    -> list of {pkg, name, ground, doc, live}. `live` is False when the package
    was built after this console started: its install prefix is not on the
    console's AMENT_PREFIX_PATH yet, so `ros2 run` would say "Package not
    found" until the console is restarted from a freshly sourced shell.
    """
    ament = set(os.environ.get('AMENT_PREFIX_PATH', '').split(':'))
    found = []
    for xml in sorted(glob.glob(os.path.join(INSTALL, '*', 'share', '*', 'package.xml'))):
        pkg = os.path.basename(os.path.dirname(xml))
        try:
            body = open(xml, encoding='utf-8').read()
        except OSError:
            continue
        if not re.search(r'<(?:exec_|build_)?depend>\s*crazyflie_py\s*<', body):
            continue
        prefix = os.path.join(INSTALL, pkg)
        libdir = os.path.join(prefix, 'lib', pkg)
        if not os.path.isdir(libdir):
            continue
        eps = _entry_points(prefix)
        for name in sorted(os.listdir(libdir)):
            path = os.path.join(libdir, name)
            if not (os.path.isfile(path) and os.access(path, os.X_OK)):
                continue
            doc = SCRIPT_NOTES.get(name) or _script_doc(eps.get(name, path)) \
                or '(no description -- give the script a module docstring)'
            found.append({'pkg': pkg, 'name': name, 'ground': _is_ground(name),
                          'doc': doc, 'live': prefix in ament})
    # your own show packages first, the stock examples after
    found.sort(key=lambda d: (d['pkg'] == 'crazyflie_examples', d['pkg'], d['name']))
    return found


def scripts_signature():
    """Cheap change detector: the executables on disk, for the catalog cache."""
    sig = []
    for d in sorted(glob.glob(os.path.join(INSTALL, '*', 'lib', '*'))):
        try:
            sig.append((d, os.path.getmtime(d), tuple(sorted(os.listdir(d)))))
        except OSError:
            pass
    return tuple(sig)


def _script_param(scripts, label, default_name):
    opts = [f"{d['pkg']} {d['name']}" for d in scripts]
    desc = {}
    for d in scripts:
        text = d['doc']
        if not d['live']:
            text = ('BUILT AFTER THIS CONSOLE STARTED -- restart the console from a '
                    'sourced shell before running it. ' + text)
        desc[f"{d['pkg']} {d['name']}"] = text
    default = next((o for o in opts if o.endswith(' ' + default_name)), opts[0] if opts else '')
    return p('script', label, 'select', default, opts, descriptions=desc, grouped=True)


def build_catalog(fleet):
    """fleet: configio.fleet_summary() -- lets per-drone actions be real commands."""
    enabled = [d for d in fleet if d['enabled']]
    drone_names = [d['name'] for d in enabled] or ['cf1']
    first = drone_names[0]
    addr_list = ' '.join(d['address'] or '' for d in enabled if d['address'])
    acts = []

    # ---------------------------------------------------------------- launch
    def launch_build(v):
        argv = ['ros2', 'launch', 'crazyflie', 'launch.py']
        for key in ('backend', 'rviz', 'preflight', 'foxglove', 'teleop', 'mocap', 'gui', 'debug'):
            argv.append(f'{key}:={v.get(key)}')
        if v.get('mocap_hostname'):
            argv.append(f'mocap_hostname:={v["mocap_hostname"]}')
        return argv

    acts.append(action(
        'launch.stack', 'Launch', 'Start the stack', launch_build,
        why='Starts everything in one process tree: mocap tracking, the crazyflie '
            'server (which owns the Crazyradio), RViz, the preflight GUI and the '
            'Foxglove bridge. This is "terminal 1" in docs/RUNNING.md section B.',
        teaches='`ros2 launch <package> <file>` runs a launch file. Everything after '
                'it is a launch ARGUMENT in name:=value form -- that is launch syntax, '
                'not shell syntax, so there are no spaces around :=.',
        kind='service', requires='server_stopped', docs='docs/RUNNING.md',
        params=[
            p('backend', 'backend', 'select', 'cpp', ['cpp', 'cflib', 'sim'],
              'cpp = the real drones over the Crazyradio. sim = no hardware, no mocap.'),
            p('mocap', 'mocap', 'select', 'True', ['True', 'False'],
              'False skips motion_capture_tracking entirely (no /poses).'),
            p('rviz', 'rviz', 'select', 'True', ['True', 'False']),
            p('preflight', 'preflight GUI', 'select', 'True', ['True', 'False']),
            p('foxglove', 'foxglove bridge', 'select', 'True', ['True', 'False'],
              'Not a window: connect Foxglove Studio to ws://localhost:8765.'),
            p('teleop', 'teleop + joy', 'select', 'True', ['True', 'False'],
              'Set False when a script wants sole control of the drones.'),
            p('gui', 'upstream gui.py', 'select', 'False', ['True', 'False']),
            p('debug', 'debug (gdb in xterm)', 'select', 'False', ['True', 'False']),
            p('mocap_hostname', 'Motive IP override', 'text', '', None,
              'Empty = use motion_capture.yaml ("auto" there = NatNet discovery ping).'),
        ]))

    acts.append(action(
        'launch.sim', 'Launch', 'Start the simulator stack',
        lambda v: ['ros2', 'launch', 'crazyflie', 'launch.py', 'backend:=sim'],
        why='No hardware and no mocap: the physics runs in crazyflie_sim. Needs the '
            'cffirmware Python bindings (./scripts/setup_sim_firmware.sh, once).',
        teaches='Same launch file, one argument different. Flight scripts then need '
                '--ros-args -p use_sim_time:=true, because the sim clock runs ~4x '
                'slower than wall time.',
        kind='service', requires='server_stopped', docs='docs/RUNNING.md#a-simulation-no-hardware-no-mocap'))

    acts.append(action(
        'launch.stop', 'Launch', 'Stop the stack', 
        lambda v: (['./console/stop_stack.sh'] + (['--hard'] if v.get('hard') == 'True' else [])),
        why='The stop that actually works. `ros2 launch` exits on Ctrl-C but its '
            'children do not always follow -- on this rig the mocap node, RViz and '
            'the preflight GUI routinely survive, and the mocap node holds UDP 1511 '
            'while blocked in recv(). The next launch then starves silently with no '
            'error. This signals every stack process directly, escalates '
            'SIGINT -> SIGTERM -> SIGKILL, and then VERIFIES 1511 is released.',
        teaches='Killing a `ros2 launch` wrapper does not kill the node binaries it '
                'started. Signal the processes themselves, then confirm the port is '
                'free -- `ss -uanp | grep :1511` is the check that matters, because a '
                'leftover there breaks the NEXT run, not this one.',
        requires='any', danger='flight', docs='docs/TROUBLESHOOTING.md',
        confirm='Stop the stack? If drones are in the air this removes their commander '
                '-- land them first.',
        params=[
            p('hard', 'skip to SIGKILL', 'select', 'False', ['False', 'True'],
              'False tries SIGINT first (what you want). True is for a stack already '
              'hung and ignoring signals.'),
        ]))

    acts.append(action(
        'build.workspace', 'Launch', 'Rebuild the workspace',
        lambda v: ['./scripts/build.sh'] + ([v['package']] if v.get('package') else []),
        why='colcon build wrapper. Run it after editing anything under src/, then '
            're-source install/setup.bash (restart this console to pick it up).',
        teaches='build.sh wraps `colcon build --symlink-install` and defends against '
                'the two traps in this repo: `set -u` vs ROS setup.bash, and a conda '
                'python shadowing /usr/bin/python3.',
        kind='task', params=[p('package', 'only this package (optional)', 'text', '')],
        docs='README.md'))

    # ------------------------------------------------------------ preflight
    acts.append(action(
        'check.scan_all', 'Preflight', 'Scan every enabled drone', lambda v: _bashc(
            'for a in %s; do echo "== $a"; ros2 run crazyflie scan --address $a; done' % addr_list),
        why='THE go/no-go check. The server connects drones in lexicographic order and '
            'blocks FOREVER, silently, on the first enabled drone that does not answer '
            'radio -- one dead drone kills the whole launch with no error message. Scan '
            'before every launch. An address that prints no radio:// URI is not flyable: '
            'fix the URI or set enabled: false.',
        teaches='`ros2 run <package> <executable> [args]` runs one binary from a package. '
                'scan sweeps channels/datarates at one address and prints the URIs that '
                'answered -- which is also how you discover a datarate mismatch (a 2M URI '
                'on a 1M drone hangs the server forever).',
        requires='server_stopped', docs='docs/TROUBLESHOOTING.md#crazyradio--drones'))

    acts.append(action(
        'check.scan_one', 'Preflight', 'Scan one address',
        lambda v: ['ros2', 'run', 'crazyflie', 'scan', '--address', v.get('address', '0xE7E7E7E7E7')],
        why='Same scan for a single drone. Use the factory address 0xE7E7E7E7E7 to find '
            'a drone whose address you do not know.',
        teaches='The reply tells you channel AND datarate, e.g. radio://*/90/1M/... -- '
                'both must match the uri in crazyflies.yaml.',
        requires='server_stopped',
        params=[p('address', 'address', 'select',
                  (enabled[0]['address'] if enabled and enabled[0]['address'] else '0xE7E7E7E7E7'),
                  [d['address'] for d in enabled if d['address']] + ['0xE7E7E7E7E7'])]))

    acts.append(action(
        'check.radio_usb', 'Preflight', 'Is the Crazyradio plugged in?',
        lambda v: _bashc('lsusb | grep -iE "crazyradio|bitcraze|1915:|35d2:" '
                         '|| echo "no Crazyradio on USB"'),
        why='The dongle is the only path to the drones. If this is empty, nothing else '
            'in the radio chain can work.',
        teaches='lsusb lists USB devices; the grep just filters to Bitcraze vendor IDs.'))

    acts.append(action(
        'check.mocap_socket', 'Preflight', 'Is UDP 1511 clear?',
        lambda v: _bashc('ss -uanp | grep ":1511" || echo "nothing bound to 1511 '
                         '(expected when the mocap node is not running)"'),
        why='The confirmed cause of "mocap suddenly died" on this rig: a leftover process '
            'still bound to UDP 1511. With two SO_REUSEPORT sockets the kernel gives each '
            'NatNet datagram to only ONE of them, so the mocap node starves and hangs '
            'silently inside connect(). Two sockets here = that bug.',
        teaches='ss -uanp = UDP, all sockets, numeric, with the owning process.',
        docs='docs/TROUBLESHOOTING.md#mocap-pipeline'))

    acts.append(action(
        'check.mocap_procs', 'Preflight', 'Leftover mocap processes?',
        lambda v: _bashc('pgrep -af motion_capture_tracking || echo "none (good)"'),
        why='A frozen mocap node ignores SIGINT (it is blocked in recv), and a leftover '
            'makes the NEXT connect abort with SIGABRT. Check before relaunching.',
        teaches='pgrep -af = find processes by command line, print the full line.'))

    acts.append(action(
        'check.pkg_prefix', 'Preflight', 'Which motion_capture_tracking resolves?',
        lambda v: ['ros2', 'pkg', 'prefix', 'motion_capture_tracking'],
        why='It must print this workspace\'s install/ directory. If it prints /opt/ros/..., '
            'the apt package (1.0.9) is shadowing the vendored driver -- and that release '
            'hard-codes a foreign interface IP into the NatNet socket, so /poses stays '
            'silent forever.',
        teaches='`ros2 pkg prefix <pkg>` prints where a package was found. It is the '
                'quickest way to catch an overlay/underlay shadowing problem.',
        docs='src/motion_capture_tracking/VENDORED.md'))

    acts.append(action(
        'check.nodes', 'Preflight', 'List running nodes',
        lambda v: ['ros2', 'node', 'list'],
        why='Who is alive on the ROS graph right now.',
        teaches='Expect /crazyflie_server, /motion_capture_tracking, and one node per '
                'helper. A node missing here that you started means it died or hung '
                'before it finished initialising.'))

    acts.append(action(
        'check.services', 'Preflight', 'Are the /all/* services up?',
        lambda v: _bashc('ros2 service list | grep -E "/all/|takeoff|land|go_to|arm|emergency"'),
        why='If /crazyflie_server is running but /all/takeoff never appears, the server is '
            'stuck mid-connect on an unreachable drone. That is the single most common '
            '"it just hangs" failure on this rig.',
        teaches='`ros2 service list` prints every advertised service; the grep narrows it. '
                'Add `--show-types` to see the srv type of each one.'))

    acts.append(action(
        'check.poses_hz', 'Preflight', 'Measure /poses rate',
        lambda v: ['ros2', 'topic', 'hz', '/poses', '--window', '50'],
        why='The mocap heartbeat. Should read ~50 Hz (Motive streams at 50 on this rig). '
            'Nothing at all = the mocap chain is broken upstream; a decaying rate = '
            'network trouble.',
        teaches='`ros2 topic hz <topic>` samples arrival times until you stop it; '
                '--window sets how many samples it averages over.',
        kind='service'))

    acts.append(action(
        'check.poses_info', 'Preflight', 'Who publishes /poses?',
        lambda v: ['ros2', 'topic', 'info', '/poses', '--verbose'],
        why='0 publishers means the mocap node is dead or starved -- not a Motive problem '
            'but a local one. 1 publisher with no data means it is blocked in connect().',
        teaches='`ros2 topic info -v` also prints each endpoint\'s QoS. /poses is '
                'BEST_EFFORT (sensor data): a RELIABLE subscriber silently matches nothing.'))

    acts.append(action(
        'check.pose_once', 'Preflight', 'Echo one onboard pose',
        lambda v: ['ros2', 'topic', 'echo', f'/{v.get("drone", first)}/pose', '--once'],
        why='The drone\'s own EKF estimate (via firmware logging). Compare it with the '
            'mocap pose in /poses: they should agree.',
        teaches='--once prints a single message and exits. Without it, echo streams '
                'forever (Ctrl-C, or the Stop button here).',
        params=[p('drone', 'drone', 'select', first, drone_names)]))

    acts.append(action(
        'check.status_once', 'Preflight', 'Echo one status message',
        lambda v: ['ros2', 'topic', 'echo', f'/{v.get("drone", first)}/status', '--once'],
        why='Battery voltage, supervisor bits (armed / tumbled / can-fly) and radio link '
            'health for one drone. "Won\'t arm" is usually answered here.',
        teaches='This topic exists only because status is enabled under '
                'all.firmware_logging.default_topics in crazyflies.yaml.',
        params=[p('drone', 'drone', 'select', first, drone_names)]))

    acts.append(action(
        'check.connstats', 'Preflight', 'Echo link statistics',
        lambda v: ['ros2', 'topic', 'echo',
                   f'/{v.get("drone", first)}/connection_statistics', '--once'],
        why='Unicast latency and ack/receive rates. Rising latency means the radio is '
            'saturated -- lower the logging rates.',
        params=[p('drone', 'drone', 'select', first, drone_names)]))

    acts.append(action(
        'check.battery', 'Preflight', 'Read a battery over the radio',
        lambda v: ['ros2', 'run', 'crazyflie', 'battery', '--uri',
                   next((d['uri'] for d in enabled if d['name'] == v.get('drone')),
                        enabled[0]['uri'] if enabled else 'radio://0/80/2M/E7E7E7E701')],
        why='Reads the pack voltage straight over the link, with no server running. '
            'Useful when you want a voltage before committing to a launch.',
        teaches='The crazyflie_tools binaries (scan, battery, reboot, console, listParams) '
                'talk to the dongle directly -- so the server must be STOPPED: only one '
                'process can own a Crazyradio.',
        requires='server_stopped',
        params=[p('drone', 'drone', 'select', first, drone_names)]))

    # ------------------------------------------------------------- position
    acts.append(action(
        'pos.sync_dry', 'Fleet position', 'Preview initial_position sync',
        lambda v: ['python3', 'scripts/sync_initial_positions.py', '--dry-run'],
        why='Samples the live /poses stream and shows what initial_position WOULD become '
            'for every enabled drone. Mocap must be up; the server should not be.',
        teaches='initial_position seeds the onboard estimate at connect. It must come '
                'from /poses (the mocap\'s view), never from /cfX/pose -- the onboard '
                'estimate was seeded by this same yaml, so copying it back is circular.',
        docs='docs/MOCAP.md#2b-setting-initial_position-from-poses'))

    acts.append(action(
        'pos.sync_apply', 'Fleet position', 'Apply initial_position sync',
        lambda v: ['python3', 'scripts/sync_initial_positions.py', '--yes'],
        why='Writes the measured positions into crazyflies.yaml (comments preserved). It '
            'REFUSES if a drone is not streamed, is moving, sits above 0.5 m, or if two '
            'drones are under 1 m apart. Restart the server afterwards -- the yaml is '
            'read only at launch.',
        confirm='This rewrites crazyflies.yaml from the live mocap. Continue?',
        docs='docs/MOCAP.md#2b-setting-initial_position-from-poses'))

    # ---------------------------------------------------------------- flight
    # One card per KIND of run, not per script: the script is a dropdown filled
    # by discover_scripts(), so a newly built show package appears here without
    # touching this file. Flight scripts and ground checks are split because
    # only one of them should ask "area clear?".
    scripts = discover_scripts()
    flights = [d for d in scripts if not d['ground']]
    grounds = [d for d in scripts if d['ground']]

    def _split(v, fallback):
        pkg, _, name = (v.get('script') or fallback).partition(' ')
        return pkg, name

    def run_flight(v):
        pkg, name = _split(v, '')
        argv = ['ros2', 'run', pkg, name]
        if v.get('sim') == 'yes':
            argv += ['--ros-args', '-p', 'use_sim_time:=true']
        return argv

    if flights:
        acts.append(action(
            'fly.script', 'Flight', 'Run a flight script', run_flight,
            why='Every flight script in this workspace, found by scanning each package '
                'that depends on crazyflie_py -- crazyflie_examples, your crazyflie_shows, '
                'and whatever you build next. The server must already be running and the '
                'preflight checklist cleared for every drone that will fly.',
            teaches='`ros2 run <package> <executable>` runs a console_scripts entry point '
                    '(declared in that package\'s setup.cfg). In SIMULATION add '
                    '--ros-args -p use_sim_time:=true or the script races ahead of the '
                    '~4x slower sim clock; on hardware never add it.',
            danger='flight', requires='server_running',
            confirm='The drones will move. Area clear, everyone back, hand near the E-STOP?',
            params=[
                _script_param(flights, 'script', 'hello_world'),
                p('sim', 'simulation clock', 'select', 'no', ['no', 'yes'],
                  'yes ONLY with backend:=sim.'),
            ],
            docs='docs/RUNNING.md#multi-drone-trajectory-demos'))

    if grounds:
        acts.append(action(
            'tool.script', 'Preflight', 'Run a ground check',
            lambda v: ['ros2', 'run', *_split(v, '')],
            why='Scripts in the same packages that never command a drone: plan_show '
                '(verifies a show against the current yaml, no ROS needed), the mocap '
                'diagnostics, LED and parameter helpers.',
            teaches='Same `ros2 run <package> <executable>` as a flight -- the only '
                    'difference is what the script does, so no "area clear" check.',
            requires='any',
            params=[_script_param(grounds, 'script', 'plan_show')]))

    # -------------------------------------------------------------- services
    acts.append(action(
        'srv.estop', 'Commands', 'E-STOP (all drones)',
        lambda v: ['ros2', 'service', 'call', '/all/emergency', 'std_srvs/srv/Empty', '{}'],
        why='Cuts the motors of every drone immediately. They fall. This is the same call '
            'the preflight GUI\'s e key makes.',
        teaches='`ros2 service call <service> <type> <yaml-args>`. Empty takes no fields, '
                'hence the bare {}.',
        danger='estop', requires='server_running'))

    acts.append(action(
        'srv.arm_all', 'Commands', 'Arm all',
        lambda v: ['ros2', 'service', 'call', '/all/arm', 'crazyflie_interfaces/srv/Arm',
                   '{arm: true}'],
        why='Arms every connected drone. Required before takeoff on current firmware.',
        teaches='The last argument is YAML for the request fields -- here the single '
                'bool `arm` from Arm.srv.',
        danger='flight', requires='server_running'))

    acts.append(action(
        'srv.disarm_all', 'Commands', 'Disarm all',
        lambda v: ['ros2', 'service', 'call', '/all/arm', 'crazyflie_interfaces/srv/Arm',
                   '{arm: false}'],
        why='Disarms every drone. Safe to call on the ground.',
        requires='server_running'))

    acts.append(action(
        'srv.takeoff_all', 'Commands', 'Takeoff (all)',
        lambda v: ['ros2', 'service', 'call', '/all/takeoff',
                   'crazyflie_interfaces/srv/Takeoff',
                   '{group_mask: 0, height: %s, duration: %s}' % (
                       _num(v.get('height'), TAKEOFF_HEIGHT),
                       _duration(_num(v.get('duration'), TAKEOFF_SEC)))],
        why='Broadcast takeoff to every drone. Arm first.',
        teaches='builtin_interfaces/Duration is a nested message, so it is a nested YAML '
                'mapping: duration: {sec: 3, nanosec: 0}. group_mask 0 = all groups.',
        danger='flight', requires='server_running',
        confirm='All drones will take off. Area clear?',
        params=[p('height', 'height [m]', 'number', TAKEOFF_HEIGHT),
                p('duration', 'duration [s]', 'number', TAKEOFF_SEC)]))

    acts.append(action(
        'srv.land_all', 'Commands', 'Land (all)',
        lambda v: ['ros2', 'service', 'call', '/all/land',
                   'crazyflie_interfaces/srv/Land',
                   '{group_mask: 0, height: %s, duration: %s}' % (
                       _num(v.get('height'), LAND_HEIGHT),
                       _duration(_num(v.get('duration'), LAND_SEC)))],
        why='Broadcast land. A longer duration means a gentler descent.',
        danger='flight', requires='server_running',
        params=[p('height', 'touchdown height [m]', 'number', LAND_HEIGHT),
                p('duration', 'duration [s]', 'number', LAND_SEC)]))

    for verb, srv, payload, why in (
        ('takeoff', 'Takeoff', '{group_mask: 0, height: %s, duration: %s}' % (
            TAKEOFF_HEIGHT, _duration(TAKEOFF_SEC)), 'Takeoff for ONE drone.'),
        ('land', 'Land', '{group_mask: 0, height: %s, duration: %s}' % (
            LAND_HEIGHT, _duration(LAND_SEC)), 'Land ONE drone.'),
    ):
        acts.append(action(
            f'srv.{verb}_one', 'Commands', f'{verb.capitalize()} (one drone)',
            (lambda vb, sv, pl: lambda v: [
                'ros2', 'service', 'call', f'/{v.get("drone", first)}/{vb}',
                f'crazyflie_interfaces/srv/{sv}', pl])(verb, srv, payload),
            why=why + ' Per-drone services live in the drone\'s namespace, e.g. /cf1/takeoff.',
            danger='flight', requires='server_running',
            confirm=f'{verb} one drone. Area clear?',
            params=[p('drone', 'drone', 'select', first, drone_names)]))

    acts.append(action(
        'srv.goto', 'Commands', 'Go to (one drone)',
        lambda v: ['ros2', 'service', 'call', f'/{v.get("drone", first)}/go_to',
                   'crazyflie_interfaces/srv/GoTo',
                   '{group_mask: 0, relative: %s, goal: {x: %s, y: %s, z: %s}, '
                   'yaw: %s, duration: %s}' % (
                       'true' if v.get('relative') == 'yes' else 'false',
                       _num(v.get('x'), 0.0), _num(v.get('y'), 0.0), _num(v.get('z'), 1.0),
                       _num(v.get('yaw'), 0.0), _duration(_num(v.get('duration'), 3.0)))],
        why='Smooth move to a point. relative: true makes the goal an offset from where '
            'the drone is now -- the safer choice when you are unsure of the frame.',
        teaches='goal is a geometry_msgs/Point, so it nests as {x: , y: , z: }. Note the '
                'yaw field is commented as degrees in GoTo.srv while crazyflie_py passes '
                'radians -- check before you trust a large value.',
        danger='flight', requires='server_running',
        confirm='The drone will fly to that point. Area clear?',
        params=[p('drone', 'drone', 'select', first, drone_names),
                p('x', 'x [m]', 'number', 0.0), p('y', 'y [m]', 'number', 0.0),
                p('z', 'z [m]', 'number', 1.0), p('yaw', 'yaw', 'number', 0.0),
                p('duration', 'duration [s]', 'number', 3.0),
                p('relative', 'relative to current pose', 'select', 'yes', ['yes', 'no'])]))

    # ---------------------------------------------------------------- params
    acts.append(action(
        'param.reset_kalman', 'Parameters', 'Reset the Kalman filter (all)',
        lambda v: ['ros2', 'param', 'set', '/crazyflie_server',
                   'all.params.kalman.resetEstimation', '1'],
        why='Re-seeds every drone\'s estimator. Do this after moving drones by hand, and '
            'whenever the preflight error plot shows a constant offset. Follow it with the '
            'same param set back to 0 (the preflight GUI\'s r key does both).',
        teaches='`ros2 param set <node> <name> <value>`. The server maps '
                'all.params.<group>.<name> to a firmware parameter broadcast; '
                'cf1.params.<group>.<name> targets one drone.',
        requires='server_running', docs='docs/RUNNING.md#runtime-firmware-parameters'))

    acts.append(action(
        'param.reset_kalman_off', 'Parameters', 'Clear the Kalman reset flag',
        lambda v: ['ros2', 'param', 'set', '/crazyflie_server',
                   'all.params.kalman.resetEstimation', '0'],
        why='The second half of a reset: the firmware flag has to go back to 0.',
        requires='server_running'))

    acts.append(action(
        'param.list', 'Parameters', 'List server parameters',
        lambda v: _bashc('ros2 param list /crazyflie_server | head -n 200'),
        why='Every firmware parameter is exposed here because server.yaml sets '
            'query_all_values_on_connect: True. Piped through head because the list is long.',
        requires='server_running'))

    acts.append(action(
        'param.get', 'Parameters', 'Get a parameter',
        lambda v: ['ros2', 'param', 'get', '/crazyflie_server',
                   v.get('name', 'all.params.kalman.resetEstimation')],
        why='Read one parameter back from the server.',
        requires='server_running',
        params=[p('name', 'parameter', 'text', 'cf1.params.stabilizer.estimator')]))

    acts.append(action(
        'param.set', 'Parameters', 'Set any parameter',
        lambda v: ['ros2', 'param', 'set', '/crazyflie_server',
                   v.get('name', ''), str(v.get('value', ''))],
        why='Free-form firmware parameter push. These reach the drones because the '
            'vendored server applies them in an on-set-parameters callback (upstream\'s '
            '/parameter_events handler never fires on this rig).',
        danger='flight', requires='server_running',
        params=[p('name', 'parameter', 'text', 'cf1.params.ring.effect'),
                p('value', 'value', 'text', '7')]))

    acts.append(action(
        'led.set', 'Parameters', 'Set the Color LED deck',
        lambda v: ['./scripts/led.sh', str(v.get('color', '1'))],
        why='Convention on this rig: green = connected and ready, red = a script owns the '
            'drones / they are flying, dark = clean shutdown.',
        teaches='led.sh drives the `ros2` CLI on purpose: its long-running daemon keeps '
                'the ROS graph warm, while a fresh rclpy process can stall in DDS '
                'discovery on this machine.',
        requires='server_running',
        params=[p('color', 'colour', 'select', '1',
                  ['0', '1', '2', '3', '4', '5', '9'],
                  '0 off, 1 green, 2 red, 3 yellow, 4 blue, 5 purple, 9 white')],
        docs='docs/RUNNING.md#color-led-deck--status-convention-and-manual-control'))

    # ------------------------------------------------------------- recovery
    acts.append(action(
        'fix.kill_mocap', 'Recovery', 'Kill leftover mocap nodes',
        lambda v: _bashc('pkill -f motion_capture_tracking_node; sleep 1; '
                         'pgrep -af motion_capture_tracking || echo "clear"'),
        why='Killing a `ros2 run` wrapper does not kill the node binary, and a leftover '
            'holding UDP 1511 starves the next one. Run this, confirm 1511 is clear, then '
            'relaunch.',
        confirm='Kill every motion_capture_tracking process?'))

    acts.append(action(
        'fix.kill_stack', 'Recovery', 'Kill a hung stack',
        lambda v: _bashc('pkill -f "ros2 launch crazyflie"; pkill -f crazyflie_server; '
                         'pkill -f motion_capture_tracking_node; sleep 1; '
                         'pgrep -af "crazyflie_server|motion_capture_tracking" || echo clear'),
        why='For the documented silent hang: the server blocked mid-connect on an '
            'unreachable drone ignores gentler signals and needs killing outright.',
        confirm='Kill the running crazyflie stack outright?', danger='flight'))

    acts.append(action(
        'fix.daemon', 'Recovery', 'Restart the ros2 daemon',
        lambda v: _bashc('ros2 daemon stop; ros2 daemon start; ros2 daemon status'),
        why='If `ros2 node list` shows nothing while nodes are clearly running, the CLI '
            'daemon\'s cached graph is stale. This clears it.',
        teaches='The daemon is what makes the ros2 CLI fast; it caches the discovery '
                'graph so each command does not have to re-discover it.'))

    acts.append(action(
        'fix.env', 'Recovery', 'Show the ROS environment',
        lambda v: _bashc('echo "ROS_DISTRO=$ROS_DISTRO"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (default)}"; '
                         'echo "RMW=${RMW_IMPLEMENTATION:-default}"; echo "python3=$(command -v python3)"; '
                         'echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH" | tr ":" "\\n"'),
        why='Most "I see no topics" reports are an environment mismatch: a different '
            'ROS_DOMAIN_ID, an unsourced overlay, or a conda python.'))

    return acts


def catalog_index(acts):
    return {a['id']: a for a in acts}


def render(action_def, values):
    # Fill every declared param the caller left out with its default: the
    # compact dashboard buttons send only what they show.
    merged = {q['name']: q['default'] for q in action_def.get('params', [])}
    merged.update({k: v for k, v in (values or {}).items() if v is not None})
    argv = action_def['build'](merged)
    return argv, ' '.join(shlex.quote(a) for a in argv)


def public(acts):
    """Catalog without the un-JSON-able build callables."""
    return [{k: v for k, v in a.items() if k != 'build'} for a in acts]
