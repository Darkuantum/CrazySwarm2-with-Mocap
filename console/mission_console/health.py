"""The system-health graph: where, exactly, the stack is broken.

The flight stack is a chain -- Motive -> mocap node -> /poses -> crazyflie_server
-> radio -> each drone -- and every link fails in its own quiet way. Most of the
failures on this rig produce NO error text at all (the server blocking forever on
an unreachable drone; the mocap node starved by a leftover UDP 1511 socket; the
apt driver publishing an empty /poses), which is exactly why reading the launch
log is such a poor way to find them.

So this module probes each link directly and reports one node per link, with the
command it used to decide. Probes go through the `ros2` CLI rather than rclpy on
purpose: the CLI daemon keeps the ROS graph warm, while a fresh rclpy process on
this machine can stall in DDS discovery and never return -- and as a bonus every
probe is a command you can run yourself.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import socket
import struct
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

from . import configio
from .procs import run_capture

REPO = configio.REPO

OK, WARN, FAIL, BLOCKED, UNKNOWN, SKIP = 'ok', 'warn', 'fail', 'blocked', 'unknown', 'skip'
# SKIP ranks with OK: a check that does not apply (no radio in a sim run) must
# not colour the whole system amber.
RANK = {FAIL: 4, WARN: 3, UNKNOWN: 2, BLOCKED: 2, SKIP: 0, OK: 0}

NATNET_CMD_PORT = 1510
NATNET_CONNECT, NATNET_SERVERINFO, NATNET_DISCOVERY = 0, 1, 14

# Error text -> diagnosis. Matched against the output of everything the console
# runs, so a launch that fails tells you what it means instead of what it said.
SIGNATURES = [
    (r'Channels \d+ and \d+ are already served by Crazyradio',
     'radio.usb', FAIL, 'Two dongles on channels less than 2 apart',
     'crazyflie-link-cpp refuses the pair: a 2M channel is ~2 MHz wide, so adjacent '
     'channels overlap.',
     'Space the channels >= 2 apart in crazyflies.yaml (e.g. 80 and 90).'),
    (r'no Motive answered a NatNet discovery ping',
     'mocap.host', FAIL, 'Motive did not answer the discovery ping',
     'The launch aborted on purpose rather than letting the mocap node hang silently.',
     'Check you are on the Motive LAN (wired 192.168.9.x -- `ip route get <ip>` must not '
     'say "via"), that Motive is streaming, and that the Windows firewall allows NatNet. '
     'Or pin it: mocap_hostname:=<ip>.'),
    (r'motion_capture_tracking resolves to the apt package',
     'mocap.pkg', FAIL, 'The apt mocap driver is shadowing the vendored one',
     'apt 1.0.9 hard-codes interface IP 141.23.110.162 into the NatNet socket, so it '
     'never receives a frame.',
     'sudo apt remove ros-$ROS_DISTRO-motion-capture-tracking '
     'ros-$ROS_DISTRO-motion-capture-tracking-interfaces, rebuild, re-source.'),
    (r'Could not load the Qt platform plugin ["\']?xcb',
     'env.python', WARN, 'The preflight GUI could not start (Qt xcb plugin)',
     'PyQt6 needs libxcb-cursor0, which Ubuntu 22.04 does not install by default.',
     'sudo apt install libxcb-cursor0 (install_deps.sh does this).'),
    (r"No module named ['\"]?_?cffirmware",
     'env.python', FAIL, 'The simulator bindings are missing',
     'crazyflie_sim imports cffirmware, which is not a pip package.',
     './scripts/setup_sim_firmware.sh (once), and make sure conda is deactivated -- '
     'ROS runs nodes with /usr/bin/python3.'),
    (r'NoParameterOverrideProvided',
     'server.node', FAIL, 'A parameter was declared without a default',
     'declare_parameter(name, {}) picks the ParameterDescriptor overload, leaving the '
     'parameter with no default at all.',
     'Spell the default out, e.g. std::vector<std::string>{}.'),
    (r'process has died.*crazyflie_server',
     'server.node', FAIL, 'crazyflie_server died',
     'The server exited instead of staying up.',
     'Read its last lines in the Processes tab; a scan of every enabled address is the '
     'usual next step.'),
    (r'process has died.*motion_capture_tracking',
     'mocap.node', FAIL, 'The mocap node died',
     'It is set to respawn, so expect this to repeat every few seconds until the cause '
     'is fixed.',
     'Check the Motive address and that UDP 1511 is clear.'),
    (r'Not able to establish a connection|Timeout while connecting',
     'radio.drones', FAIL, 'A drone did not answer over the radio',
     'The server could not reach a drone it was told to connect to.',
     'Scan every enabled address; fix the URI (channel AND datarate) or set the drone '
     'enabled: false.'),
    (r'The message type .* is invalid',
     'env.python', FAIL, 'Message bindings were generated for the wrong Python',
     'A conda/venv interpreter shadowed /usr/bin/python3 during the build.',
     'Deactivate conda and rebuild with ./scripts/build.sh.'),
]


# The supervisor's infoBitfield, as crazyflie-firmware packs it in
# supervisor.c updateLogData(). crazyflie_interfaces/msg/Status.msg names only
# the first seven; bit 7 (isCrashed) is packed by the firmware but has no
# constant there, and drones in this lab have been seen reporting bits above
# that -- their firmware is newer than the reference checkout. So decode what we
# know and SAY SO about the rest rather than quietly dropping it: an unnamed bit
# on a drone you are about to arm is exactly the thing you want to be told.
SUPERVISOR_BITS = [
    (0x0001, 'can be armed'),
    (0x0002, 'armed'),
    (0x0004, 'auto-arm'),
    (0x0008, 'can fly'),
    (0x0010, 'flying'),
    (0x0020, 'tumbled'),
    (0x0040, 'locked'),
    (0x0080, 'crashed'),
]
PM_STATES = {0: '', 1: 'charging', 2: 'charged', 3: 'LOW POWER', 4: 'SHUTTING DOWN'}


def supervisor_state(info, pm=None):
    """(short state, flag names, severity) from a supervisor_info bitfield.

    The short state is what a drone tile shows, so it answers the question you
    actually have walking up to the rig: can this thing fly, and if not, why?
    Ordered worst-first -- a locked drone that is also tumbled reads
    "E-STOPPED + flipped", because the e-stop is what you must clear first.
    """
    if info is None:
        return '', [], OK
    flags = [name for bit, name in SUPERVISOR_BITS if info & bit]
    unknown = info & ~sum(bit for bit, _ in SUPERVISOR_BITS)
    if unknown:
        flags.append(f'unknown bits 0x{unknown:04x}')
    locked, crashed = info & 0x0040, info & 0x0080
    tumbled, flying, armed = info & 0x0020, info & 0x0010, info & 0x0002
    words, sev = [], OK
    if locked:
        words.append('E-STOPPED')
        sev = FAIL
    if crashed:
        words.append('CRASHED')
        sev = FAIL
    if tumbled:
        words.append('flipped' if words else 'FLIPPED')
        sev = FAIL
    if not words:
        if flying:
            words.append('FLYING')
        elif armed:
            words.append('armed')
        elif info & 0x0001:
            words.append('ready to arm')
        else:
            # not locked, not tumbled, simply not armable yet: a low battery, a
            # missing position estimate, or still settling after boot
            words.append('cannot arm yet')
            sev = WARN
    pm_word = PM_STATES.get(pm, '')
    if pm_word:
        words.append(pm_word)
        if pm in (3, 4):
            sev = FAIL if pm == 4 else max(sev, WARN, key=lambda s: RANK[s])
    return ' + '.join(words), flags, sev


def _node(id, label, col, row, group, why):
    return {'id': id, 'label': label, 'col': col, 'row': row, 'group': group,
            'why': why, 'status': UNKNOWN, 'summary': '', 'detail': '', 'fix': '',
            'commands': [], 'findings': [], 'metrics': {}}


class Health:
    """Builds the graph. One instance lives for the life of the console."""

    def __init__(self, procs=None):
        self.procs = procs
        self.last = None
        self.last_run = 0.0
        self.deep_radio = {}          # address -> result of the last scan

    # -------------------------------------------------------------- helpers
    @staticmethod
    def _have_ros2():
        return shutil.which('ros2') is not None

    def _ros(self, argv, timeout=6.0):
        env = dict(os.environ)
        env['PYTHONUNBUFFERED'] = '1'
        env.setdefault('RCUTILS_COLORIZED_OUTPUT', '0')
        return run_capture(argv, timeout=timeout, cwd=REPO, env=env)

    @staticmethod
    def _record(node, res):
        node['commands'].append({'cmd': res['cmd'], 'out': res['out'].strip()[-4000:],
                                 'rc': res['rc'], 'timeout': res['timeout']})
        return res

    # ---------------------------------------------------------------- probes
    def _check_env(self, nodes):
        n = nodes['env.ros']
        distro = os.environ.get('ROS_DISTRO', '')
        if not self._have_ros2():
            n.update(status=FAIL, summary='no `ros2` on PATH',
                     detail='This console was started without a ROS environment, so it '
                            'cannot run or see anything.',
                     fix='Stop it and start it with console/run.sh, which sources '
                         '/opt/ros/$ROS_DISTRO/setup.bash and this workspace\'s '
                         'install/setup.bash for you.')
        elif not distro:
            n.update(status=WARN, summary='ros2 found but ROS_DISTRO is empty',
                     detail='Some scripts key off $ROS_DISTRO and will misbehave.',
                     fix='source /opt/ros/<distro>/setup.bash')
        else:
            domain = os.environ.get('ROS_DOMAIN_ID', '0 (default)')
            n.update(status=OK, summary=f'{distro}, ROS_DOMAIN_ID={domain}',
                     detail='A node is only visible to shells with the SAME '
                            'ROS_DOMAIN_ID. If you see no topics anywhere, compare this '
                            'with the shell that started the stack.')

        n = nodes['env.overlay']
        install = os.path.join(REPO, 'install')
        setup = os.path.join(install, 'setup.bash')
        prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(':')
        sourced = any(p.startswith(install) for p in prefixes if p)
        if not os.path.exists(setup):
            n.update(status=FAIL, summary='the workspace is not built',
                     detail=f'{setup} does not exist, so nothing in src/ is runnable.',
                     fix='./scripts/build.sh')
        elif not sourced:
            n.update(status=FAIL, summary='built, but this overlay is not sourced',
                     detail='Commands would resolve to the apt packages in /opt/ros '
                            'instead of the vendored, patched ones in this repo.',
                     fix='source install/setup.bash (console/run.sh does it).')
        else:
            built = sorted(os.listdir(install)) if os.path.isdir(install) else []
            pkgs = [b for b in built if not b.startswith(('local_setup', 'setup', '_', 'COLCON'))]
            n.update(status=OK, summary=f'{len(pkgs)} packages sourced from install/',
                     detail='Built packages: ' + ', '.join(pkgs))

        n = nodes['env.python']
        py = shutil.which('python3') or ''
        conda = os.environ.get('CONDA_PREFIX') or os.environ.get('VIRTUAL_ENV')
        if conda or (py and py != '/usr/bin/python3'):
            n.update(status=FAIL, summary=f'python3 is {py or "?"}',
                     detail='ROS 2 runs its nodes with /usr/bin/python3. A conda or venv '
                            'interpreter here causes rclpy import errors and message '
                            'types that load as invalid.'
                            + (f' CONDA_PREFIX/VIRTUAL_ENV = {conda}' if conda else ''),
                     fix='conda deactivate (repeat until gone), then restart this console.')
        else:
            n.update(status=OK, summary='/usr/bin/python3, no conda or venv active')

    def _check_config(self, nodes, cf_doc, cf_err, mc_doc, mc_err):
        n = nodes['cfg.parse']
        if cf_err or mc_err:
            n.update(status=FAIL, summary='a config file does not parse',
                     detail=(cf_err or '') + ('\n' + mc_err if mc_err else ''),
                     fix='Fix the YAML in the Config tab -- the launch cannot start '
                         'until it parses.')
            return
        fleet = configio.fleet_summary(cf_doc)
        enabled = [d for d in fleet if d['enabled']]
        n.update(status=OK,
                 summary=f'{len(enabled)} of {len(fleet)} drones enabled: '
                         + (', '.join(d['name'] for d in enabled) or 'none'),
                 detail='crazyflies.yaml is the ground truth for the fleet, and it is '
                        'read ONLY at server launch -- edits need a restart.')

        n = nodes['cfg.fleet']
        findings = configio.validate_crazyflies(cf_doc) + configio.validate_motion_capture(mc_doc)
        n['findings'] = findings
        fails = [f for f in findings if f['level'] == 'fail']
        warns = [f for f in findings if f['level'] == 'warn']
        if fails:
            n.update(status=FAIL, summary=f'{len(fails)} problem(s) in the config',
                     detail='\n'.join(f'- {f["title"]}: {f["detail"]}' for f in fails),
                     fix='\n'.join(f['fix'] for f in fails if f['fix']))
        elif warns:
            n.update(status=WARN, summary=f'{len(warns)} warning(s)',
                     detail='\n'.join(f'- {f["title"]}: {f["detail"]}' for f in warns))
        else:
            n.update(status=OK, summary='fleet geometry, URIs and log blocks all check out',
                     detail='Checked: unique addresses, >= 1 m between enabled drones, '
                            'known robot types, dongle channel spacing, and the 26-byte '
                            'firmware log-block budget.')

    def _check_radio_usb(self, nodes):
        n = nodes['radio.usb']
        res = self._record(n, run_capture(['bash', '-c', 'lsusb'], timeout=5))
        hits = [l for l in res['out'].splitlines()
                if re.search(r'crazyradio|bitcraze|1915:|35d2:', l, re.I)]
        if res['rc'] != 0:
            n.update(status=UNKNOWN, summary='could not run lsusb',
                     detail=res['out'].strip())
        elif hits:
            n.update(status=OK, summary=f'{len(hits)} Crazyradio dongle(s) on USB',
                     detail='\n'.join(hits))
        else:
            n.update(status=FAIL, summary='no Crazyradio found on USB',
                     detail='lsusb shows no Bitcraze device. Without the dongle the '
                            'server has no path to any drone.',
                     fix='Plug the dongle in. If it is plugged in, check the udev rule '
                         'from README Setup Step 3 and that you are in the plugdev group.')

    def _check_motive(self, nodes):
        n = nodes['mocap.host']
        mc = configio.load('motion_capture')
        params = ((mc['doc'] or {}).get('/motion_capture_tracking') or {}).get('ros__parameters') or {}
        configured = str(params.get('hostname', 'auto'))
        override = os.environ.get('CRAZYSWARM_MOCAP_HOST', '').strip()
        responders, err = _natnet_discover()
        n['commands'].append({'cmd': f'(NatNet discovery: UDP {NATNET_CMD_PORT} broadcast ping)',
                              'out': '\n'.join(responders) or (err or 'no answer'), 'rc': 0,
                              'timeout': False})
        target = override or configured
        if responders:
            extra = f' (config says {target})' if target.lower() != 'auto' else ''
            status = OK
            detail = ('Motive answered the same discovery ping launch.py uses, so the '
                      'network path is good in both directions.')
            if target.lower() != 'auto' and target not in responders:
                status = WARN
                detail += (f'\nBut the configured address {target} is NOT among the '
                           'responders -- the Motive PC has moved (its DHCP lease has '
                           'drifted .100 -> .124 -> .152 before, silently killing /poses).')
            n.update(status=status, summary='Motive at ' + ', '.join(responders) + extra,
                     detail=detail,
                     fix='' if status == OK else 'Set hostname: "auto" in motion_capture.yaml, '
                         'or pass mocap_hostname:=' + responders[0])
        else:
            n.update(status=FAIL, summary='no Motive answered the discovery ping',
                     detail=(err or 'Nothing replied on UDP 1510 within the timeout.') +
                            '\nPing proves nothing here: unicast can work while multicast '
                            'is blocked.',
                     fix='Check you are on the Motive LAN segment (wired 192.168.9.x; '
                         '`ip route get <motive-ip>` must not say "via"), that Motive is '
                         'running with streaming enabled (Multicast, 50 Hz), and that the '
                         'Windows firewall allows NatNet.')

        n = nodes['mocap.port']
        res = self._record(n, run_capture(['bash', '-c', 'ss -uanp 2>/dev/null | grep ":1511" || true'],
                                          timeout=5))
        lines = [l for l in res['out'].splitlines() if l.strip()]
        if len(lines) > 1:
            n.update(status=FAIL, summary=f'{len(lines)} sockets bound to UDP 1511',
                     detail='This is the confirmed cause of "mocap suddenly died" on this '
                            'rig. With two SO_REUSEPORT sockets on 1511 the kernel hands '
                            'each NatNet datagram to only ONE of them, so the mocap node '
                            'starves and hangs silently inside connect().\n' + res['out'].strip(),
                     fix='Kill the leftover (Recovery -> Kill leftover mocap nodes), '
                         'confirm nothing holds 1511, then relaunch.')
        elif lines:
            n.update(status=OK, summary='one socket on UDP 1511 (the mocap node)',
                     detail=res['out'].strip())
        else:
            n.update(status=OK, summary='UDP 1511 is free',
                     detail='Expected when the mocap node is not running. If it IS '
                            'running and nothing holds 1511, it is blocked in connect().')

    def _check_graph(self, nodes, fleet):
        """One `ros2 node list` + one `ros2 service list` feed several nodes."""
        if not self._have_ros2():
            for nid in ('mocap.pkg', 'mocap.node', 'mocap.poses', 'server.node', 'server.services'):
                nodes[nid].update(status=BLOCKED, summary='no ROS environment')
            return {}

        n = nodes['mocap.pkg']
        res = self._record(n, self._ros(['ros2', 'pkg', 'prefix', 'motion_capture_tracking'], 10))
        prefix = res['out'].strip().splitlines()[-1] if res['out'].strip() else ''
        if res['rc'] != 0 or not prefix or 'not found' in prefix.lower():
            n.update(status=FAIL, summary='motion_capture_tracking not found',
                     detail='The vendored driver is not built or not sourced.',
                     fix='./scripts/build.sh, then source install/setup.bash.')
        elif prefix.startswith('/opt/ros'):
            n.update(status=FAIL, summary='resolves to the apt package',
                     detail=f'{prefix}\napt 1.0.9 hard-codes interface IP 141.23.110.162 '
                            'into the NatNet socket: the node either aborts with exit -6 '
                            'or publishes an empty /poses forever.',
                     fix='sudo apt remove ros-$ROS_DISTRO-motion-capture-tracking '
                         'ros-$ROS_DISTRO-motion-capture-tracking-interfaces, then '
                         './scripts/build.sh and re-source.')
        else:
            n.update(status=OK, summary='vendored driver', detail=prefix)

        graph = {}
        res = self._record(nodes['server.node'], self._ros(['ros2', 'node', 'list'], 12))
        node_list = [l.strip() for l in res['out'].splitlines() if l.strip().startswith('/')]
        graph['nodes'] = node_list
        graph['node_list_failed'] = res['timeout'] or res['rc'] not in (0, None)

        server_up = any(l.endswith('/crazyflie_server') or l == '/crazyflie_server'
                        for l in node_list)
        mocap_up = any('motion_capture_tracking' in l for l in node_list)
        graph['server_running'] = server_up
        graph['mocap_running'] = mocap_up

        # One topic list serves every drone check below -- and /clock tells us the
        # simulator is running, which changes what "missing" means further down.
        topics = []
        if server_up:
            tres = self._ros(['ros2', 'topic', 'list'], 12)
            topics = [l.strip() for l in tres['out'].splitlines() if l.strip().startswith('/')]
        graph['topics'] = topics
        graph['sim'] = '/clock' in topics

        n = nodes['mocap.node']
        n['commands'] = list(nodes['server.node']['commands'])
        if mocap_up:
            n.update(status=OK, summary='/motion_capture_tracking is alive',
                     detail='Alive is not the same as streaming -- check /poses below. '
                            'A node that is up but publishing nothing is blocked in '
                            'connect() against a stale Motive address.')
        else:
            # Deliberately not keyed off the Motive box: that probe runs on another
            # thread and may not have finished. "Not running" is the normal state
            # before launch anyway -- the upstream boxes carry the real failure.
            n.update(status=WARN,
                     summary='mocap node not running',
                     detail='Not in `ros2 node list`. Either the stack is not launched, '
                            'it was launched with mocap:=False or backend:=sim, or the '
                            'node died / hung before it finished initialising.',
                     fix='Launch the stack, or check the Processes tab for its output.')

        n = nodes['server.node']
        if server_up:
            n.update(status=OK, summary='/crazyflie_server is alive',
                     detail='\n'.join(node_list))
        elif graph['node_list_failed']:
            n.update(status=UNKNOWN, summary='`ros2 node list` did not answer',
                     detail='The CLI daemon may be wedged.',
                     fix='Recovery -> Restart the ros2 daemon.')
        else:
            n.update(status=WARN, summary='crazyflie_server not running',
                     detail='Nothing is talking to the drones. This is the normal state '
                            'before launch.\n' + ('\n'.join(node_list) or '(no nodes at all)'),
                     fix='Launch -> Start the stack (scan the fleet first).')

        n = nodes['server.services']
        if not server_up:
            n.update(status=BLOCKED, summary='no server')
        else:
            res = self._record(n, self._ros(['ros2', 'service', 'list'], 12))
            svcs = [l.strip() for l in res['out'].splitlines() if l.strip().startswith('/')]
            graph['services'] = svcs
            wanted = ['/all/takeoff', '/all/land', '/all/arm', '/all/emergency']
            missing = [w for w in wanted if w not in svcs]
            if not missing:
                n.update(status=OK, summary='/all/* services are up',
                         detail='takeoff, land, arm and emergency are all advertised, so '
                                'the server finished connecting every enabled drone.')
            elif len(missing) == len(wanted):
                n.update(status=FAIL, summary='the /all/* services never appeared',
                         detail='The server process is alive but has NOT finished '
                                'connecting. It connects drones in lexicographic order and '
                                'blocks FOREVER, silently, on the first enabled drone that '
                                'does not answer radio -- one unreachable drone kills the '
                                'whole launch.',
                         fix='Kill the stack (Recovery), scan every enabled address, then '
                             'fix the URI -- channel AND datarate must match -- or set the '
                             'dead drone enabled: false. cf6 died exactly this way.')
            else:
                # Some are there: the server DID get through its connect loop, so this is
                # a backend difference, not the hang. crazyflie_sim has no arming.
                n.update(status=OK if (graph.get('sim') and missing == ['/all/arm']) else WARN,
                         summary='up, without ' + ', '.join(missing),
                         detail='The server finished connecting (some /all/* services '
                                'exist), so this is not the silent-hang failure. '
                                + ('The simulator backend does not implement arming, so a '
                                   'missing /all/arm is expected here.'
                                   if graph.get('sim') else
                                   'Missing: ' + ', '.join(missing) + '. Check which '
                                   'backend you launched -- cflib and sim advertise fewer '
                                   'services than the cpp server.'))
        return graph

    def _check_poses(self, nodes, graph):
        n = nodes['mocap.poses']
        if not self._have_ros2():
            n.update(status=BLOCKED, summary='no ROS environment')
            return None
        res = self._record(n, self._ros(['ros2', 'topic', 'info', '/poses'], 10))
        pubs = 0
        m = re.search(r'Publisher count:\s*(\d+)', res['out'])
        if m:
            pubs = int(m.group(1))
        if pubs == 0:
            n.update(status=FAIL if graph.get('mocap_running') else WARN,
                     summary='/poses has no publisher',
                     detail='Nothing is producing mocap poses, so every drone will fall '
                            'back on its own estimator and drift.'
                            + (' The mocap node IS running, so it is starved or blocked '
                               'in connect() -- check UDP 1511 and the Motive address.'
                               if graph.get('mocap_running') else
                               ' The mocap node is not running.'),
                     fix='See the Motive and UDP 1511 nodes to the left.')
            return None
        rate = None
        res = self._record(n, self._ros(['ros2', 'topic', 'hz', '/poses', '--window', '20'], 5))
        rates = [float(x) for x in re.findall(r'average rate:\s*([\d.]+)', res['out'])]
        if rates:
            rate = rates[-1]
        if rate is None:
            n.update(status=FAIL, summary=f'{pubs} publisher(s) but no messages',
                     detail='The topic exists and nothing arrives within 5 s. On this rig '
                            'that means the driver is connected but starved: a leftover on '
                            'UDP 1511, or Motive streaming to a different interface.',
                     fix='Check UDP 1511, then Motive -> Settings -> Streaming '
                         '(Multicast, 50 Hz, matching motion_capture.yaml).')
        elif rate < 30:
            n['metrics'] = {'hz': rate}
            n.update(status=WARN, summary=f'/poses at {rate:.1f} Hz (expected ~50)',
                     detail='Below the streaming rate this rig is configured for. '
                            'A sagging rate means network loss or an overloaded Motive PC.',
                     fix='Check the Motive streaming rate and the network path.')
        else:
            n['metrics'] = {'hz': rate}
            n.update(status=OK, summary=f'/poses at {rate:.1f} Hz',
                     detail=f'{pubs} publisher(s). This is the mocap heartbeat every '
                            'drone\'s position estimate depends on.')
        return rate

    def _check_drones(self, nodes, fleet, graph):
        enabled = [d for d in fleet if d['enabled']]
        if not graph.get('server_running'):
            for d in enabled:
                nodes[f'drone.{d["name"]}'].update(
                    status=BLOCKED, summary='server not running',
                    detail='A drone only appears on the ROS graph once the server has '
                           'connected to it.')
            return
        with ThreadPoolExecutor(max_workers=min(8, max(1, len(enabled)))) as pool:
            list(pool.map(lambda d: self._check_one_drone(nodes[f'drone.{d["name"]}'], d, graph),
                          enabled))

    def _check_one_drone(self, n, drone, graph):
        name = drone['name']
        topics = graph.get('topics') or []
        svcs = graph.get('services') or []
        has_status, has_pose = f'/{name}/status' in topics, f'/{name}/pose' in topics
        sim = graph.get('sim')
        # Its own services are the honest "did the server connect this drone?" signal:
        # every backend advertises them, while the telemetry topics differ by backend.
        if svcs and f'/{name}/takeoff' not in svcs:
            n.update(status=FAIL, summary='never connected',
                     detail=f'The server advertises no /{name}/takeoff, so it never '
                            'finished connecting this drone.',
                     fix='Scan its address; fix the URI (channel AND datarate) or set it '
                         'enabled: false.')
            return
        if not has_status and not has_pose:
            n.update(status=OK if sim else WARN,
                     summary='connected, no telemetry topics',
                     detail=f'/{name}/takeoff exists, so the drone is connected, but it '
                            f'publishes neither /{name}/status nor /{name}/pose.' +
                            ('\nNormal on the simulator backend: crazyflie_sim reports '
                             'state through /tf and RViz, not through firmware logging.'
                             if sim else
                             '\nOn hardware that means firmware logging is off, so battery '
                             'voltage, link health and the onboard estimate are all '
                             'invisible -- and the preflight GUI will stay empty.'),
                     fix='' if sim else 'Set all.firmware_logging.enabled: true and keep '
                         'the pose/status default_topics in crazyflies.yaml.')
            return
        if not has_status and has_pose:
            res = self._record(n, self._ros(
                ['ros2', 'topic', 'echo', f'/{name}/pose', '--once'], 6))
            alive = not res['timeout'] and 'position' in res['out']
            n.update(status=OK if (alive and sim) else WARN,
                     summary='pose only, no /status' + ('' if alive else ' (and no pose yet)'),
                     detail=(f'/{name}/pose is publishing but /{name}/status is not.'
                             if alive else f'/{name}/pose produced nothing within 6 s.') +
                            ('\nNormal on the simulator backend: there is no radio link and '
                             'no battery to report.' if sim else
                             '\nOn hardware this means battery voltage, supervisor bits and '
                             'link health are invisible -- enable the status topic under '
                             'all.firmware_logging.default_topics in crazyflies.yaml.'),
                     fix='' if sim else 'Enable the status default_topic in crazyflies.yaml.')
            return
        res = self._record(n, self._ros(
            ['ros2', 'topic', 'echo', f'/{name}/status', '--once'], 6))
        out = res['out']
        if res['timeout'] or 'battery_voltage' not in out:
            n.update(status=FAIL, summary='no status message',
                     detail=f'/{name}/status produced nothing within 6 s. The server '
                            'either never connected this drone, or firmware logging is '
                            'off for it.',
                     fix='Scan its address, and check that status is enabled under '
                         'all.firmware_logging.default_topics in crazyflies.yaml.')
            return
        def grab(field, cast=float):
            m = re.search(rf'^{field}:\s*(\S+)', out, re.M)
            if not m:
                return None
            try:
                return cast(m.group(1))
            except ValueError:
                return m.group(1)
        volts = grab('battery_voltage')
        rssi = grab('rssi')
        latency = grab('latency_unicast')
        info = grab('supervisor_info', int)
        pm = grab('pm_state', int)
        state, flags, sev = supervisor_state(info, pm)
        bits = []
        if state:
            bits.append(state)
        if volts is not None:
            bits.append(f'{volts:.2f} V')
        if rssi is not None:
            bits.append(f'rssi {rssi:.0f}')
        if latency is not None:
            bits.append(f'latency {latency:.1f} ms')
        # Three independent things can be wrong at once -- the supervisor, the
        # battery, the radio -- so take the WORST status and collect every fix,
        # rather than letting whichever test runs last overwrite the others. (A
        # low battery must not downgrade an E-STOPPED drone to a warning.)
        detail = out.strip()[:1200]
        if flags:
            detail = 'supervisor: ' + ', '.join(flags) + f'  (0x{info:04x})\n\n' + detail
        status, fixes = sev, []
        worst = lambda a, b: a if RANK[a] >= RANK[b] else b      # noqa: E731
        if info is not None and info & 0x0040:
            fixes.append('The supervisor is LOCKED -- this is where an emergency stop '
                         'leaves a drone. Power-cycle it (battery out and in); nothing '
                         'else clears it.')
        if info is not None and info & 0x0080:
            fixes.append('The firmware flagged a crash. Check the airframe, then '
                         'power-cycle it.')
        if info is not None and info & 0x0020:
            fixes.append('The drone is not level (tumbled). Stand it back on its feet.')
        if volts is not None and volts < 3.7:
            status = worst(status, FAIL)
            fixes.append('Swap the battery: below voltage_critical (3.7 V).')
        elif volts is not None and volts < 3.8:
            status = worst(status, WARN)
            fixes.append('Below voltage_warning (3.8 V) -- charge before a long flight.')
        if latency is not None and latency > 10:
            status = worst(status, WARN)
            fixes.append('Unicast latency is above the 10 ms threshold: the radio is '
                         'saturated -- lower the firmware logging rates.')
        fix = ' '.join(fixes)
        n.update(status=status, summary=', '.join(bits) or 'connected', detail=detail, fix=fix,
                 metrics={'volts': volts, 'rssi': rssi, 'latency': latency,
                          'supervisor': info, 'state': state, 'flags': flags,
                          'pm_state': pm})

    def _check_signatures(self, nodes):
        """Read the output of everything we have run and translate known errors."""
        if self.procs is None:
            return
        blob_by_proc = []
        for proc in list(self.procs.procs.values()):
            text = '\n'.join(t for _, _, t in proc.tail())
            if text:
                blob_by_proc.append((proc, text))
        for pattern, node_id, level, title, detail, fix in SIGNATURES:
            rx = re.compile(pattern, re.I)
            # newest process first: if a signature matched twice, the evidence
            # worth showing is from the most recent run, not the first one.
            for proc, text in reversed(blob_by_proc):
                hits = [l for l in text.splitlines() if rx.search(l)]
                if not hits:
                    continue
                n = nodes.get(node_id)
                if n is None:
                    continue
                # A line in a process that has already finished normally is
                # HISTORY, not a live fault. Every `Stop the stack` leaves
                # "process has died ... motion_capture_tracking" in the launch's
                # output (that node aborts on shutdown, exit -6), so matching it
                # unconditionally pinned this box to "The mocap node died" for the
                # rest of the session -- over the top of a live `ros2 node list`
                # that had just seen the node alive and /poses at 50 Hz.
                # Only a process that is still running, or one that actually
                # failed, may override a live probe.
                authoritative = proc.state() in ('running', 'stopping', 'failed')
                n['findings'].append({
                    'level': level if authoritative else 'info',
                    'title': title if authoritative else f'{title} (earlier run)',
                    'detail': detail + '\n\nSeen in: ' + proc.label +
                              ('' if authoritative else
                               ' -- which has since finished, so this is history, '
                               'not the current state') + '\n  ' +
                              '\n  '.join(hits[-3:]),
                    'fix': fix, 'source': proc.id})
                if authoritative and RANK[level] > RANK[n['status']]:
                    n.update(status=level, summary=title, detail=n['detail'] or detail,
                             fix=n['fix'] or fix)
                break

    # ------------------------------------------------------------------ run
    def run(self):
        started = time.time()
        cf = configio.load('crazyflies')
        mc = configio.load('motion_capture')
        fleet = configio.fleet_summary(cf['doc']) if cf['doc'] else []
        enabled = [d for d in fleet if d['enabled']]

        nodes = {}
        for nid, label, col, row, group, why in [
            ('env.ros', 'ROS 2 environment', 0, 0, 'Workspace',
             'Is a ROS distro sourced, and on which domain?'),
            ('env.overlay', 'Workspace overlay', 0, 1, 'Workspace',
             'Is this repo built and sourced ahead of /opt/ros?'),
            ('env.python', 'Python interpreter', 0, 2, 'Workspace',
             'ROS runs nodes with /usr/bin/python3; conda breaks that.'),
            ('cfg.parse', 'Config files', 1, 0, 'Configuration',
             'Do crazyflies.yaml and motion_capture.yaml parse?'),
            ('cfg.fleet', 'Fleet sanity', 1, 1, 'Configuration',
             'Addresses, separation, types, log-block budget.'),
            ('radio.usb', 'Crazyradio dongle', 2, 0, 'Radio',
             'Is the USB radio present?'),
            ('radio.drones', 'Drones answer radio', 2, 1, 'Radio',
             'Does every enabled address reply to a scan? The go/no-go check.'),
            ('mocap.host', 'Motive / NatNet', 1, 3, 'Mocap',
             'Does the Motive PC answer a discovery ping?'),
            ('mocap.port', 'UDP 1511', 1, 4, 'Mocap',
             'Is anything else holding the NatNet data port?'),
            ('mocap.pkg', 'Mocap driver build', 2, 3, 'Mocap',
             'Vendored driver, or the broken apt one?'),
            ('mocap.node', 'motion_capture_tracking', 3, 3, 'Mocap',
             'Is the driver node alive?'),
            ('mocap.poses', '/poses stream', 4, 3, 'Mocap',
             'Are mocap poses actually flowing, and how fast?'),
            ('server.node', 'crazyflie_server', 3, 0, 'Server',
             'Is the server process alive?'),
            ('server.services', '/all/* services', 4, 0, 'Server',
             'Did it finish connecting every drone? Silence here is the classic hang.'),
        ]:
            nodes[nid] = _node(nid, label, col, row, group, why)
        for i, d in enumerate(enabled):
            nodes[f'drone.{d["name"]}'] = _node(
                f'drone.{d["name"]}', d['name'], 5, i, 'Drones',
                f'{d["name"]}: link health, battery and telemetry ({d["address"] or "?"}).')

        edges = [
            ('env.ros', 'env.overlay'), ('env.python', 'env.overlay'),
            ('env.overlay', 'cfg.parse'), ('cfg.parse', 'cfg.fleet'),
            ('cfg.fleet', 'radio.usb'), ('radio.usb', 'radio.drones'),
            ('cfg.fleet', 'mocap.host'), ('mocap.host', 'mocap.pkg'),
            ('mocap.port', 'mocap.pkg'), ('env.overlay', 'mocap.pkg'),
            ('mocap.pkg', 'mocap.node'), ('mocap.node', 'mocap.poses'),
            ('radio.drones', 'server.node'), ('server.node', 'server.services'),
            ('mocap.poses', 'server.services'),
        ]
        for d in enabled:
            edges.append(('server.services', f'drone.{d["name"]}'))

        # --- run the probes ---
        self._check_env(nodes)
        self._check_config(nodes, cf['doc'], cf['parse_error'], mc['doc'], mc['parse_error'])
        with ThreadPoolExecutor(max_workers=4) as pool:
            futures = [pool.submit(self._check_radio_usb, nodes),
                       pool.submit(self._check_motive, nodes)]
            graph = self._check_graph(nodes, fleet)
            for f in futures:
                f.result()
        rate = self._check_poses(nodes, graph) if graph else None
        if graph.get('sim'):
            for nid, msg in (('mocap.node', 'not used on the sim backend'),
                             ('mocap.poses', 'not used on the sim backend'),
                             ('mocap.host', 'not used on the sim backend'),
                             ('radio.usb', 'not used on the sim backend')):
                if nodes[nid]['status'] in (WARN, FAIL):
                    nodes[nid].update(
                        status=SKIP, summary=msg,
                        detail='The simulator provides its own state and needs no radio '
                               'and no mocap, so this part of the chain is out of the '
                               'picture. It matters again the moment you launch with '
                               'backend:=cpp.')
        self._check_drones(nodes, fleet, graph)

        # radio scan is slow and needs the radio free, so it is opt-in
        n = nodes['radio.drones']
        if graph.get('sim'):
            n.update(status=SKIP, summary='not used on the sim backend',
                     detail='No radio is involved in a simulator run. Scan before every '
                            'hardware launch instead.')
        elif graph.get('server_running'):
            n.update(status=SKIP, summary='server owns the radio',
                     detail='A scan cannot run while the server holds the dongle -- only '
                            'one process may own a Crazyradio. The fact that the /all/* '
                            'services came up already proves every enabled drone answered.')
        elif self.deep_radio:
            answered = [a for a, v in self.deep_radio.items() if v['ok']]
            silent = [a for a, v in self.deep_radio.items() if not v['ok']]
            n['commands'] = [{'cmd': v['cmd'], 'out': v['out'], 'rc': 0, 'timeout': False}
                             for v in self.deep_radio.values()]
            if silent:
                n.update(status=FAIL, summary=f'{len(silent)} address(es) did not answer',
                         detail='Silent: ' + ', '.join(silent) + '\nAnswered: ' +
                                (', '.join(answered) or 'none') +
                                '\nLaunching now would hang the server forever, silently, '
                                'on the first silent drone.',
                         fix='Fix the URI (channel AND datarate must match the scan reply) '
                             'or set that drone enabled: false before launching.')
            else:
                n.update(status=OK, summary=f'all {len(answered)} enabled drones answered',
                         detail='\n'.join(f'{a}: {self.deep_radio[a]["out"]}' for a in answered))
        else:
            n.update(status=UNKNOWN, summary='not scanned yet',
                     detail='This is the go/no-go check before every launch: the server '
                            'blocks forever, silently, on the first enabled drone that '
                            'does not answer. Nothing else catches a dead drone or a '
                            'datarate mismatch.',
                     fix='Press "Scan the fleet" (needs the radio free, i.e. no server).')

        self._check_signatures(nodes)

        # --- mark everything downstream of a hard failure as blocked ---
        incoming = {}
        for a, b in edges:
            incoming.setdefault(b, []).append(a)
        order = ['env.ros', 'env.python', 'env.overlay', 'cfg.parse', 'cfg.fleet',
                 'radio.usb', 'radio.drones', 'mocap.host', 'mocap.port', 'mocap.pkg',
                 'mocap.node', 'mocap.poses', 'server.node', 'server.services']
        order += [f'drone.{d["name"]}' for d in enabled]
        for nid in order:
            n = nodes.get(nid)
            if n is None or n['status'] in (FAIL, SKIP):
                continue
            broken = [p for p in incoming.get(nid, [])
                      if nodes.get(p, {}).get('status') in (FAIL, BLOCKED)]
            if broken and n['status'] != OK:
                n.update(status=BLOCKED,
                         summary=n['summary'] or 'blocked upstream',
                         detail=(n['detail'] + '\n\n' if n['detail'] else '') +
                                'Blocked by: ' + ', '.join(nodes[b]['label'] for b in broken))

        worst = max((n['status'] for n in nodes.values()), key=lambda s: RANK[s], default=OK)
        summary = _headline(nodes, worst)
        self.last = {
            'nodes': list(nodes.values()),
            'edges': [{'from': a, 'to': b} for a, b in edges],
            'facts': {
                'server_running': bool(graph.get('server_running')),
                'mocap_running': bool(graph.get('mocap_running')),
                'poses_hz': rate,
                'enabled_drones': [d['name'] for d in enabled],
                'fleet': fleet,
            },
            'worst': worst,
            'headline': summary,
            'elapsed': time.time() - started,
            'ts': time.time(),
        }
        self.last_run = time.time()
        return self.last

    # -------------------------------------------------------- the deep scan
    def scan_fleet(self, fleet):
        """Run `scan` against every enabled address. Needs the radio free."""
        out = {}
        for d in [x for x in fleet if x['enabled'] and x['address']]:
            res = self._ros(['ros2', 'run', 'crazyflie', 'scan', '--address', d['address']], 25)
            text = res['out'].strip()
            out[d['address']] = {
                'ok': 'radio://' in text, 'out': text or '(no reply)', 'cmd': res['cmd'],
                'drone': d['name'], 'uri': d['uri'],
            }
            if 'radio://' in text and d['datarate']:
                replies = re.findall(r'radio://[^\s]+', text)
                if not any(f'/{d["datarate"]}/' in r for r in replies):
                    out[d['address']]['ok'] = False
                    out[d['address']]['out'] = (
                        text + f'\n!! the drone answered, but not at {d["datarate"]} -- '
                        'a datarate mismatch in the URI hangs the server forever.')
        self.deep_radio = out
        return out


def _headline(nodes, worst):
    if worst in (OK, SKIP):
        skipped = sum(1 for n in nodes.values() if n['status'] == SKIP)
        return 'All checks pass.' + (f' ({skipped} not applicable right now.)' if skipped else '')
    order = ['env.ros', 'env.python', 'env.overlay', 'cfg.parse', 'cfg.fleet', 'radio.usb',
             'radio.drones', 'mocap.host', 'mocap.port', 'mocap.pkg', 'mocap.node',
             'mocap.poses', 'server.node', 'server.services']
    order += sorted(k for k in nodes if k.startswith('drone.'))
    for nid in order:
        n = nodes.get(nid)
        if n and n['status'] == FAIL:
            return f'{n["label"]}: {n["summary"]}'
    for nid in order:
        n = nodes.get(nid)
        if n and n['status'] == WARN:
            return f'{n["label"]}: {n["summary"]}'
    return 'Some checks have not run yet.'


def _natnet_discover(timeout_s=1.5):
    """The same discovery ping launch.py uses: NAT_CONNECT broadcast on UDP 1510."""
    targets = {'255.255.255.255'}
    try:
        for iface in json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr'], timeout=2)):
            for a in iface.get('addr_info', []):
                if a.get('broadcast'):
                    targets.add(a['broadcast'])
    except Exception:                                   # noqa: BLE001
        pass
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.2)
        for msg_id in (NATNET_CONNECT, NATNET_DISCOVERY):
            probe = struct.pack('<HH', msg_id, 0)
            for t in sorted(targets):
                try:
                    sock.sendto(probe, (t, NATNET_CMD_PORT))
                except OSError:
                    pass
        responders, deadline = [], time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                data, (ip, _) = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                return [], str(exc)
            if len(data) >= 4 and struct.unpack_from('<H', data, 0)[0] == NATNET_SERVERINFO \
                    and ip not in responders:
                responders.append(ip)
        return responders, None
    finally:
        sock.close()
