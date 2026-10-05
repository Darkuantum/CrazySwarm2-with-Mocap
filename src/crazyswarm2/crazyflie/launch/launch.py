import os
import ipaddress
import json
import socket
import struct
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable

def _mocap_enabled(context):
    backend = LaunchConfiguration('backend').perform(context)
    mocap = LaunchConfiguration('mocap').perform(context)
    return backend != 'sim' and mocap.lower() in ('true', '1')


def check_vendored_mocap_driver(context):
    """Abort the launch if motion_capture_tracking would come from apt.

    The apt release 1.0.9 (Humble; queued for Jazzy) hard-codes a foreign
    interface IP (141.23.110.162) into the NatNet socket code: the node either
    aborts with exit -6 or publishes an empty /poses forever. This repo vendors
    a fixed copy in src/motion_capture_tracking (see its VENDORED.md); the
    workspace build must be the one that resolves in this shell.
    """
    if not _mocap_enabled(context):
        return
    try:
        prefix = get_package_prefix('motion_capture_tracking')
    except PackageNotFoundError:
        raise RuntimeError(
            "motion_capture_tracking not found. It is vendored in "
            "src/motion_capture_tracking: run ./scripts/build.sh and then "
            "`source install/setup.bash` in this shell (or launch with mocap:=False).")
    if prefix.startswith('/opt/ros'):
        raise RuntimeError(
            f"motion_capture_tracking resolves to the apt package at {prefix}. "
            "apt 1.0.9 hard-codes interface IP 141.23.110.162 and never receives "
            "NatNet frames, so /poses would stay silent. This repo vendors a fixed "
            "copy in src/motion_capture_tracking: run ./scripts/build.sh, "
            "`source install/setup.bash` in THIS shell, and remove the apt copy "
            "(sudo apt remove ros-$ROS_DISTRO-motion-capture-tracking "
            "ros-$ROS_DISTRO-motion-capture-tracking-interfaces).")


NATNET_CMD_PORT = 1510
NATNET_CONNECT = 0      # NatNet 'ping'; Motive answers ServerInfo (what the SDK's PacketClient broadcasts)
NATNET_SERVERINFO = 1
NATNET_DISCOVERY = 14   # newer SDK discovery request; also answered with ServerInfo


def discover_motive_host(timeout_s=2.0):
    """Find the Motive PC by NatNet discovery: a NAT_CONNECT ping broadcast on
    UDP 1510. Motive answers with ServerInfo from its own address. Returns the
    IPv4 string of the (first) responder, or raises RuntimeError with guidance.
    The lab's Motive PC gets its address from DHCP and has changed several
    times (.100 → .124 → .152), which each time left /poses silent.
    """
    targets = {'255.255.255.255'}
    try:
        for iface in json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr'], timeout=2)):
            for a in iface.get('addr_info', []):
                if a.get('broadcast'):
                    targets.add(a['broadcast'])
    except Exception:
        pass  # limited broadcast alone is usually enough
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.2)
    for msg_id in (NATNET_CONNECT, NATNET_DISCOVERY):
        probe = struct.pack('<HH', msg_id, 0)
        for t in sorted(targets):
            try:
                sock.sendto(probe, (t, NATNET_CMD_PORT))
            except OSError:
                pass
    responders = []
    import time
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            data, (ip, _) = sock.recvfrom(4096)
        except socket.timeout:
            continue
        if len(data) >= 4 and struct.unpack_from('<H', data, 0)[0] == NATNET_SERVERINFO and ip not in responders:
            responders.append(ip)
    sock.close()
    if not responders:
        raise RuntimeError(
            "mocap hostname is 'auto' but no Motive answered a NatNet discovery ping "
            f"(UDP {NATNET_CMD_PORT} broadcast, {timeout_s:.0f}s). Check: you are on the Motive "
            "LAN (wired 192.168.9.x, `ip route get <motive-ip>` must not say 'via'); Motive is "
            "running with streaming enabled; the Windows firewall allows NatNet. Or set the "
            "address explicitly: launch arg mocap_hostname:=<ip>, env CRAZYSWARM_MOCAP_HOST=<ip>, "
            "or hostname: in motion_capture.yaml.")
    return responders


def resolve_mocap_hostname(context, yaml_hostname):
    """Effective Motive address: launch arg / env override > yaml value; 'auto' discovers."""
    override = LaunchConfiguration('mocap_hostname').perform(context).strip()
    host = override or str(yaml_hostname or '').strip()
    source = 'mocap_hostname launch arg / CRAZYSWARM_MOCAP_HOST' if override else 'motion_capture.yaml'
    if host.lower() in ('', 'auto'):
        found = discover_motive_host()
        host, source = found[0], 'NatNet discovery'
        if len(found) > 1:
            source += f' (several Motive hosts answered: {found}; using the first)'
    try:
        ipaddress.IPv4Address(host)
    except ValueError:
        raise RuntimeError(
            f"mocap hostname must be an IPv4 literal (the NatNet parser does not resolve names), got {host!r} from {source}.")
    return host, source


def parse_yaml(context):
    check_vendored_mocap_driver(context)
    # Load the crazyflies YAML file
    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml, 'r') as file:
        crazyflies = yaml.safe_load(file)
    # store the fileversion
    fileversion = 1
    if "fileversion" in crazyflies:
        fileversion = crazyflies["fileversion"]

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]
    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_params[1]['robot_description'] = robot_desc

    # Packet tracing, off unless asked for:  trace_cf:=all   or  trace_cf:=cf1,cf3
    # `ros2 launch` has no --ros-args, so this has to be a launch argument that is
    # folded into the server's parameters here. Read at connect by design (this
    # rig's same-process /parameter_events never loop back -- see CLAUDE.md).
    server_params[1]['debug.trace_cf'] = \
        LaunchConfiguration('trace_cf').perform(context)

    # construct motion_capture_configuration
    motion_capture_yaml = LaunchConfiguration('motion_capture_yaml_file').perform(context)
    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture_content = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture_content['/motion_capture_tracking']['ros__parameters']
    motion_capture_params['rigid_bodies'] = dict()
    mocap_log = []
    if _mocap_enabled(context):
        host, host_source = resolve_mocap_hostname(context, motion_capture_params.get('hostname'))
        motion_capture_params['hostname'] = host
        mocap_log.append(LogInfo(msg=f"mocap: Motive at {host} (from {host_source}), parser type '{motion_capture_params.get('type')}'"))
    for key, value in crazyflies['robots'].items():
        type = crazyflies['robot_types'][value['type']]
        if value['enabled'] and \
            ((fileversion == 1 and type['motion_capture']['enabled']) or \
            ((fileversion >= 2 and type['motion_capture']['tracking'] == "librigidbodytracker"))):
            motion_capture_params['rigid_bodies'][key] =  {
                    'initial_position': value['initial_position'],
                    'marker': type['motion_capture']['marker'],
                    'dynamics': type['motion_capture']['dynamics'],
                }

    # copy relevent settings to server params
    server_params[1]['poses_qos_deadline'] = motion_capture_params['topics']['poses']['qos']['deadline']
    
    return mocap_log + [
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' != 'sim' and '", LaunchConfiguration('mocap'), "'.lower() in ('true', '1')"])),
            name='motion_capture_tracking',
            output='screen',
            emulate_tty=True,   # line-buffer the parser's stdout so 'NatNet: ...' lines show live
            # The vendored parser now aborts (instead of hanging) when Motive does not
            # answer within 5 s; respawn so the error repeats loudly and the node
            # recovers by itself once Motive / the network is back.
            respawn=True,
            respawn_delay=3.0,
            parameters= [motion_capture_params],
        ),
        # `server:=False` brings up the mocap, RViz and the preflight GUI and
        # starts NO server, so nothing opens the radio and nothing can be
        # armed. That is the state scripts/sync_initial_positions.py documents
        # as its requirement ("mocap up, server not yet started"), and until
        # 2026-10-02 the launch had no way to produce it -- you launched
        # everything, synced, then restarted to make the yaml take effect.
        Node(
            package='crazyflie_server_py',
            executable='crazyflie_server',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('backend'), "' == 'cflib' and '",
                 LaunchConfiguration('server'), "'.lower() in ('true', '1')"])),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('backend'), "' == 'cpp' and '",
                 LaunchConfiguration('server'), "'.lower() in ('true', '1')"])),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
            prefix=PythonExpression(['"xterm -e gdb -ex run --args" if ', LaunchConfiguration('debug'), ' else ""']),
        ),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('backend'), "' == 'sim' and '",
                 LaunchConfiguration('server'), "'.lower() in ('true', '1')"])),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters= server_params,
        )]

def generate_launch_description():
    default_crazyflies_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    
    default_motion_capture_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'motion_capture.yaml')

    default_rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'config.rviz')

    telop_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('crazyflies_yaml_file', 
                              default_value=default_crazyflies_yaml_path),
        DeclareLaunchArgument('motion_capture_yaml_file', 
                              default_value=default_motion_capture_yaml_path),
        DeclareLaunchArgument('rviz_config_file', 
                              default_value=default_rviz_config_path),
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('gui', default_value='False'),
        DeclareLaunchArgument('preflight', default_value='True'),
        DeclareLaunchArgument('foxglove', default_value='True'),
        DeclareLaunchArgument('teleop', default_value='True'),
        DeclareLaunchArgument('mocap', default_value='True'),
        # False = mocap/GUIs only, no server, no radio, nothing armable.
        DeclareLaunchArgument('server', default_value='True'),
        # '' = off, 'all' = every drone, or a comma-separated list (cf1,cf3).
        DeclareLaunchArgument('trace_cf', default_value=''),
        DeclareLaunchArgument('mocap_hostname', default_value=EnvironmentVariable('CRAZYSWARM_MOCAP_HOST', default_value=''),
                              description='Motive PC IPv4; overrides motion_capture.yaml hostname. Empty = use yaml; yaml "auto" = NatNet discovery'),
        DeclareLaunchArgument('teleop_yaml_file', default_value=''),
        OpaqueFunction(function=parse_yaml),
        Node(
            condition=LaunchConfigurationEquals('teleop', 'True'),
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('emergency', 'all/emergency'),
                ('arm', 'all/arm'),
                ('takeoff', 'all/takeoff'),
                ('land', 'all/land'),
                # uncomment to manually control (and update teleop.yaml)
                # ('cmd_vel_legacy', 'cf6/cmd_vel_legacy'),
                # ('cmd_full_state', 'cf6/cmd_full_state'),
                # ('notify_setpoints_stop', 'cf6/notify_setpoints_stop'),
            ],
            parameters= [PythonExpression(["'" + telop_yaml_path +"' if '", LaunchConfiguration('teleop_yaml_file'), "' == '' else '", LaunchConfiguration('teleop_yaml_file'), "'"])],
        ),
        Node(
            condition=LaunchConfigurationEquals('teleop', 'True'),
            package='joy',
            executable='joy_node',
            name='joy_node' # by default id=0
        ),
        Node(
            condition=LaunchConfigurationEquals('rviz', 'True'),
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('gui', 'True'),
            package='crazyflie',
            namespace='',
            executable='gui.py',
            name='gui',
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('preflight', 'True'),
            package='crazyflie',
            namespace='',
            executable='preflight_kalman_plotter.py',
            name='preflight_kalman_plotter',
            output='screen',
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('foxglove', 'True'),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                "port": 8765,
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
    ])
