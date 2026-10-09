"""Tests for the mission specs, the usage log and the process changes behind
the mission window. Stdlib only, no ROS needed:

    cd console && python3 -m unittest discover -s tests -v
"""

import os
import signal
import sys
import tempfile
import textwrap
import time
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from mission_console import missions as M          # noqa: E402
from mission_console import procs as P             # noqa: E402


def spec(text):
    return M.normalise(__import__('yaml').safe_load(textwrap.dedent(text)), 'pkg', 'pkg/t',
                       tempfile.gettempdir())


class ShippedSpecs(unittest.TestCase):
    """The specs in crazyflie_shows/missions must load and build the commands
    the runbooks document."""

    def test_all_load(self):
        found = {m['id']: m for m in M.discover()}
        for mid in ('console/script', 'crazyflie_shows/escort',
                    'crazyflie_shows/carousel', 'crazyflie_shows/constellation'):
            self.assertIn(mid, found)
            self.assertIsNone(found[mid].get('error'), found[mid].get('error'))

    def test_escort_default_is_the_runbook_command(self):
        m = M.get('crazyflie_shows/escort')
        _, cmd, _ = M.build(m, 'main', {})
        # runbooks/ESCORT.runcard.md, "the show, operator-paced (recommended)"
        self.assertEqual(cmd, 'ros2 run crazyflie_shows escort_show --ros-args '
                              '-p vip_mode:=point -p adversary:=scripted -p paced:=true')

    def test_escort_sim_and_helper(self):
        m = M.get('crazyflie_shows/escort')
        argv, _, _ = M.build(m, 'main', {'vip_mode': 'manual'}, sim=True)
        self.assertEqual(argv[argv.index('-p') + 1], 'use_sim_time:=true')
        self.assertEqual([h['key'] for h in M.active_helpers(m, {'vip_mode': 'manual'})], ['vip_keys'])
        self.assertEqual(M.active_helpers(m, {}), [])
        _, cmd, _ = M.build(m, 'vip_keys', {}, sim=True)
        self.assertEqual(cmd, 'ros2 run crazyflie_shows escort_teleop --ros-args '
                              '-p use_sim_time:=true -p target:=vip')

    def test_conditional_option_only_when_it_applies(self):
        m = M.get('crazyflie_shows/escort')
        _, cmd, _ = M.build(m, 'main', {'vip_name': 'dji'})
        self.assertNotIn('vip_name', cmd)                 # vip_mode is point
        _, cmd, _ = M.build(m, 'main', {'vip_mode': 'mocap', 'vip_name': 'dji'})
        self.assertIn('-p vip_name:=dji', cmd)

    def test_operator_values_are_validated(self):
        m = M.get('crazyflie_shows/escort')
        with self.assertRaises(ValueError):
            M.build(m, 'main', {'vip_mode': 'evil'})
        with self.assertRaises(ValueError):
            M.build(m, 'main', {'vip_mode': 'mocap', 'vip_name': 'two words'})
        with self.assertRaises(ValueError):
            M.build(m, 'main', {'extra': 'not-a-param'})
        _, cmd, _ = M.build(m, 'main', {'extra': 'ring_radius:=1.1'})
        self.assertTrue(cmd.endswith('-p ring_radius:=1.1'))

    def test_builtin_only_runs_listed_scripts(self):
        m = M.get('console/script')
        with self.assertRaises(ValueError):
            M.build(m, 'main', {'script': 'crazyflie_shows rm'})


class SpecValidation(unittest.TestCase):
    def test_minimal(self):
        m = spec("""
            title: T
            run: show
        """)
        self.assertEqual(m['run'], {'pkg': 'pkg', 'exe': 'show'})
        self.assertEqual(m['layout'], M.DEFAULT_LAYOUT)
        self.assertEqual(m['abort'], 'sigint')

    def test_refusals(self):
        bad = [
            'title: T\nrun: a b c',                                    # run shape
            'mission: 2\ntitle: T\nrun: s',                            # version
            'title: T\nrun: s\nlayout: {main: [radar]}',               # unknown widget
            'title: T\nrun: s\noptions: [{name: m, choices: [a, b], default: c}]',
            'title: T\nrun: s\nhelpers: {k: {run: t, when: {nope: x}}}',
            'title: T\nrun: s\nhelpers: {k: {run: t, teleop: joystick}}',
            'title: T\nrun: s\nabort: sigterm',
            'title: T\nrun: s\nevents: ["(unclosed"]',
            'title: T\nrun: s\nview: ../../etc/passwd.js',
        ]
        for text in bad:
            with self.subTest(text=text), self.assertRaises(M.SpecError):
                spec(text)

    def test_regex_dialects(self):
        self.assertEqual(M._regex(r'>>> (?P<text>.+)', 'x'), r'>>> (?<text>.+)')
        self.assertEqual(M._regex(r'>>> (?<text>.+)', 'x'), r'>>> (?<text>.+)')
        self.assertEqual(M._regex(r'(?<=a)b(?<!c)', 'x'), r'(?<=a)b(?<!c)')
        self.assertEqual(M._regex(r'(?i)error', 'x'), r'(?i)error')
        for bad in (r'a(?i)b', r'(?m)^x', r'(?s:.)'):
            with self.assertRaises(M.SpecError):
                M._regex(bad, 'x')

    def test_asset_path_stays_in_the_spec_dir(self):
        with tempfile.TemporaryDirectory() as d:
            open(os.path.join(d, 'view.mjs'), 'w').close()
            m = {'base_dir': d}
            self.assertTrue(M.asset_path(m, 'view.mjs'))
            self.assertIsNone(M.asset_path(m, '../view.mjs'))
            self.assertIsNone(M.asset_path(m, '/etc/passwd'))


class Usage(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        os.environ['MISSION_CONSOLE_USAGE_DIR'] = self.dir.name
        import importlib
        from mission_console import usage
        self.u = importlib.reload(usage)

    def tearDown(self):
        os.environ.pop('MISSION_CONSOLE_USAGE_DIR', None)
        self.dir.cleanup()

    def test_record_and_summary(self):
        u = self.u
        u.record('run', action='fly.script', source='dash')
        u.record('run', action='fly.script', source='palette')
        u.record('run', action='srv.land_all', source='dash')
        u.record('tab', tab='health')
        u.record('mission_start', mission='m', role='main')
        u.record('mission_start', mission='m', role='vip_keys')
        s = u.summary(labels={'fly.script': 'Fly'})
        self.assertEqual(s['actions'][0]['id'], 'fly.script')
        self.assertEqual(s['actions'][0]['count'], 2)
        self.assertEqual(s['actions'][0]['label'], 'Fly')
        self.assertEqual(s['actions'][0]['sources'], {'dash': 1, 'palette': 1})
        self.assertEqual(s['tabs'], {'health': 1})
        self.assertEqual(s['missions']['m']['starts'], 1)        # helpers are not runs

    def test_stdin_content_is_never_kept(self):
        self.assertEqual(self.u.stdin_shape('\n'), 'enter')
        self.assertEqual(self.u.stdin_shape('q\n'), 'q')
        self.assertEqual(self.u.stdin_shape('hunter2\n'), 'text')

    def test_prefs(self):
        self.assertEqual(self.u.load_prefs(), {})
        self.u.save_prefs({'pins': ['a'] * 40, 'junk': 1})
        p = self.u.load_prefs()
        self.assertEqual(len(p['pins']), 24)
        self.assertNotIn('junk', p)


class Builds(unittest.TestCase):
    """builds.py against a fake src/ + install/ tree."""

    def setUp(self):
        from mission_console import builds
        self.b = builds
        self.tmp = tempfile.TemporaryDirectory()
        root = self.tmp.name
        self.saved = (builds.SRC, builds.INSTALL, builds.REPO)
        builds.SRC, builds.INSTALL, builds.REPO = (os.path.join(root, 'src'),
                                                   os.path.join(root, 'install'), root)
        src = os.path.join(root, 'src', 'myshows')
        os.makedirs(os.path.join(src, 'myshows'))
        with open(os.path.join(src, 'package.xml'), 'w') as fh:
            fh.write('<package><name>myshows</name><exec_depend>crazyflie_py</exec_depend></package>')
        with open(os.path.join(src, 'setup.cfg'), 'w') as fh:
            fh.write('[options.entry_points]\nconsole_scripts =\n'
                     '    old_show = myshows.old_show:main\n    new_show = myshows.new_show:main\n')
        for mod, body in (('old_show', 'from myshows import util\n'), ('util', ''),
                          ('new_show', 'from . import fresh\n'), ('fresh', '')):
            with open(os.path.join(src, 'myshows', mod + '.py'), 'w') as fh:
                fh.write(body)
        self.src = src

    def tearDown(self):
        self.b.SRC, self.b.INSTALL, self.b.REPO = self.saved
        self.tmp.cleanup()

    def install(self, exes, mods):
        inst = os.path.join(self.b.INSTALL, 'myshows')
        os.makedirs(os.path.join(inst, 'share', 'myshows'), exist_ok=True)
        open(os.path.join(inst, 'share', 'myshows', 'package.xml'), 'w').close()
        os.makedirs(os.path.join(inst, 'lib', 'myshows'), exist_ok=True)
        moddir = os.path.join(inst, 'local', 'lib', 'python3.10', 'dist-packages', 'myshows')
        os.makedirs(moddir, exist_ok=True)
        for e in exes:
            open(os.path.join(inst, 'lib', 'myshows', e), 'w').close()
        for m in mods:
            open(os.path.join(moddir, m), 'w').close()

    def test_never_built(self):
        (st,) = self.b.status()
        self.assertFalse(st['installed'])
        self.assertIn('never built', st['problems'][0])
        self.assertEqual(st['build_cmd'], './scripts/build.sh myshows')

    def test_new_script_and_module(self):
        self.install(['old_show'], ['old_show.py', 'util.py'])
        (st,) = self.b.status()
        self.assertEqual(st['missing_exes'], ['new_show'])
        self.assertEqual(st['new_modules'], ['fresh.py', 'new_show.py'])
        self.assertFalse(st['built'])

    def test_fully_built(self):
        self.install(['old_show', 'new_show'],
                     ['old_show.py', 'util.py', 'new_show.py', 'fresh.py'])
        (st,) = self.b.status()
        self.assertTrue(st['built'], st['problems'])

    def test_imports_decide_which_scripts_a_new_module_breaks(self):
        self.assertEqual(self.b.script_modules('myshows', self.src, 'old_show'),
                         {'old_show.py', 'util.py'})
        self.assertEqual(self.b.script_modules('myshows', self.src, 'new_show'),
                         {'new_show.py', 'fresh.py'})


class Settings(unittest.TestCase):
    """One store for every choice (settings.py) and how render() applies it."""

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        os.environ['MISSION_CONSOLE_USAGE_DIR'] = self.dir.name
        import importlib
        from mission_console import usage, settings
        importlib.reload(usage)
        self.s = importlib.reload(settings)

    def tearDown(self):
        os.environ.pop('MISSION_CONSOLE_USAGE_DIR', None)
        self.dir.cleanup()

    def test_store(self):
        self.assertEqual(self.s.load(), {})
        self.s.update({'rviz': 'False', 'srv.takeoff_all.height': '0.7'})
        self.assertEqual(self.s.load()['rviz'], 'False')
        self.s.update({'rviz': None})
        self.assertNotIn('rviz', self.s.load())
        with self.assertRaises(ValueError):
            self.s.update({'bad key with spaces': '1'})

    def test_one_setting_reaches_every_launch_button(self):
        from mission_console import catalog
        acts = {a['id']: a for a in catalog.build_catalog([])}
        st = {'rviz': 'False', 'backend': 'sim'}
        for aid in ('launch.stack', 'launch.restart'):
            _, cmd = catalog.render(acts[aid], {}, st)
            self.assertIn('rviz:=False', cmd)
            self.assertIn('backend:=sim', cmd)
        _, cmd = catalog.render(acts['launch.mocap'], {}, st)
        self.assertIn('rviz:=False', cmd)
        # what the request carries still wins
        _, cmd = catalog.render(acts['launch.stack'], {'rviz': 'True'}, st)
        self.assertIn('rviz:=True', cmd)
        # a takeoff height is not a landing height
        self.assertEqual(catalog.setting_key('srv.takeoff_all', 'height'), 'srv.takeoff_all.height')
        self.assertEqual(catalog.setting_key('check.status_once', 'drone'), 'drone')
        # a stale select value is ignored, not run
        _, cmd = catalog.render(acts['launch.stack'], {}, {'backend': 'gone'})
        self.assertIn('backend:=cpp', cmd)

    def test_sim_clock_auto_follows_the_stack(self):
        from mission_console import catalog
        fly = next(a for a in catalog.build_catalog([]) if a['id'] == 'fly.script')
        _, cmd = catalog.render(fly, {'__stack_is_sim': True}, {})
        self.assertIn('use_sim_time:=true', cmd)
        _, cmd = catalog.render(fly, {'__stack_is_sim': False}, {})
        self.assertNotIn('use_sim_time', cmd)


class Journal(unittest.TestCase):
    """/rosout lines -> fleet commands, from anyone."""

    def test_parse(self):
        from mission_console import journal as J
        sim = J.classify(J.parse_csv_line(
            '1791531941,931411909,20,crazyflie_server,[all] emergency not yet implemented,'
            '/x/crazyflie_server.py,_emergency_callback,253'))
        self.assertEqual((sim['target'], sim['verb'], sim['sim_noop']), ('all', 'emergency', True))
        real = J.classify(J.parse_csv_line(
            '1,2,20,/crazyflie_server,[cf3] land(height=0.040000 m, duration=4.000000 s, group_mask=0),'
            'f.cpp,land,996'))
        self.assertEqual((real['target'], real['verb']), ('cf3', 'land'))
        self.assertIn('duration=4.000000 s', real['detail'])           # commas inside msg survive
        self.assertIsNone(J.classify(J.parse_csv_line('1,2,20,/rviz2,[all] emergency(),f,g,1')))
        self.assertIsNone(J.classify(J.parse_csv_line('1,2,20,crazyflie_server,connected to cf1,f,g,1')))
        self.assertIsNone(J.parse_csv_line('garbage'))


class Census(unittest.TestCase):
    """system.py: rig processes, whoever started them, on any domain."""

    def test_classify(self):
        from mission_console import system
        exes = {('crazyflie_shows', 'escort_show'), ('crazyflie_shows', 'plan_show')}
        role, d = system._classify(['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'crazyflie',
                                    'launch.py', 'backend:=sim', 'server:=False'], exes)
        self.assertEqual((role, d['backend'], d['server']), ('launch', 'sim', False))
        role, d = system._classify(['/x/install/crazyflie_sim/lib/crazyflie_sim/crazyflie_server'], exes)
        self.assertEqual((role, d['backend']), ('server', 'sim'))
        role, d = system._classify(['/usr/bin/python3', '/w/install/crazyflie_shows/lib/crazyflie_shows/escort_show',
                                    '--ros-args', '-p', 'paced:=true'], exes)
        self.assertEqual((role, d['exe'], d['ground'], d['params']['paced']), ('script', 'escort_show', False, 'true'))
        self.assertIsNone(system._classify(['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run',
                                            'crazyflie_shows', 'escort_show'], exes))   # the wrapper, not the script

    def test_finds_a_process_on_another_domain(self):
        """A fake mocap node, started outside the console on ROS_DOMAIN_ID=55."""
        from mission_console import system
        with tempfile.TemporaryDirectory() as d:
            node = os.path.join(d, 'motion_capture_tracking_node')
            with open(node, 'w') as fh:
                fh.write('#!/bin/sh\nsleep 20\n')
            os.chmod(node, 0o755)
            import subprocess
            child = subprocess.Popen([node], env={**os.environ, 'ROS_DOMAIN_ID': '55'},
                                     start_new_session=True)
            try:
                time.sleep(0.3)
                procs = system.census({}, my_domain='1')
                hit = [p for p in procs if p['pid'] == child.pid]
                self.assertEqual(len(hit), 1, procs)
                self.assertEqual((hit[0]['role'], hit[0]['domain'], hit[0]['by'], hit[0]['other_domain']),
                                 ('mocap', '55', 'outside', True))
                summ = system.summarise(procs, '1')
                self.assertTrue(summ['mocap_any'] and summ['stack_any'])
                self.assertIn('55', summ['other_domains'])
                # and the console's own child is recognised as its own
                procs = system.census({child.pid: 'p9'}, my_domain='1')
                self.assertEqual([p['by'] for p in procs if p['pid'] == child.pid], ['console'])
            finally:
                child.kill()
                child.wait()


class Procs(unittest.TestCase):
    CHILD = textwrap.dedent('''
        import signal, sys, time
        def h(sig, _):
            print('got', signal.Signals(sig).name, flush=True)
        signal.signal(signal.SIGINT, h)
        signal.signal(signal.SIGTERM, h)
        sys.stdout.write('Enter to go: '); sys.stdout.flush()
        time.sleep(%s)
        print('bye', flush=True)
    ''')

    def setUp(self):
        self.bus = P.EventBus()
        self.sub = self.bus.subscribe()
        self.pm = P.ProcessManager(self.bus, tempfile.gettempdir())

    def events(self, secs):
        out, end = [], time.time() + secs
        while time.time() < end:
            e = self.sub.get(0.1)
            if e:
                out.append(e)
        return out

    def test_partial_prompt_is_published(self):
        p = self.pm.start('t', 't', [sys.executable, '-c', self.CHILD % 3])
        ev = self.events(1.2)
        parts = [e for e in ev if e['type'] == 'partial']
        self.assertTrue(parts and parts[0]['text'] == 'Enter to go:', ev)
        self.assertEqual(p.summary()['partial'], 'Enter to go:')
        self.pm.stop(p.id, hard=True)

    def test_flight_scripts_get_a_longer_sigint_grace(self):
        self.assertTrue(P.is_flight('fly.script'))
        self.assertTrue(P.is_flight('mission:crazyflie_shows/escort:main'))
        self.assertFalse(P.is_flight('mission:crazyflie_shows/escort:vip_keys'))
        self.assertFalse(P.is_flight('launch.stack'))
        # longer than abort.py's landing (ABORT_LAND_DURATION 4 s + 0.5 s) with margin
        self.assertGreaterEqual(P.FLIGHT_STOP_ESCALATION[0][1], 10.0)

    def test_interrupt_is_one_sigint_and_nothing_follows(self):
        # stop() would follow SIGINT with SIGTERM at 6 s; interrupt() must not
        p = self.pm.start('t', 't', [sys.executable, '-c', self.CHILD % 9])
        time.sleep(0.5)
        self.assertTrue(self.pm.interrupt(p.id))
        texts = [e['text'] for e in self.events(7.5)
                 if e['type'] == 'line' and not e['text'].startswith('$ ')]   # not the echoed source
        self.assertIn('Enter to go: got SIGINT', texts)
        self.assertFalse(any('SIGTERM' in t for t in texts), texts)
        self.assertTrue(p.running)
        self.assertEqual(p.state(), 'stopping')
        self.pm.stop(p.id, hard=True)


if __name__ == '__main__':
    unittest.main()
