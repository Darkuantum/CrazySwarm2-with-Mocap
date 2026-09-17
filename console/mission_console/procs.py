"""Process manager: run ros2 commands, stream their output, stop them cleanly.

Everything the console runs is a plain subprocess of an ALREADY ROS-sourced
environment (see console/run.sh), so the argv shown in the UI is byte-for-byte
the argv executed -- that is the whole point of this module's existence: you
can copy any command out of the UI into a terminal and get the same result.

Two deliberate design choices:

* **A pty, not a pipe.** libc line-buffers on a tty and full-buffers on a pipe,
  so `ros2 launch` / the C++ server would otherwise deliver their output in
  4 KiB lumps (or not at all until exit). The pty also lets us feed stdin to
  interactive helpers such as `scripts/led.sh`.
* **A new session per child** (`start_new_session=True`). `ros2 launch` forks a
  tree of nodes; signalling only the launcher leaves orphans holding the radio
  and UDP 1511 -- the documented cause of the next launch aborting. We signal
  the whole process group and escalate SIGINT -> SIGTERM -> SIGKILL, because a
  frozen mocap node blocked in recv() ignores SIGINT.
"""

from __future__ import annotations

import errno
import itertools
import os
import pty
import re
import shlex
import signal
import subprocess
import threading
import time
from collections import deque

MAX_LINES = 4000          # per-process ring buffer
STOP_ESCALATION = (       # (signal, seconds to wait before the next one)
    (signal.SIGINT, 6.0),
    (signal.SIGTERM, 4.0),
    (signal.SIGKILL, 2.0),
)
_ANSI = re.compile(r'\x1b\[[0-9;?]*[a-zA-Z]|\x1b\][^\x07]*\x07|\r')


class Subscriber:
    """One SSE client's mailbox. Identity-based so two idle clients never
    compare equal (a plain (deque, sem) tuple would: empty deques are ==)."""

    def __init__(self):
        self.queue = deque(maxlen=2000)
        self.wake = threading.Semaphore(0)

    def get(self, timeout):
        if not self.wake.acquire(timeout=timeout):
            return None
        try:
            return self.queue.popleft()
        except IndexError:
            return None


class EventBus:
    """Fan-out of JSON-able events to every connected SSE client."""

    def __init__(self):
        self._lock = threading.Lock()
        self._subscribers = []

    def subscribe(self):
        sub = Subscriber()
        with self._lock:
            self._subscribers.append(sub)
        return sub

    def unsubscribe(self, sub):
        with self._lock:
            self._subscribers = [s for s in self._subscribers if s is not sub]

    def publish(self, event):
        with self._lock:
            subs = list(self._subscribers)
        for sub in subs:
            sub.queue.append(event)
            sub.wake.release()


class Proc:
    """One running (or finished) command."""

    _ids = itertools.count(1)

    def __init__(self, action_id, label, argv, cwd, kind='task', note=''):
        self.id = f'p{next(Proc._ids)}'
        self.action_id = action_id
        self.label = label
        self.argv = list(argv)
        self.cwd = cwd
        self.kind = kind              # 'service' (long-running) | 'task' (one-shot)
        self.note = note
        self.started = time.time()
        self.ended = None
        self.returncode = None
        self.lines = deque(maxlen=MAX_LINES)
        self.seq = 0
        self.popen = None
        self.master_fd = None
        self.stop_requested = False
        self._lock = threading.Lock()

    @property
    def cmdline(self):
        return ' '.join(shlex.quote(a) for a in self.argv)

    @property
    def running(self):
        return self.popen is not None and self.returncode is None

    def state(self):
        if self.running:
            return 'stopping' if self.stop_requested else 'running'
        if self.returncode is None:
            return 'unknown'
        # A command we asked to stop is not a failure, whatever it exited with:
        # `ros2 topic hz` leaves 2 on SIGINT, launch leaves 130, and so on.
        if self.stop_requested or self.returncode in (-2, 130, -15, 143, -9, 137):
            return 'stopped'
        return 'done' if self.returncode == 0 else 'failed'

    def summary(self):
        return {
            'id': self.id,
            'action_id': self.action_id,
            'label': self.label,
            'cmdline': self.cmdline,
            'cwd': self.cwd,
            'kind': self.kind,
            'note': self.note,
            'state': self.state(),
            'returncode': self.returncode,
            'started': self.started,
            'ended': self.ended,
            'line_count': self.seq,
        }

    def tail(self, after=0, limit=MAX_LINES):
        with self._lock:
            rows = [r for r in self.lines if r[0] > after]
        return rows[-limit:]


class ProcessManager:
    def __init__(self, bus: EventBus, cwd: str, env=None):
        self.bus = bus
        self.cwd = cwd
        self.env = dict(env or os.environ)
        self.procs = {}
        self._lock = threading.Lock()

    # --- lifecycle ---------------------------------------------------------
    def start(self, action_id, label, argv, kind='task', note='', cwd=None):
        proc = Proc(action_id, label, argv, cwd or self.cwd, kind, note)
        with self._lock:
            self.procs[proc.id] = proc

        master, slave = pty.openpty()
        try:
            os.set_blocking(master, True)
            env = dict(self.env)
            env.setdefault('PYTHONUNBUFFERED', '1')
            env.setdefault('TERM', 'dumb')
            # ros2 CLI colours its output; TERM=dumb + this keeps the log clean.
            env.setdefault('RCUTILS_COLORIZED_OUTPUT', '0')
            proc.popen = subprocess.Popen(
                proc.argv, cwd=proc.cwd, env=env,
                stdin=slave, stdout=slave, stderr=slave,
                start_new_session=True, close_fds=True)
        except Exception as exc:                       # noqa: BLE001 - reported to UI
            os.close(master)
            os.close(slave)
            proc.returncode = 127
            proc.ended = time.time()
            self._emit_line(proc, f'!! failed to start: {exc}')
            self._emit_state(proc)
            return proc
        finally:
            try:
                os.close(slave)
            except OSError:
                pass

        proc.master_fd = master
        self._emit_line(proc, f'$ {proc.cmdline}')
        self._emit_state(proc)
        threading.Thread(target=self._pump, args=(proc,), daemon=True).start()
        return proc

    def _pump(self, proc):
        """Read the pty until EOF, split into lines, broadcast each one."""
        buf = b''
        while True:
            try:
                chunk = os.read(proc.master_fd, 65536)
            except OSError as exc:
                if exc.errno == errno.EINTR:
                    continue
                chunk = b''          # EIO = the child closed the slave side
            if not chunk:
                break
            buf += chunk
            *complete, buf = buf.split(b'\n')
            for raw in complete:
                self._emit_line(proc, _clean(raw))
            if len(buf) > 16384:     # a very long unterminated line: flush it
                self._emit_line(proc, _clean(buf))
                buf = b''
        if buf:
            self._emit_line(proc, _clean(buf))
        try:
            os.close(proc.master_fd)
        except OSError:
            pass
        proc.returncode = proc.popen.wait()
        proc.ended = time.time()
        self._emit_line(proc, f'[exit {proc.returncode}]')
        self._emit_state(proc)

    def stop(self, proc_id, hard=False):
        proc = self.procs.get(proc_id)
        if proc is None or not proc.running:
            return False
        proc.stop_requested = True
        threading.Thread(target=self._stop_worker, args=(proc, hard), daemon=True).start()
        return True

    def _stop_worker(self, proc, hard):
        steps = STOP_ESCALATION[2:] if hard else STOP_ESCALATION
        try:
            pgid = os.getpgid(proc.popen.pid)
        except OSError:
            return
        for sig, wait_s in steps:
            self._emit_line(proc, f'[console] sending {sig.name} to process group {pgid}')
            try:
                os.killpg(pgid, sig)
            except OSError:
                return
            deadline = time.time() + wait_s
            while time.time() < deadline:
                if not proc.running:
                    return
                time.sleep(0.1)
        self._emit_line(proc, '[console] process survived SIGKILL -- check `pgrep -f` by hand')

    def send_input(self, proc_id, text):
        proc = self.procs.get(proc_id)
        if proc is None or not proc.running or proc.master_fd is None:
            return False
        try:
            os.write(proc.master_fd, text.encode())
            return True
        except OSError:
            return False

    # --- queries -----------------------------------------------------------
    def get(self, proc_id):
        return self.procs.get(proc_id)

    def list(self):
        with self._lock:
            procs = list(self.procs.values())
        return [p.summary() for p in sorted(procs, key=lambda p: p.started)]

    def running_actions(self):
        """Set of action ids with at least one live process (drives the UI badges)."""
        return {p.action_id for p in self.procs.values() if p.running}

    def find_running(self, action_id):
        for p in self.procs.values():
            if p.running and p.action_id == action_id:
                return p
        return None

    def output_of(self, action_id, max_lines=400):
        """Recent output of the newest process for an action (for log diagnosis)."""
        matches = [p for p in self.procs.values() if p.action_id == action_id]
        if not matches:
            return []
        newest = max(matches, key=lambda p: p.started)
        return [text for _, _, text in newest.tail()][-max_lines:]

    def prune(self):
        """Drop finished processes, keeping the newest few for reference."""
        with self._lock:
            finished = sorted((p for p in self.procs.values() if not p.running),
                              key=lambda p: p.started)
            for p in finished[:-6]:
                self.procs.pop(p.id, None)

    # --- events ------------------------------------------------------------
    def _emit_line(self, proc, text):
        with proc._lock:
            proc.seq += 1
            row = (proc.seq, time.time(), text)
            proc.lines.append(row)
        self.bus.publish({'type': 'line', 'proc': proc.id, 'seq': row[0], 'text': text})

    def _emit_state(self, proc):
        self.bus.publish({'type': 'proc', 'proc': proc.summary()})


def _clean(raw: bytes) -> str:
    return _ANSI.sub('', raw.decode('utf-8', 'replace')).rstrip()


def run_capture(argv, timeout=6.0, cwd=None, env=None):
    """Run a short command and capture it -- used by the health probes.

    Never raises: a missing binary or a timeout is returned as a normal result
    so one broken probe can never take the health view down with it.
    """
    started = time.time()
    try:
        cp = subprocess.run(argv, cwd=cwd, env=env, timeout=timeout,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = cp.stdout.decode('utf-8', 'replace')
        return {'rc': cp.returncode, 'out': out, 'timeout': False,
                'cmd': ' '.join(shlex.quote(a) for a in argv),
                'elapsed': time.time() - started}
    except subprocess.TimeoutExpired as exc:
        out = (exc.stdout or b'').decode('utf-8', 'replace')
        return {'rc': None, 'out': out, 'timeout': True,
                'cmd': ' '.join(shlex.quote(a) for a in argv),
                'elapsed': time.time() - started}
    except FileNotFoundError:
        return {'rc': 127, 'out': f'{argv[0]}: not found', 'timeout': False,
                'cmd': ' '.join(shlex.quote(a) for a in argv),
                'elapsed': time.time() - started}
    except Exception as exc:                            # noqa: BLE001
        return {'rc': 1, 'out': str(exc), 'timeout': False,
                'cmd': ' '.join(shlex.quote(a) for a in argv),
                'elapsed': time.time() - started}
