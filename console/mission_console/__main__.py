"""Entry point: python3 -m mission_console (see console/run.sh)."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
import webbrowser


def main():
    ap = argparse.ArgumentParser(
        prog='mission_console',
        description='Web console for the CrazySwarm2 + OptiTrack rig.')
    ap.add_argument('--host', default='127.0.0.1',
                    help='bind address (default 127.0.0.1; use 0.0.0.0 to reach it '
                         'from another machine on the lab network)')
    ap.add_argument('--port', type=int, default=8077)
    ap.add_argument('--no-browser', action='store_true')
    args = ap.parse_args()

    from .server import APP, serve      # imported late so --help never touches ROS

    try:
        httpd = serve(args.host, args.port)
    except OSError as exc:
        print(f'console: cannot listen on {args.host}:{args.port} -- {exc}')
        print(f'console: another console is probably already running. Open '
              f'http://localhost:{args.port}/ , or start this one with --port <other>.')
        return 1
    url = f'http://{"localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host}:{args.port}/'
    print(f'\n  CrazySwarm2 mission console -> {url}')
    print('  Ctrl-C to stop. Long-running processes started here are signalled on exit.\n')
    if not args.no_browser:
        def _open():
            # webbrowser.open() returns False when it cannot find a browser at
            # all (no DISPLAY over ssh, no xdg-open); say so instead of leaving
            # the user wondering why nothing appeared. It can also "succeed"
            # into an existing window on another workspace, hence the URL echo.
            try:
                opened = webbrowser.open(url, new=2)
            except Exception as exc:          # noqa: BLE001 - never kill the server
                opened = False
                print(f'  could not open a browser: {exc}')
            if not opened:
                print(f'  no browser opened automatically -- open {url} yourself'
                      f'{" (no DISPLAY set)" if not os.environ.get("DISPLAY") else ""}')
        threading.Timer(0.7, _open).start()

    stopping = threading.Event()

    def _shutdown():
        """Runs on its own thread: httpd.shutdown() deadlocks if it is called
        from the thread that is inside serve_forever(), which is exactly where a
        signal handler runs."""
        live = [p for p in APP.procs.procs.values() if p.running]
        for p in live:
            print(f'  SIGINT -> {p.label} ({p.cmdline[:60]})')
            APP.procs.stop(p.id)
        if live:
            time.sleep(2.0)
        httpd.shutdown()

    def on_signal(signum, frame):
        if stopping.is_set():
            print('\nsecond signal: exiting now')
            os._exit(1)
        stopping.set()
        print('\nstopping console (long-running processes get SIGINT)...')
        threading.Thread(target=_shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
