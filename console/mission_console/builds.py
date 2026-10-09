"""Is every show in src/ actually built? -- the "I added a show, why can't I run it" check.

`scripts/build.sh` uses `colcon build --symlink-install`, which makes the
common edit live with no rebuild (an installed module is a symlink back into
src/) and hides the edits that are NOT:

* a **new package** has no install/ entry at all -- `ros2 run` says
  "Package not found";
* a **new entry point** in setup.cfg has no generated shim under
  install/<pkg>/lib/<pkg>/ -- "No executable found", which reads like a typo;
* a **new module file** has no symlink in the installed package dir, so the
  script that imports it dies with ModuleNotFoundError at start;
* a package built **after this console started** is not on the console's
  AMENT_PREFIX_PATH, so `ros2 run` from the console cannot see it until the
  console is restarted from a freshly sourced shell.

Each is reported per package with the command that fixes it. The scan reads
only files (package.xml, setup.cfg, directory listings) -- nothing is
imported or executed -- and covers every package under src/ that depends on
crazyflie_py, which is the same rule the console uses to find flight scripts.
"""

from __future__ import annotations

import configparser
import glob
import os
import re

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SRC = os.path.join(REPO, 'src')
INSTALL = os.path.join(REPO, 'install')
_DEPENDS = re.compile(r'<(?:exec_|build_)?depend>\s*crazyflie_py\s*<')


def show_packages():
    """{pkg: source dir} for every crazyflie_py-dependent package in src/."""
    out = {}
    for xml in glob.glob(os.path.join(SRC, '**', 'package.xml'), recursive=True):
        try:
            with open(xml, encoding='utf-8') as fh:
                body = fh.read()
        except OSError:
            continue
        if not _DEPENDS.search(body):
            continue
        m = re.search(r'<name>\s*([^<\s]+)\s*</name>', body)
        if m:
            out[m.group(1)] = os.path.dirname(xml)
    return dict(sorted(out.items()))


def _entry_points(src_dir):
    """console_scripts declared in setup.cfg (the form both show packages use)."""
    return list(_entry_targets(src_dir))


def _entry_targets(src_dir):
    """{script: dotted module} from setup.cfg's console_scripts."""
    cfg = os.path.join(src_dir, 'setup.cfg')
    if not os.path.isfile(cfg):
        return {}
    cp = configparser.ConfigParser()
    try:
        cp.read(cfg, encoding='utf-8')
        raw = cp.get('options.entry_points', 'console_scripts', fallback='')
    except configparser.Error:
        return {}
    out = {}
    for ln in raw.splitlines():
        if '=' in ln:
            name, target = (x.strip() for x in ln.split('=', 1))
            out[name] = target.split(':')[0]
    return out


def script_modules(pkg, src_dir, exe):
    """The module files a script needs from its own package: its entry module
    and that module's same-package imports, followed transitively. Read with
    ast -- never imported. Used to decide whether a NEW, not-yet-installed
    module actually breaks this script, or only some other one."""
    import ast
    target = _entry_targets(src_dir).get(exe)
    if not target or not target.startswith(pkg + '.'):
        return set()
    todo, seen = [target.split('.', 1)[1]], set()
    while todo:
        mod = todo.pop()
        if mod in seen:
            continue
        seen.add(mod)
        path = os.path.join(src_dir, pkg, mod.replace('.', os.sep) + '.py')
        try:
            with open(path, encoding='utf-8') as fh:
                tree = ast.parse(fh.read())
        except (OSError, SyntaxError):
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                base = node.module or ''
                if node.level:                       # from . import x / from .x import y
                    names = [base] if base else [a.name for a in node.names]
                elif base == pkg:                    # from crazyflie_shows import x
                    names = [a.name for a in node.names]
                elif base.startswith(pkg + '.'):     # from crazyflie_shows.x import y
                    names = [base.split('.', 1)[1]]
                else:
                    continue
                todo += [n for n in names if n]
            elif isinstance(node, ast.Import):
                todo += [a.name.split('.', 1)[1] for a in node.names if a.name.startswith(pkg + '.')]
    return {m.split('.')[0] + '.py' for m in seen}


def _installed_module_dir(pkg):
    for pat in ('lib/python3*/site-packages', 'local/lib/python3*/dist-packages',
                'lib/python3*/dist-packages'):
        for d in glob.glob(os.path.join(INSTALL, pkg, pat, pkg)):
            if os.path.isdir(d):
                return d
    return None


def package_status(pkg, src_dir):
    prefix = os.path.join(INSTALL, pkg)
    installed = os.path.isfile(os.path.join(prefix, 'share', pkg, 'package.xml'))
    on_path = prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':')
    libdir = os.path.join(prefix, 'lib', pkg)
    eps = _entry_points(src_dir)
    missing_exes = [e for e in eps if installed and not os.path.exists(os.path.join(libdir, e))]
    new_modules = []
    mod_src = os.path.join(src_dir, pkg)
    mod_dst = _installed_module_dir(pkg) if installed else None
    if mod_dst and os.path.isdir(mod_src):
        for f in sorted(os.listdir(mod_src)):
            if f.endswith('.py') and not os.path.exists(os.path.join(mod_dst, f)):
                new_modules.append(f)
    problems = []
    if not installed:
        problems.append('never built: no install/' + pkg)
    if missing_exes:
        problems.append(f'{len(missing_exes)} script(s) declared but not built: '
                        + ', '.join(missing_exes))
    if new_modules:
        problems.append(f'{len(new_modules)} new module(s) not installed: '
                        + ', '.join(new_modules))
    restart = installed and not on_path and not problems
    return {
        'pkg': pkg, 'src': os.path.relpath(src_dir, REPO), 'installed': installed,
        'on_path': on_path, 'entry_points': eps, 'missing_exes': missing_exes,
        'new_modules': new_modules, 'problems': problems,
        # built, but after this console started: it cannot `ros2 run` it yet
        'restart_console': restart,
        'built': installed and not problems,
        'build_cmd': f'./scripts/build.sh {pkg}',
    }


def status():
    """[package_status] for every show package in src/."""
    return [package_status(p, d) for p, d in show_packages().items()]


def signature():
    """Cheap change detector: setup.cfg / package dirs in src and their install."""
    sig = []
    for pkg, d in show_packages().items():
        for path in (os.path.join(d, 'setup.cfg'), os.path.join(d, pkg),
                     os.path.join(INSTALL, pkg, 'lib', pkg)):
            try:
                sig.append((path, os.path.getmtime(path)))
            except OSError:
                sig.append((path, None))
    return tuple(sig)
