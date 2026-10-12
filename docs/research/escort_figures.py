#!/usr/bin/env python3
"""Render the figures for docs/research/2026-10-12-escort-technical-briefing.md.

Every figure is computed by calling the REAL escort code (crazyflie_shows.escort
and plan_escort), so the pictures cannot drift from what flies. Re-run after
changing EscortConfig:

    python3 docs/research/escort_figures.py

No ROS needed. Writes SVGs to docs/img/escort/.
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'src', 'crazyswarm2', 'crazyflie_shows'))

from crazyflie_shows import escort, plan_escort  # noqa: E402

OUT = os.path.join(ROOT, 'docs', 'img', 'escort')
os.makedirs(OUT, exist_ok=True)

DEF = ['#1f77b4', '#2ca02c', '#9467bd']
ADV = '#d62728'
VIP = '#222222'
plt.rcParams.update({'font.size': 10, 'figure.facecolor': 'white',
                     'axes.facecolor': 'white', 'savefig.facecolor': 'white'})


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    print('wrote', os.path.relpath(path, ROOT))


def ring_panel(ax, cfg, close, lead, title, threat_deg=200.0):
    v = np.zeros(3)
    phi = np.radians(threat_deg)
    base = escort.wrap_pi(2 * np.pi * np.arange(3) / 3)
    psi = phi - base[lead] if close > 0 else 0.0
    slots = escort.ring_targets(v, psi, cfg, close, lead)
    ax.add_patch(plt.Circle((0, 0), cfg.ring_radius, fill=False, ls='--', color='0.6'))
    ax.add_patch(plt.Circle((0, 0), cfg.min_vip_dist, color='0.85', alpha=0.4, zorder=0))
    adv = np.array([np.cos(phi), np.sin(phi)]) * (cfg.ring_radius + cfg.min_adv_sep)
    ax.plot([0, 2.3 * np.cos(phi)], [0, 2.3 * np.sin(phi)], color=ADV, lw=1, ls=':')
    ax.plot(*adv, 'o', color=ADV, ms=11)
    ax.annotate('attacker', adv, xytext=(-10, -18), textcoords='offset points',
                color=ADV, ha='center')
    ax.plot(0, 0, 's', color=VIP, ms=10)
    ax.annotate('VIP', (0, 0), xytext=(8, 6), textcoords='offset points')
    for k, s in enumerate(slots):
        ax.plot(s[0], s[1], 'o', color=DEF[k], ms=11)
        tag = f'slot {k}' + (' (lead)' if close > 0 and k == lead else '')
        u = s[:2] / max(np.linalg.norm(s[:2]), 1e-6)       # label radially outward
        ax.annotate(tag, s[:2], xytext=(28 * u[0], 16 * u[1]), textcoords='offset points',
                    color=DEF[k], ha='left' if u[0] > 0.2 else 'right' if u[0] < -0.2 else 'center',
                    va='center')
    for i in range(3):
        a, b = slots[i], slots[(i + 1) % 3]
        d = np.linalg.norm(a[:2] - b[:2])
        m = (a[:2] + b[:2]) / 2
        ax.plot([a[0], b[0]], [a[1], b[1]], color='0.75', lw=0.8, zorder=0)
        ax.annotate(f'{d:.2f} m', m * 0.55, fontsize=8, color='0.4', ha='center')
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.set_xlim(-2.3, 1.6)
    ax.set_ylim(-1.6, 1.6)
    ax.set_xticks([])
    ax.set_yticks([])


def fig_ring():
    cfg = escort.EscortConfig()
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.3))
    ring_panel(axs[0], cfg, 0.0, 0, 'At rest: three slots 120 deg apart\n(gaps the attacker can see)')
    ring_panel(axs[1], cfg, 1.0, 1, 'Blocking: ring turned so the nearest slot\n'
               'sits on the threat bearing; wings close to +/-58 deg')
    save(fig, 'ring_geometry.svg')


def fig_encounter():
    cfg = escort.EscortConfig()
    adv = escort.ReactiveAdversary(height=cfg.height)
    tr = plan_escort.simulate(cfg, adv, walk_speed=0.0)
    t = tr['t']
    fig = plt.figure(figsize=(11, 4.6))
    ax = fig.add_subplot(1, 2, 1)
    c = cfg.room_center
    ax.add_patch(plt.Circle(c, cfg.arena_radius, fill=False, ls='--', color='0.7'))
    ax.annotate('arena limit', (c[0], c[1] + cfg.arena_radius), ha='center',
                xytext=(0, 4), textcoords='offset points', color='0.5', fontsize=8)
    for i in range(3):
        ax.plot(tr['def'][:, i, 0], tr['def'][:, i, 1], color=DEF[i], lw=1.3,
                label=f'defender {i}')
    ax.plot(tr['adv'][:, 0], tr['adv'][:, 1], color=ADV, lw=1.3, label='attacker')
    ax.plot(tr['adv'][0, 0], tr['adv'][0, 1], 'o', color=ADV, ms=5)
    ax.plot(tr['vip'][0, 0], tr['vip'][0, 1], 's', color=VIP, ms=8, label='VIP (static)')
    ax.set_aspect('equal')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_title('One simulated encounter, plan view\n(plan_escort, reactive attacker)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    ax2 = fig.add_subplot(2, 2, 2)
    d = np.linalg.norm(tr['adv'][:, :2] - tr['vip'][:, :2], axis=1)
    ax2.plot(t, d, color=ADV, label='attacker to VIP')
    ax2.axhline(cfg.alert_radius, color='k', ls='--', lw=0.8)
    ax2.axhline(cfg.release_radius, color='k', ls=':', lw=0.8)
    ax2.axhline(cfg.ring_radius, color='0.5', lw=0.8)
    ax2.text(t[-1], cfg.alert_radius, ' engage', va='top', fontsize=8)
    ax2.text(t[-1], cfg.release_radius, ' release', va='bottom', fontsize=8)
    ax2.text(t[-1], cfg.ring_radius, ' ring', va='bottom', fontsize=8, color='0.4')
    eng = tr['engaged'].astype(bool)
    ax2.fill_between(t, 0, 3, where=eng, color='orange', alpha=0.15, lw=0,
                     label='blocking engaged')
    ax2.set_ylim(0.5, 2.6)
    ax2.set_ylabel('distance (m)')
    ax2.legend(fontsize=8, loc='lower left')
    ax2.set_title('Hysteresis latch: engage at one radius, release at a larger one',
                  fontsize=9)
    ax3 = fig.add_subplot(2, 2, 4, sharex=ax2)
    ax3.plot(t, np.degrees(np.unwrap(tr['psi'])), color='0.2', label='ring phase psi (deg)')
    ax3b = ax3.twinx()
    ax3b.plot(t, tr['close'], color='orange', label='wall closure (0..1)')
    ax3b.set_ylim(-0.05, 1.05)
    ax3.set_xlabel('show time (s)')
    ax3.set_ylabel('psi (deg)')
    ax3b.set_ylabel('closure')
    h1, l1 = ax3.get_legend_handles_labels()
    h2, l2 = ax3b.get_legend_handles_labels()
    ax3.legend(h1 + h2, l1 + l2, fontsize=8, loc='upper left')
    ax3.set_title('Rate-limited rotation and the ramped wall', fontsize=9)
    fig.tight_layout()
    save(fig, 'encounter.svg')


def fig_race():
    cfg = escort.EscortConfig()
    R, r = cfg.ring_radius, cfg.ring_radius + cfg.min_adv_sep
    dth = np.radians(40)
    th0 = np.radians(200)
    fig, ax = plt.subplots(figsize=(5.4, 5))
    for rad, col, lab in ((R, DEF[0], 'defender'), (r, ADV, 'attacker')):
        ax.add_patch(plt.Circle((0, 0), rad, fill=False, ls='--', color='0.75'))
        a = np.linspace(th0, th0 + dth, 50)
        ax.plot(rad * np.cos(a), rad * np.sin(a), color=col, lw=4,
                label=f'{lab}: arc = {rad:.1f} m x {np.degrees(dth):.0f} deg '
                      f'= {rad * dth:.2f} m')
        ax.plot(rad * np.cos(th0), rad * np.sin(th0), 'o', color=col)
    ax.plot([0, 2.0 * np.cos(th0)], [0, 2.0 * np.sin(th0)], color='0.6', lw=0.8)
    ax.plot([0, 2.0 * np.cos(th0 + dth)], [0, 2.0 * np.sin(th0 + dth)], color='0.6', lw=0.8)
    ax.plot(0, 0, 's', color=VIP, ms=9)
    ax.annotate('VIP', (0, 0), xytext=(6, 6), textcoords='offset points')
    ax.set_aspect('equal')
    ax.set_xlim(-2.1, 0.9)
    ax.set_ylim(-1.9, 0.6)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.legend(fontsize=8, loc='lower right')
    ax.set_title('The race is ANGULAR: to cover the same bearing change,\n'
                 f'the attacker (outside) flies {r / R:.1f}x the distance')
    save(fig, 'angular_race.svg')


def fig_field():
    """The attacker's raw potential field around a resting ring, before projection."""
    cfg = escort.EscortConfig()
    v = np.zeros(3)
    defenders = escort.ring_targets(v, np.radians(90), cfg)
    xs = np.linspace(-2.4, 2.4, 61)
    X, Y = np.meshgrid(xs, xs)
    U, V = np.zeros_like(X), np.zeros_like(Y)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            a = escort.ReactiveAdversary(height=cfg.height, seek_gaps=False)
            a.p = np.array([X[i, j], Y[i, j], cfg.height])
            a._best = np.inf
            u = a._attack(v, defenders, None, 0.05)
            n = np.linalg.norm(u)
            if n > a.v_max:
                u = u * a.v_max / n
            U[i, j], V[i, j] = u
    fig, ax = plt.subplots(figsize=(6.2, 5.4))
    sp = ax.streamplot(X, Y, U, V, color=np.hypot(U, V), cmap='Greys', density=1.3,
                       linewidth=0.8, arrowsize=0.8)
    fig.colorbar(sp.lines, ax=ax, shrink=0.7, label='commanded speed (m/s)')
    ax.add_patch(plt.Circle((0, 0), cfg.ring_radius, fill=False, ls='--', color='0.6'))
    ax.add_patch(plt.Circle((0, 0), cfg.min_adv_sep, color=ADV, alpha=0.08))
    ax.annotate('hard keep-out shell\n(projection, not a force)', (0, 0),
                xytext=(0, -14), textcoords='offset points', ha='center', fontsize=7,
                color=ADV, va='top')
    for k, s in enumerate(defenders):
        ax.plot(s[0], s[1], 'o', color=DEF[k], ms=11)
    ax.plot(0, 0, 's', color=VIP, ms=9)
    ax.set_aspect('equal')
    ax.set_xlim(-2.4, 2.4)
    ax.set_ylim(-2.4, 2.4)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title('Attacker potential field around a resting ring\n'
                 '(head-on variant, before the hard projections)', fontsize=10)
    save(fig, 'potential_field.svg')


if __name__ == '__main__':
    fig_ring()
    fig_encounter()
    fig_race()
    fig_field()
