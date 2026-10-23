#!/usr/bin/env python3
"""Export the report's trajectory figure from generated simulation JSON."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.1), layout='constrained')
    for number, ax in enumerate(axes, 1):
        data = json.loads((ROOT/f'build/assignment{number}.json').read_text())
        truth, estimate = np.array(data['truth']), np.array(data['estimate'])
        if number < 3:
            gps = np.array(data['measurements'])
            ax.scatter(*gps.T, s=3, alpha=.35, c='#b8853b', label='GPS')
        else:
            ax.imshow(data['height_map'], origin='lower', extent=[0, 8, 0, 15],
                      cmap='Blues', vmin=0, vmax=1, alpha=.8, aspect='equal')
        ax.plot(*truth.T, c='#2769a4', lw=1.2, label='Truth')
        ax.plot(*estimate.T, c='#168263', lw=1, label='Estimate')
        ax.set_title(['Position KF', 'Five-state EKF', 'Drone particle filter'][number-1])
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.grid(alpha=.15)
        if number == 1:
            ax.set_ylim(-1, 1)
            ax.legend(fontsize=7, loc='upper left')
        if number == 2:
            ax.set_aspect('equal')
        if number == 3:
            ax.set_xlim(0, 8)
            ax.set_ylim(0, 15)
    destination = ROOT/'docs/report/assets/trajectories.pdf'
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    main()
