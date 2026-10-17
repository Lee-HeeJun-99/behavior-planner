#!/usr/bin/env python3
"""Summarize the read-only full-bag production-filter audit and export figures."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
frames = np.genfromtxt(root / 'frames.csv', delimiter=',', names=True)
clusters = np.genfromtxt(root / 'clusters.csv', delimiter=',', names=True)
recorded = json.loads((root / 'recorded_centers.json').read_text())
topics = json.loads((root / 'topics.json').read_text())

def stats(values):
    return dict(min=float(np.min(values)), mean=float(np.mean(values)),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                max=float(np.max(values)))

summary = {
    'bag': '/home/wgr/behavior_ws/10_05_2',
    'method': 'Read-only SQLite/CDR -> actual lidar::obstacleClusters; no ROS node/DDS',
    'frames': len(frames),
    'raw_lidar_duration_s': float((frames['timestamp_ns'][-1]-frames['timestamp_ns'][0])/1e9),
    'input_points': stats(frames['input_points']),
    'finite_points': stats(frames['finite_points']),
    'roi_points': stats(frames['roi_points']),
    'generic_clusters_per_frame': stats(frames['clusters']),
    'generic_cluster_observations': int(frames['clusters'].sum()),
    'frames_with_generic_detection': int(np.sum(frames['clusters'] > 0)),
    'previous_source_gate_clusters_per_frame': stats(frames['old_gate_accepted']),
    'previous_source_gate_observations': int(frames['old_gate_accepted'].sum()),
    'newly_accepted_observations': int(frames['newly_accepted'].sum()),
    'frames_with_additional_clusters': int(np.sum(frames['newly_accepted'] > 0)),
    'processing_ms': stats(frames['processing_ms']),
    'cluster_point_count': stats(clusters['points']),
    'cluster_min_z': stats(clusters['min_z']),
    'cluster_max_z': stats(clusters['max_z']),
    'cluster_height': stats(clusters['height']),
    'clusters_entirely_below_sensor_z_zero': int(np.sum(clusters['max_z'] < 0)),
    'clusters_max_z_at_most_minus_0_2': int(np.sum(clusters['max_z'] <= -0.2)),
    'new_clusters_max_z_at_most_minus_0_2': int(np.sum((clusters['old_gate_accepted'] == 0) & (clusters['max_z'] <= -0.2))),
    'clusters_at_minimum_three_points': int(np.sum(clusters['points'] == 3)),
    'centers_finite': bool(np.all(np.isfinite(np.array([clusters['x'],clusters['y'],clusters['z']])))),
    'minimum_points_preserved': bool(np.all(clusters['points'] >= 3)),
    'all_z_bounds_inside_roi': bool(np.all((clusters['min_z'] >= -0.350001) & (clusters['max_z'] <= 1.800001))),
    'recorded_center_messages': len(recorded),
    'recorded_center_messages_nonempty': sum(r['count'] > 0 for r in recorded),
    'recorded_center_count': stats([r['count'] for r in recorded]),
    'recorded_sentinels_valid': all(r['trailing_sentinel'] for r in recorded),
    'missing_for_real_planning': [t['topic'] for t in topics if t['topic'] in
        ['/Local/utm','/Local/heading','/Convert/small_object_UTM','/Planning/behavior'] and t['count']==0],
    'interpretation': 'Observation counts are repeated detections, not distinct objects. Low z/height can be ground residual; no semantic ground truth in this bag. Previous source gate replay is not the recorded detector output.'
}
assert summary['frames'] == 1008
assert summary['centers_finite'] and summary['minimum_points_preserved'] and summary['all_z_bounds_inside_roi']
(root / 'summary.json').write_text(json.dumps(summary, indent=2))

times = (frames['timestamp_ns'] - frames['timestamp_ns'][0]) / 1e9
fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
axes[0].plot(times, frames['clusters'], label='Current generic acceptance', lw=1)
axes[0].plot(times, frames['old_gate_accepted'], label='Previous source size gate (recomputed)', lw=1)
axes[0].set_ylabel('Clusters / frame')
axes[0].legend()
axes[0].grid(alpha=.3)
axes[1].plot(times, frames['roi_points'], label='ROI points after finite + voxel + crop')
axes[1].set_ylabel('ROI points')
axes[1].set_xlabel('Time from first raw LiDAR frame (s)')
axes[1].legend()
axes[1].grid(alpha=.3)
fig.suptitle('10_05_2: all 1,008 raw frames, unchanged ROI / clustering')
fig.tight_layout()
fig.savefig(root / 'detection_timeline.png', dpi=160)
plt.close(fig)

fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharex=True, sharey=True)
for ax, frame in zip(axes, [0, 500, 1007]):
    points = np.genfromtxt(root / f'roi_{frame}.csv', delimiter=',', names=True)
    for mask, label, color in [(points['cluster'] < 0, 'Sparse noise (rejected)', '#aaaaaa'),
        (points['old_gate_accepted'] == 1, 'Accepted by both', '#0072b2'),
        (points['old_gate_accepted'] == 0, 'Additional generic clusters', '#d55e00')]:
        ax.scatter(points['x'][mask], points['y'][mask], s=9, c=color, label=label)
    ax.set_title(f'Frame {frame}, t={times[frame]:.1f}s')
    ax.set_xlabel('Sensor x (m)')
    ax.set_xlim(0,20)
    ax.set_ylim(-3.5,3.5)
    ax.grid(alpha=.3)
axes[0].set_ylabel('Sensor y (m)')
axes[0].legend(fontsize=8, loc='upper right')
fig.suptitle('ROI top view: additional detections are not semantic object labels')
fig.tight_layout()
fig.savefig(root / 'roi_comparison.png', dpi=160)
plt.close(fig)
print(json.dumps(summary, indent=2))
